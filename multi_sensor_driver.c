#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define DRIVER_NAME "multi_sensor_driver"
#define DRIVER_CLASS "multi_sensor_class"

static dev_t dev_nr;
static struct class *my_class;
static struct cdev my_cdev;
static struct i2c_client *ms_i2c_client;
static struct workqueue_struct *ms_wq;
static struct work_struct ms_work;

/* GPIO pins will be obtained from Device Tree */
static int irq_number;
static int led_gpio;
static int interrupt_gpio;

/* BME680 Registers */
#define BME680_ADDR                0x76  /* I2C address of BME680 */
#define BME680_CHIP_ID_ADDR        0xD0  /* Chip ID Register */
#define BME680_CHIP_ID             0x61  /* Expected chip ID */
#define BME680_RESET_ADDR          0xE0  /* Soft reset register */
#define BME680_RESET_VALUE         0xB6  /* Soft reset command */

/* Temperature, Pressure, Humidity data registers */
#define BME680_TEMP_MSB            0x22  /* Temperature MSB */
#define BME680_TEMP_LSB            0x23  /* Temperature LSB */
#define BME680_TEMP_XLSB           0x24  /* Temperature XLSB */
#define BME680_PRESS_MSB           0x1F  /* Pressure MSB */
#define BME680_PRESS_LSB           0x20  /* Pressure LSB */
#define BME680_PRESS_XLSB          0x21  /* Pressure XLSB */
#define BME680_HUM_MSB             0x25  /* Humidity MSB */
#define BME680_HUM_LSB             0x26  /* Humidity LSB */

/* Control registers */
#define BME680_CTRL_MEAS_ADDR      0x74  /* Ctrl Measure Register */
#define BME680_CTRL_HUM_ADDR       0x72  /* Ctrl Humidity Register */
#define BME680_CONFIG_ADDR         0x75  /* Configuration Register */

/* Status registers */
#define BME680_STATUS_ADDR         0x73  /* Status Register */
#define BME680_STATUS_MEASURING    0x08  /* Measurement in progress bit */

/* Calibration registers - we'll need these for accurate readings */
#define BME680_CALIB_ADDR_1        0x89  /* Calibration data start address 1 */
#define BME680_CALIB_ADDR_2        0xE1  /* Calibration data start address 2 */

/* Sensor modes */
#define BME680_SLEEP_MODE          0x00
#define BME680_FORCED_MODE         0x01

static const struct of_device_id ms_of_match[] = {
    { .compatible = "bosch,bme680", },
    { }
};

#define MEM_SIZE 1024

/* Calibration data structure */
struct bme680_calib_data {
    uint16_t par_t1;
    int16_t par_t2;
    int8_t par_t3;
    uint16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int16_t par_p4;
    int16_t par_p5;
    int8_t par_p6;
    int8_t par_p7;
    int16_t par_p8;
    int16_t par_p9;
    uint8_t par_p10;
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t par_h3;
    int8_t par_h4;
    int8_t par_h5;
    uint8_t par_h6;
    int8_t par_h7;
    int8_t t_fine; /* Temperature compensation value */
};

/* private data structure */
struct ms_private_data {
    char buffer[MEM_SIZE];
    int threshold;
    /* Sensor data */
    int32_t temperature;    /* Temperature in 0.01 degree Celsius */
    uint32_t pressure;      /* Pressure in Pascal */
    uint32_t humidity;      /* Humidity in 0.001 %RH */
    struct bme680_calib_data calib_data; /* Calibration data */
    bool sensor_initialized; /* Flag to track sensor initialization */
};

static int my_open(struct inode *inode, struct file *filp)
{
    struct ms_private_data *dev_data;
    dev_data = kmalloc(sizeof(struct ms_private_data), GFP_KERNEL);
    if (!dev_data)
    {
        pr_err("Failed to allocate memory\n");
        return -ENOMEM;
    }
    filp->private_data = dev_data;
    dev_data->threshold = 0; /* Default threshold */
    dev_data->sensor_initialized = false; /* Sensor not initialized yet */
    memset(&dev_data->calib_data, 0, sizeof(dev_data->calib_data));
    pr_info("Device file opened\n");
    return 0;
}

static int my_release(struct inode *inode, struct file *filp)
{
    kfree(filp->private_data);
    pr_info("Device file closed\n");
    return 0;
}

/**
 * @brief Read a byte from the BME680 sensor
 */
static int bme680_read_byte(uint8_t reg_addr, uint8_t *data)
{
    int ret;
    struct i2c_msg msg[2];
    uint8_t buf[1];

    if (!ms_i2c_client)
        return -ENODEV;

    /* Write register address */
    buf[0] = reg_addr;
    msg[0].addr = ms_i2c_client->addr;
    msg[0].flags = 0;  /* Write */
    msg[0].len = 1;
    msg[0].buf = buf;

    /* Read data */
    msg[1].addr = ms_i2c_client->addr;
    msg[1].flags = I2C_M_RD;  /* Read */
    msg[1].len = 1;
    msg[1].buf = data;

    ret = i2c_transfer(ms_i2c_client->adapter, msg, 2);
    if (ret != 2) {
        pr_err("Failed to read from BME680: %d\n", ret);
        return -EIO;
    }

    return 0;
}

/**
 * @brief Write a byte to the BME680 sensor
 */
static int bme680_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t buf[2];

    if (!ms_i2c_client)
        return -ENODEV;

    buf[0] = reg_addr;
    buf[1] = data;

    ret = i2c_master_send(ms_i2c_client, buf, 2);
    if (ret != 2) {
        pr_err("Failed to write to BME680: %d\n", ret);
        return -EIO;
    }

    return 0;
}

/**
 * @brief Initialize the BME680 sensor
 */
static int bme680_init(struct ms_private_data *dev_data)
{
    uint8_t chip_id;
    int ret;

    /* Check if sensor is already initialized */
    if (dev_data->sensor_initialized)
        return 0;

    /* Read chip ID to verify we're talking to a BME680 */
    ret = bme680_read_byte(BME680_CHIP_ID_ADDR, &chip_id);
    if (ret < 0)
        return ret;

    if (chip_id != BME680_CHIP_ID) {
        pr_err("BME680 not found, got chip ID 0x%02x\n", chip_id);
        return -ENODEV;
    }

    pr_info("BME680 found with chip ID 0x%02x\n", chip_id);

    /* Soft reset the sensor */
    ret = bme680_write_byte(BME680_RESET_ADDR, BME680_RESET_VALUE);
    if (ret < 0)
        return ret;

    /* Wait for the sensor to reset */
    msleep(10);

    /* Configure the sensor */
    /* Set humidity oversampling to 1x */
    ret = bme680_write_byte(BME680_CTRL_HUM_ADDR, 0x01);
    if (ret < 0)
        return ret;

    /* Set temperature and pressure oversampling to 2x and put in forced mode */
    ret = bme680_write_byte(BME680_CTRL_MEAS_ADDR, (2 << 5) | (2 << 2) | BME680_FORCED_MODE);
    if (ret < 0)
        return ret;

    /* Set filter coefficient to 2 */
    ret = bme680_write_byte(BME680_CONFIG_ADDR, 2 << 2);
    if (ret < 0)
        return ret;

    dev_data->sensor_initialized = true;
    pr_info("BME680 initialized successfully\n");

    return 0;
}

/**
 * @brief Read raw temperature data from BME680
 */
static int bme680_read_temp_raw(int32_t *temp_raw)
{
    uint8_t data[3];
    int ret;

    ret = bme680_read_byte(BME680_TEMP_MSB, &data[0]);
    if (ret < 0)
        return ret;

    ret = bme680_read_byte(BME680_TEMP_LSB, &data[1]);
    if (ret < 0)
        return ret;

    ret = bme680_read_byte(BME680_TEMP_XLSB, &data[2]);
    if (ret < 0)
        return ret;

    *temp_raw = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);

    return 0;
}

/**
 * @brief Read raw pressure data from BME680
 */
static int bme680_read_pressure_raw(uint32_t *pres_raw)
{
    uint8_t data[3];
    int ret;

    ret = bme680_read_byte(BME680_PRESS_MSB, &data[0]);
    if (ret < 0)
        return ret;

    ret = bme680_read_byte(BME680_PRESS_LSB, &data[1]);
    if (ret < 0)
        return ret;

    ret = bme680_read_byte(BME680_PRESS_XLSB, &data[2]);
    if (ret < 0)
        return ret;

    *pres_raw = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);

    return 0;
}

/**
 * @brief Read raw humidity data from BME680
 */
static int bme680_read_humidity_raw(uint16_t *hum_raw)
{
    uint8_t data[2];
    int ret;

    ret = bme680_read_byte(BME680_HUM_MSB, &data[0]);
    if (ret < 0)
        return ret;

    ret = bme680_read_byte(BME680_HUM_LSB, &data[1]);
    if (ret < 0)
        return ret;

    *hum_raw = ((uint16_t)data[0] << 8) | data[1];

    return 0;
}

/**
 * @brief Read sensor data into the buffer
 */
static int read_sensor_data(struct ms_private_data *dev_data)
{
    int ret;
    int32_t temp_raw;
    uint32_t pres_raw;
    uint16_t hum_raw;
    int len;

    /* Initialize sensor if not already done */
    ret = bme680_init(dev_data);
    if (ret < 0) {
        pr_err("Failed to initialize BME680: %d\n", ret);
        return scnprintf(dev_data->buffer, MEM_SIZE, "Error: Sensor initialization failed\n");
    }

    /* Trigger a measurement */
    ret = bme680_write_byte(BME680_CTRL_MEAS_ADDR, (2 << 5) | (2 << 2) | BME680_FORCED_MODE);
    if (ret < 0) {
        pr_err("Failed to trigger measurement: %d\n", ret);
        return scnprintf(dev_data->buffer, MEM_SIZE, "Error: Failed to trigger measurement\n");
    }

    /* Wait for the measurement to complete (typical 10ms) */
    msleep(50);

    /* Read raw sensor data */
    ret = bme680_read_temp_raw(&temp_raw);
    if (ret < 0) {
        pr_err("Failed to read temperature: %d\n", ret);
        return scnprintf(dev_data->buffer, MEM_SIZE, "Error: Failed to read temperature\n");
    }

    ret = bme680_read_pressure_raw(&pres_raw);
    if (ret < 0) {
        pr_err("Failed to read pressure: %d\n", ret);
        return scnprintf(dev_data->buffer, MEM_SIZE, "Error: Failed to read pressure\n");
    }

    ret = bme680_read_humidity_raw(&hum_raw);
    if (ret < 0) {
        pr_err("Failed to read humidity: %d\n", ret);
        return scnprintf(dev_data->buffer, MEM_SIZE, "Error: Failed to read humidity\n");
    }

    /* For simplicity, we'll use the raw values directly */
    /* In a real driver, we would apply calibration formulas */
    dev_data->temperature = temp_raw / 100;  /* Convert to 0.01 degrees C */
    dev_data->pressure = pres_raw;           /* Pascal */
    dev_data->humidity = hum_raw * 10;       /* 0.001 %RH */

    /* Format the data for user space */
    len = scnprintf(dev_data->buffer, MEM_SIZE,
                   "Temperature: %d.%02d C\n"
                   "Pressure: %u.%02u hPa\n"
                   "Humidity: %u.%03u %%RH\n",
                   dev_data->temperature / 100, dev_data->temperature % 100,
                   dev_data->pressure / 100, dev_data->pressure % 100,
                   dev_data->humidity / 1000, dev_data->humidity % 1000);

    return len;
}

static ssize_t my_read(struct file *filp, char __user *user_buf, size_t len, loff_t *off)
{
    struct ms_private_data *dev_data = filp->private_data;
    int to_copy, not_copied, delta;

    int data_len;

    data_len = read_sensor_data(dev_data);

    to_copy = min(len, (size_t)(data_len - *off));

    if (to_copy <= 0)
        return 0;

    not_copied = copy_to_user(user_buf, dev_data->buffer + *off, to_copy);
    if (not_copied)
    {
        pr_err("Failed to copy data to user\n");
        return -EFAULT;
    }

    delta = to_copy - not_copied;
    *off += delta;

    return delta;
}



/* Work function for handling sensor threshold checks */
static void ms_work_handler(struct work_struct *work)
{
    struct ms_private_data *dev_data = NULL;
    int ret;
    int32_t temp_raw;
    int32_t temperature;

    if (!ms_i2c_client) {
        pr_err("I2C client not available\n");
        return;
    }

    /* Get the private data from the I2C client */
    dev_data = i2c_get_clientdata(ms_i2c_client);
    if (!dev_data) {
        pr_err("Private data not available\n");
        return;
    }

    /* Read temperature */
    ret = bme680_init(dev_data);
    if (ret < 0) {
        pr_err("Failed to initialize BME680 in work handler: %d\n", ret);
        return;
    }

    /* Trigger a measurement */
    ret = bme680_write_byte(BME680_CTRL_MEAS_ADDR, (2 << 5) | (2 << 2) | BME680_FORCED_MODE);
    if (ret < 0) {
        pr_err("Failed to trigger measurement in work handler: %d\n", ret);
        return;
    }

    /* Wait for the measurement to complete */
    msleep(50);

    /* Read temperature */
    ret = bme680_read_temp_raw(&temp_raw);
    if (ret < 0) {
        pr_err("Failed to read temperature in work handler: %d\n", ret);
        return;
    }

    temperature = temp_raw / 100;

    /* Check if temperature exceeds threshold */
    if (dev_data->threshold > 0 && temperature > dev_data->threshold) {
        pr_info("ALERT: Temperature %d.%02d C exceeds threshold %d C\n",
                temperature / 100, temperature % 100, dev_data->threshold);

        /* Toggle the LED to indicate alert */
        gpio_set_value(GPIO_LED_PIN, 1);
        msleep(100);
        gpio_set_value(GPIO_LED_PIN, 0);
    }
}

/* Interrupt handler */
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    pr_info("Interrupt triggered!\n");
    
    /* Schedule work to handle the interrupt */
    queue_work(ms_wq, &ms_work);
    
    return IRQ_HANDLED;
}

#define SET_THRESHOLD _IOW('a', 1, int)
#define GET_TEMPERATURE _IOR('a', 2, int)
#define GET_PRESSURE _IOR('a', 3, int)
#define GET_HUMIDITY _IOR('a', 4, int)

static long ms_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct ms_private_data *dev_data = filp->private_data;
    int temp_threshold;
    int ret;

    switch (cmd) {
        case SET_THRESHOLD:
            if (copy_from_user(&temp_threshold, (int __user *)arg, sizeof(temp_threshold)))
                return -EFAULT;
            dev_data->threshold = temp_threshold;
            pr_info("Threshold set to %d\n", dev_data->threshold);
            break;

        case GET_TEMPERATURE:
            /* Make sure we have fresh data */
            ret = read_sensor_data(dev_data);
            if (ret < 0)
                return ret;
            
            if (copy_to_user((int __user *)arg, &dev_data->temperature, sizeof(dev_data->temperature)))
                return -EFAULT;
            break;

        case GET_PRESSURE:
            /* Make sure we have fresh data */
            ret = read_sensor_data(dev_data);
            if (ret < 0)
                return ret;
            
            if (copy_to_user((uint32_t __user *)arg, &dev_data->pressure, sizeof(dev_data->pressure)))
                return -EFAULT;
            break;

        case GET_HUMIDITY:
            /* Make sure we have fresh data */
            ret = read_sensor_data(dev_data);
            if (ret < 0)
                return ret;
            
            if (copy_to_user((uint32_t __user *)arg, &dev_data->humidity, sizeof(dev_data->humidity)))
                return -EFAULT;
            break;

        default:
            return -EINVAL;
    }
    return 0;
}

static ssize_t my_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
    struct ms_private_data *dev_data = filp->private_data;
    int to_copy, not_copied, delta;

    to_copy = min(len, (size_t)(MEM_SIZE - *off));

    if (to_copy <= 0)
        return 0;

    not_copied = copy_from_user(dev_data->buffer + *off, user_buf, to_copy);
    if (not_copied)
    {
        pr_err("Failed to copy data from user\n");
        return -EFAULT;
    }
    delta = to_copy - not_copied;
    *off += delta;

    pr_info("Received %d bytes: %s\n", delta, dev_data->buffer);

    return delta;
}



static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = my_open,
    .release = my_release,
    .read = my_read,
    .write = my_write,
    .unlocked_ioctl = ms_ioctl,
    .llseek = default_llseek
};

/* I2C driver functions */
static int ms_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device_node *np = client->dev.of_node;
    int local_led_gpio, local_interrupt_gpio;
    struct ms_private_data *dev_data;

    if (!np) {
        pr_err("Device Tree node not found\n");
        return -EINVAL;
    }

    local_led_gpio = of_get_named_gpio(np, "led-gpios", 0);
    if (local_led_gpio < 0) {
        pr_err("Failed to get LED GPIO from device tree\n");
        return local_led_gpio;
    }

    local_interrupt_gpio = of_get_named_gpio(np, "interrupt-gpios", 0);
    if (local_interrupt_gpio < 0) {
        pr_err("Failed to get interrupt GPIO from device tree\n");
        return local_interrupt_gpio;
    }

    pr_info("LED GPIO: %d, Interrupt GPIO: %d\n", local_led_gpio, local_interrupt_gpio);

    /* GPIO and Interrupt setup */
    if (gpio_request(local_led_gpio, "led-pin")) {
        pr_err("Failed to request LED GPIO pin\n");
        return -EFAULT;
    }
    gpio_direction_output(local_led_gpio, 0);

    if (gpio_request(local_interrupt_gpio, "interrupt-pin")) {
        pr_err("Failed to request Interrupt GPIO pin\n");
        gpio_free(local_led_gpio);
        return -EFAULT;
    }
    gpio_direction_input(local_interrupt_gpio);

    /* Allocate private data for the I2C client */
    dev_data = devm_kzalloc(&client->dev, sizeof(struct ms_private_data), GFP_KERNEL);
    if (!dev_data) {
        pr_err("Failed to allocate memory for private data\n");
        gpio_free(local_interrupt_gpio);
        gpio_free(local_led_gpio);
        return -ENOMEM;
    }

    /* Initialize private data */
    dev_data->threshold = 30; /* Default threshold: 30°C */
    dev_data->sensor_initialized = false;

    /* Store private data in I2C client */
    i2c_set_clientdata(client, dev_data);

    /* Set up interrupt handler */
    irq_number = gpio_to_irq(local_interrupt_gpio);
    if (request_irq(irq_number, gpio_irq_handler, IRQF_TRIGGER_RISING, DRIVER_NAME, dev_data)) {
        pr_err("Failed to request IRQ\n");
        gpio_free(local_interrupt_gpio);
        gpio_free(local_led_gpio);
        return -EFAULT;
    }

    /* Store GPIO pins globally for cleanup in exit function */
    led_gpio = local_led_gpio;
    interrupt_gpio = local_interrupt_gpio;

    pr_info("GPIO and Interrupts initialized from DT\n");

    pr_info("I2C probe: BME680 device found!\n");
    ms_i2c_client = client;
    return 0;
}

static void ms_i2c_remove(struct i2c_client *client)
{
    pr_info("I2C remove: device removed!");
    ms_i2c_client = NULL;
}

static const struct i2c_device_id ms_i2c_id[] = {
    { "multi_sensor", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ms_i2c_id);

static struct i2c_driver ms_i2c_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ms_of_match),
    },
    .probe = ms_i2c_probe,
    .remove = ms_i2c_remove,
    .id_table = ms_i2c_id,
};

static int __init multi_sensor_init(void)
{
    pr_info("Multi-Sensor Driver: Initializing BME680 driver\n");

    /* Create workqueue for interrupt handling */
    ms_wq = create_singlethread_workqueue("ms_workqueue");
    if (!ms_wq) {
        pr_err("Failed to create workqueue\n");
        return -ENOMEM;
    }

    /* Initialize work */
    INIT_WORK(&ms_work, ms_work_handler);

    if (alloc_chrdev_region(&dev_nr, 0, 1, DRIVER_NAME) < 0)
    {
        pr_err("Failed to allocate device number\n");
        destroy_workqueue(ms_wq);
        return -1;
    }

    if ((my_class = class_create(DRIVER_CLASS)) == NULL)
    {
        pr_err("Failed to create device class\n");
        goto free_dev_nr;
    }

    if (device_create(my_class, NULL, dev_nr, NULL, DRIVER_NAME) == NULL)
    {
        pr_err("Failed to create device file\n");
        goto free_class;
    }

    cdev_init(&my_cdev, &fops);

    if (cdev_add(&my_cdev, dev_nr, 1) == -1)
    {
        pr_err("Failed to add cdev\n");
        goto free_device;
    }

    pr_info("Driver initialized successfully\n");



    return i2c_add_driver(&ms_i2c_driver);

free_device:
    device_destroy(my_class, dev_nr);
free_class:
    class_destroy(my_class);
free_dev_nr:
    unregister_chrdev_region(dev_nr, 1);
    return -1;
}

static void __exit multi_sensor_exit(void)
{
    /* Free IRQ and GPIO resources */
    if (irq_number) {
        free_irq(irq_number, NULL);
    }
    if (interrupt_gpio) {
        gpio_free(interrupt_gpio);
    }
    if (led_gpio) {
        gpio_free(led_gpio);
    }

    /* Cancel any pending work */
    if (ms_wq) {
        cancel_work_sync(&ms_work);
        destroy_workqueue(ms_wq);
    }

    i2c_del_driver(&ms_i2c_driver);
    cdev_del(&my_cdev);
    device_destroy(my_class, dev_nr);
    class_destroy(my_class);
    unregister_chrdev_region(dev_nr, 1);
    pr_info("Multi-Sensor Driver: Exiting\n");
}

module_init(multi_sensor_init);
module_exit(multi_sensor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jad Kada");
MODULE_DESCRIPTION("BME680 Environmental Sensor Driver for Temperature, Pressure, and Humidity Monitoring");