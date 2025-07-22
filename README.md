# BME680 Environmental Sensor Driver

This driver provides a comprehensive solution for interfacing with the BME680 environmental sensor, which measures temperature, pressure, and humidity. It exposes sensor data and control through a character device and leverages the I2C subsystem for communication with the BME680 sensor. Key features include dynamic memory management, interrupt handling for real-time alerts, workqueue-based processing, and Device Tree integration for platform-agnostic hardware configuration.

### Features

*   **BME680 Sensor Integration**: Fully implements I2C communication with the BME680 environmental sensor to read temperature, pressure, and humidity data.
*   **Character Device Interface**: Exposes a character device (`/dev/multi_sensor_driver`) for user-space applications to read formatted sensor data and configure the driver.
*   **I2C Communication**: Implements low-level I2C bus communication with register-level access to the BME680 sensor.
*   **GPIO and Interrupt Handling**: Manages GPIO pins for controlling LEDs and handling interrupts for temperature threshold-based alerts.
*   **Workqueue Implementation**: Uses kernel workqueues to handle sensor reading and threshold checking in interrupt context.
*   **Extended ioctl Interface**: Implements multiple `ioctl` commands to allow user-space applications to:
    - Set temperature thresholds for alerts
    - Directly access raw temperature data
    - Directly access raw pressure data
    - Directly access raw humidity data
*   **Device Tree Integration**: Retrieves hardware configuration (e.g., GPIO pins) from the Device Tree, making the driver portable across different platforms.
*   **Dynamic Memory Management**: Uses `kmalloc`, `kfree`, and `devm_kzalloc` for efficient memory management of private data structures.

## Getting Started

### Prerequisites

- A Linux system with kernel headers (e.g., Raspberry Pi OS)
- A C compiler (e.g., `gcc`)
- `make`

### Building the Driver

To build the driver, run the following command in the project directory:

```sh
make
```

This will produce a `multi_sensor_driver.ko` file, which is the compiled kernel module.

### Loading the Driver

To load the driver into the kernel:

```sh
sudo insmod multi_sensor_driver.ko
```

To unload the driver:

```sh
sudo rmmod multi_sensor_driver
```

### Using the Driver

#### Reading Sensor Data

After loading the driver, you can read formatted sensor data from the device file:

```sh
cat /dev/multi_sensor_driver
```

This will output temperature, pressure, and humidity readings in a human-readable format.

#### Setting Thresholds and Accessing Raw Data

You can set temperature thresholds and access raw sensor data using the ioctl interface. Here's a sample C program demonstrating these capabilities:

```c
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define SET_THRESHOLD _IOW('a', 1, int)
#define GET_TEMPERATURE _IOR('a', 2, int)
#define GET_PRESSURE _IOR('a', 3, int)
#define GET_HUMIDITY _IOR('a', 4, int)

int main() {
    int fd = open("/dev/multi_sensor_driver", O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }
    
    // Set temperature threshold to 30°C
    int threshold = 30;
    if (ioctl(fd, SET_THRESHOLD, &threshold) < 0) {
        perror("Failed to set threshold");
    }
    
    // Read raw temperature data
    int temperature;
    if (ioctl(fd, GET_TEMPERATURE, &temperature) < 0) {
        perror("Failed to get temperature");
    } else {
        printf("Temperature: %d.%02d °C\n", temperature / 100, temperature % 100);
    }
    
    // Read raw pressure data
    unsigned int pressure;
    if (ioctl(fd, GET_PRESSURE, &pressure) < 0) {
        perror("Failed to get pressure");
    } else {
        printf("Pressure: %u.%02u hPa\n", pressure / 100, pressure % 100);
    }
    
    // Read raw humidity data
    unsigned int humidity;
    if (ioctl(fd, GET_HUMIDITY, &humidity) < 0) {
        perror("Failed to get humidity");
    } else {
        printf("Humidity: %u.%03u %%RH\n", humidity / 1000, humidity % 1000);
    }
    
    close(fd);
    return 0;
}
```