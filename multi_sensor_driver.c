#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>

static int __init multi_sensor_init(void)
{
    pr_info("Multi-Sensor Driver: Initializing\n");
    return 0;
}

static void __exit multi_sensor_exit(void)
{
    pr_info("Multi-Sensor Driver: Exiting\n");
}

module_init(multi_sensor_init);
module_exit(multi_sensor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Multi-Sensor Environmental Monitoring System Driver");