
#include <linux/module.h>    /* for all kernel modules */
#include <linux/kernel.h>    /* for KERN_INFO */
#include <linux/init.h>      /* for __init and __exit macros */


static int __init kernel_module_init(void)
{
	printk(KERN_INFO "Kernel hello world: hello world from EmbeddedCrab\n");
	return 0;
}

static void __exit kernel_module_exit(void)
{
	printk(KERN_INFO "Kernel hello world: goodbye from EmbeddedCrab\n");
}

module_init(kernel_module_init);
module_exit(kernel_module_exit);

MODULE_DESCRIPTION("Hello World Kernel Driver");
MODULE_AUTHOR("Hemant Sharma <hemant.sharma@embeddedcrab.com>");
MODULE_LICENSE("GPL v2");