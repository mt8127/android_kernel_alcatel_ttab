/*
*
*drivers/misc/mediatek/hall/hall.c
*Created by TCTSZ-WH,2014-6-17. Hall supported.
*Modified by TCTSZ falin.luo at 2015-4-17
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/printk.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <cust_eint.h>
#include "hall.h"

#define CUST_EINT_POLARITY_LOW 0

static u8 hall_state = !CUST_EINT_POLARITY_LOW;
static struct input_dev *hall_input = NULL;
static struct hall_data *hall = NULL;
static struct class *hall_class = NULL;
static struct device *hall_dev = NULL;
static struct platform_device *hall_pdev = NULL;
extern unsigned int bl_brightness;


static void do_hall_work(struct work_struct *work)
{
	u8 old_state = hall_state;
	bool pressed;
	
	mt_eint_mask(CUST_EINT_MHALL_NUM);
	
	pr_info("%s",__func__);

	hall_state = !hall_state;
	pressed = (hall_state == !!CUST_EINT_POLARITY_LOW);

	if((pressed == 1) && (bl_brightness != 0)) {
		input_report_key(hall_input, KEY_POWER, pressed);
		input_sync(hall_input);
		input_report_key(hall_input, KEY_POWER, !pressed);
		input_sync(hall_input);
	}

	if((pressed == 0) && (bl_brightness == 0)) {
		input_report_key(hall_input, KEY_POWER, !pressed);
		input_sync(hall_input);
		input_report_key(hall_input, KEY_POWER, pressed);
		input_sync(hall_input);
	}

	mt_eint_set_polarity(CUST_EINT_MHALL_NUM, old_state);
	mt_eint_unmask(CUST_EINT_MHALL_NUM);
}

static void interrupt_hall_irq(void)
{
//	pr_err("Hall sensor irq handled, gpio_value = %d\n", mt_get_gpio_in(GPIO_MHALL_EINT_PIN));
	queue_work(hall->hall_wq, &hall->hall_work);
}


int hall_probe(struct platform_device *pdev)
{
	int rc = 0;
	
	hall = kmalloc(sizeof(struct hall_data),GFP_KERNEL);
	
	if (hall == NULL) {
		pr_err("%s:%d Unable to allocate memory\n",__func__, __LINE__);
		return -ENOMEM;
	}

	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_GPIO);	
	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_UP);
	 
	mt_eint_set_sens(CUST_EINT_MHALL_NUM, MT_LEVEL_SENSITIVE);
	mt_eint_set_polarity(CUST_EINT_MHALL_NUM, CUST_EINT_POLARITY_LOW);
	mt_eint_set_hw_debounce(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_TYPE, interrupt_hall_irq, 0); 
	mt_eint_unmask(CUST_EINT_MHALL_NUM);

	hall_input = input_allocate_device();
	if (!hall_input) {
		pr_err("%s:%d Not enough memory!\n", __func__, __LINE__);
		return -ENOMEM;
	}

	hall_input->name = "hall";
	input_set_capability(hall_input, EV_KEY, KEY_POWER);

	rc = input_register_device(hall_input);
	if (rc)
	{
		pr_err("Failed to register hall input device\n");
		return rc;
	}

	hall->hall_wq = create_singlethread_workqueue("hall_wq");
	INIT_WORK(&hall->hall_work, do_hall_work);

	hall_class = class_create(THIS_MODULE, "hall");
	hall_dev = device_create(hall_class, NULL, 0, NULL, "cover");
	
	pr_info("Hall Sensor Driver Probe Success!\n");
	
	return 0;
}

int hall_remove(struct platform_device *pdev)
{
	if (hall_dev)
		device_del(hall_dev);
	if (hall_class)
		class_destroy(hall_class);
	
	input_unregister_device(hall_input);
	
	if (hall_input)
	{
		input_free_device(hall_input);
		hall_input = NULL;
	}

	kfree(hall);

	return 0;
}

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.remove = hall_remove,
	.driver = {
		   .name = "hall_sensor",
		   .owner = THIS_MODULE,
	}
};

static int __init hall_init(void)
{
	int ret = platform_driver_register(&hall_driver);
	
	if (ret) 
		return ret;
	
	hall_pdev = platform_device_register_simple("hall_sensor", -1, NULL, 0);
	
	if (IS_ERR(hall_pdev)) {
		platform_driver_unregister(&hall_driver);
		return PTR_ERR(hall_pdev);
	}

	pr_info("Hall Sensor Driver Initializ Success.\n");

	return 0;
}

static void __init hall_exit(void)
{
	platform_device_unregister(hall_pdev);
	platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_LICENSE("GPL");

