/*
 *  Opal H2W device detection driver.
 *
 * Copyright (C) 2008 TCL&Alcatel Corporation.

 *
 * Authors:
 *  Fuzhen Lin<fuzhen.lin@tct-sh.com>
 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


/*  For detecting OPAL  2 Wire devices, such as wired headset.

    Logically, the H2W driver is always present, and H2W state (hi->state)
    indicates what is currently plugged into the H2W interface.

    When the headset is plugged in, CABLE_IN1(GPIO92)  is pulled low. 

    


    Headset insertion/removal causes UEvent's to be sent, and
    /sys/class/switch/h2w/state to be updated.

    We tend to check the status of CABLE_IN1 a few more times than strictly
    necessary during headset detection, to avoid spurious headset insertion
    events caused by serial debugger TX traffic.
*/


#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <asm/atomic.h>
#include <mach/board.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include "board-sapphire.h"
#include <mach/pmic.h>

#include <linux/time.h>

#ifdef CONFIG_DEBUG_OPAL_H2W
#define H2W_DBG(fmt, arg...) printk(KERN_INFO "[H2W] %s " fmt "\n", __FUNCTION__, ## arg)
#else
#define H2W_DBG(fmt, arg...) do {} while (0)
#endif

#define AUDIO_CABLE_IN (92)

int gpio92status;
static struct timer_list jrd_headset_timer;   // Liaosicong add for headset plug detection 2010-10-15
static struct workqueue_struct *g_detection_work_queue;
static void detection_work(struct work_struct *work);
static void jrd_headset_work(struct work_struct *work); // Liaosicong add for headset plug detection 2010-10-15
static DECLARE_WORK(g_detection_work, detection_work);
DECLARE_DELAYED_WORK(jrd_headset_workqueue, jrd_headset_work); // Liaosicong add for headset plug detection 2010-10-15

enum {
	NO_DEVICE	= 0,
	OPAL_HEADSET	= 1,
};

enum {
	UART3		= 0,
	GPIO		= 1,
};

struct h2w_info {
	struct switch_dev sdev;
	struct input_dev *input;

	
	unsigned int irq;
	
};
static struct h2w_info *hi;

static ssize_t Opal_h2w_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hi->sdev)) {
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case OPAL_HEADSET:
		return sprintf(buf, "Headset\n");
	}
	return -EINVAL;
}



#ifdef CONFIG_MSM_SERIAL_DEBUGGER
extern void msm_serial_debug_enable(int);
#endif

static void insert_headset(void)
{
	int enflag =0;
        enum mic_volt voltage;//add by flin
#ifdef PMIC_MIC_BIAS_CTRL
	H2W_DBG("");
//================begin for disable pm_mic_bias===============
        if(0!=pmic_mic_en(false)){
                printk("@@@@mic disable ERR\r\n");
        }
 
        if(pmic_mic_is_en(&enflag)!=0){
                printk("@@@pmic_is_en ERR\r\n");
        }
        else{
                printk("@@@Read pmic_mic_en = %d\r\n",enflag);
        }
 
        printk("@@@@@@@@switch to Headset@@@@@@@\r\n");
        if(0!=pmic_mic_get_volt(&voltage)){
                printk("@@@@@@@ERR-get get the PM_MIC_BIAS_voltage");
        }else{
                printk("@@@@@IN MIC2 Read PM_MIC_BIAS_VOLTAGE=%d\r\n",voltage);
        }
//==================end for disable pm_mic_bias====================
#endif
	printk("============insert_headset==============\r\n");
	switch_set_state(&hi->sdev, OPAL_HEADSET);
	
#ifdef CONFIG_MSM_SERIAL_DEBUGGER
	msm_serial_debug_enable(false);
#endif



	
}

static void remove_headset(void)
{
#ifdef PMIC_MIC_BIAS_CTRL
//=============begin to enable pm_mic_bias====================================
        int enflag =0;
	enum mic_volt voltage;//add by flin
	if(0!=pmic_mic_en(true)){
             printk("@@@@mic disable ERR\r\n");
        }
 
        if(pmic_mic_is_en(&enflag)!=0){
             printk("@@@pmic_is_en ERR\r\n");
        }
        else{
              printk("@@@Read pmic_mic_en = %d\r\n",enflag);
        }
        printk("@@@@@@@@switch to Handset@@@@@@@\r\n");

        //voltage = MIC_VOLT_1_93V;
        if(0!= pmic_mic_set_volt(MIC_VOLT_1_93V)){
              printk("@@@@could not set the MIC_BIAS=1.93V\r\n");
        }else{
              printk("@@@@@set PM_MIC_BIAS=1.93V\r\n");
        }
 
        if(0!=pmic_mic_get_volt(&voltage)){
              printk("@@@@@@@ERR-get get the PM_MIC_BIAS_voltage");
        }else{
              printk("@@@@@IN MIC1 Read PM_MIC_BIAS_VOLTAGE=%d\r\n",voltage);
        }
//=====================end to enable pm_mic_bias=================================
//	H2W_DBG("");
#endif
	printk("============remove_headset==============\r\n");
	switch_set_state(&hi->sdev, NO_DEVICE);
	
}
void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable);
//flin added
static uint32_t msm_earphone_detect_gpio_table[] = {
	GPIO_CFG(AUDIO_CABLE_IN, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA)
};
/*Liaosicong add for headset plug timer delay detection 2010-10-15*/
static void jrd_headset_work(struct work_struct *work)
{
	int  cable_in1;

	cable_in1 = gpio_get_value(AUDIO_CABLE_IN);

	if (cable_in1 !=0 && gpio92status ==0){
		/* Headset not plugged in */
		if (switch_get_state(&hi->sdev) == OPAL_HEADSET)
			remove_headset();
	}
	   

	if (cable_in1 == 0&& gpio92status == 1 ) {
		if (switch_get_state(&hi->sdev) == NO_DEVICE)
			insert_headset();
	} 

	gpio92status = cable_in1;
	del_timer(&jrd_headset_timer);
}

static void jrd_headset_timer_expire(unsigned long data)
{
	schedule_delayed_work(&jrd_headset_workqueue, 0);
}

static void detection_work(struct work_struct *work)
{
	H2W_DBG("");

	printk( "======JRD:start jrd_headset_timer 500ms.\n");
	mod_timer(&jrd_headset_timer, jiffies + (HZ/2));

	//local_irq_save(irq_flags);
	enable_irq(hi->irq);
	//local_irq_restore(irq_flags);


	
}



static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	printk("===========detect_irq_handler( irq = %d)===========\r\n",irq);
         
	disable_irq(hi->irq);
	
     
	queue_work(g_detection_work_queue, &g_detection_work);//flin add 
	return IRQ_HANDLED;
}

#if defined(CONFIG_DEBUG_FS)
static void h2w_debug_set(void *data, u64 val)
{
	switch_set_state(&hi->sdev, (int)val);
}

static u64 h2w_debug_get(void *data)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(h2w_debug_fops, h2w_debug_get, h2w_debug_set, "%llu\n");
static int __init h2w_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("h2w", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("state", 0644, dent, NULL, &h2w_debug_fops);

	return 0;
}

device_initcall(h2w_debug_init);
#endif



static int opal_h2w_probe(struct platform_device *pdev)
{
	int ret;
	int cabledetect;

	printk( "////////////H2W: Registering H2W (headset) driver\n");

	
	hi = kzalloc(sizeof(struct h2w_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	
	hi->sdev.name = "h2w";
	hi->sdev.print_name = Opal_h2w_print_name;

	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	g_detection_work_queue = create_workqueue("detection");
	if (g_detection_work_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}
// Liaosicong add timer for headset plug detection 2010-10-15	
	init_timer(&jrd_headset_timer);
	jrd_headset_timer.function = jrd_headset_timer_expire;
	jrd_headset_timer.data = 0;		
	
//#if 0 //---tmp for test the GPIO92 is ownerd by ARM9 0r ARM11
	ret = gpio_request(AUDIO_CABLE_IN, "h2w_detect");
	printk("////////ret = gpio_request  ret=%d\n",ret);
	if (ret < 0)
		goto err_request_detect_gpio;
	
	
//#endif //---tmp for test the GPIO92 is ownerd by ARM9 0r ARM11	
	config_lcdc_gpio_table(msm_earphone_detect_gpio_table, 1, 1);//flin add

	
	hi->irq = gpio_to_irq(AUDIO_CABLE_IN);
	printk("//////////gpio_to_irq() ,h2w IRQ numner=%d\n",hi->irq );
	if (hi->irq < 0) {
		ret = hi->irq;
		goto err_get_h2w_detect_irq_num_failed;
	}
	
	cabledetect = gpio_get_value(AUDIO_CABLE_IN);//add by flin
	printk("================get h2w cable init state=%d=============\n",cabledetect);//add by flin

	if (cabledetect == 0){
		insert_headset();
	//	remove_headset();
	}
	else{
		remove_headset();
	//	insert_headset();
	}
	
	
	ret = request_irq(hi->irq, detect_irq_handler,\
	IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING /*IRQF_TRIGGER_LOW//add by flin*/, "h2w_detect", NULL);
	printk("================request_irq()========= ,ret=%d\n",ret );
	if (ret < 0)
		goto err_request_detect_irq;
	
	ret = set_irq_wake(hi->irq, 1);
	if (ret < 0)
		goto err_request_input_dev;
	
	hi->input = input_allocate_device();
	printk("===================h2w device register==================\n");
	if (!hi->input) {
		ret = -ENOMEM;
		printk("================input_allocate_device failed===============\n");
		goto err_request_input_dev;
	}

	hi->input->name = "h2w headset";
	hi->input->evbit[0] = BIT_MASK(EV_KEY);
	hi->input->keybit[BIT_WORD(KEY_MEDIA)] = BIT_MASK(KEY_MEDIA);

	ret = input_register_device(hi->input);
	printk("==================h2w device register===================\n");
	if (ret < 0)
		goto err_register_input_dev;

	return 0;

err_register_input_dev:
	input_free_device(hi->input);
err_request_input_dev:
	//free_irq(hi->irq_btn, 0);
err_request_detect_irq:
err_get_h2w_detect_irq_num_failed:
err_request_detect_gpio:
	destroy_workqueue(g_detection_work_queue);
err_create_work_queue:
	switch_dev_unregister(&hi->sdev);
err_switch_dev_register:
	printk(KERN_ERR "H2W: Failed to register driver\n");

	return ret;
}

static int opal_h2w_remove(struct platform_device *pdev)
{
	H2W_DBG("");
	if (switch_get_state(&hi->sdev))
		remove_headset();
	input_unregister_device(hi->input);
	gpio_free(AUDIO_CABLE_IN);
	free_irq(hi->irq, 0);
	destroy_workqueue(g_detection_work_queue);
	switch_dev_unregister(&hi->sdev);

	return 0;
}

static struct platform_device opal_h2w_device = {
	.name		= "opal-h2w",
};

static struct platform_driver opal_h2w_driver = {
	.probe		= opal_h2w_probe,
	.remove		=opal_h2w_remove,
	.driver		= {
		.name		= "opal-h2w",
		.owner		= THIS_MODULE,
	},
};

static int __init opal_h2w_init(void)
{

	int ret;
	H2W_DBG("");
	ret = platform_driver_register(&opal_h2w_driver);
	if (ret)
		return ret;
	return platform_device_register(&opal_h2w_device);
}

static void __exit opal_h2w_exit(void)
{
	platform_device_unregister(&opal_h2w_device);
	platform_driver_unregister(&opal_h2w_driver);
}

module_init(opal_h2w_init);
module_exit(opal_h2w_exit);

MODULE_AUTHOR("Fuzhen Lin <fuzhen.lin@tct-sh.com>");
MODULE_DESCRIPTION("OPAL 2 Wire detection driver for OPAL BOARD");
MODULE_LICENSE("GPL");
