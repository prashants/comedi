#define DRIVER_VERSION "1.0"
#define DRIVER_AUTHOR "Prashant Shah, pshah.mumbai@gmail.com"
#define DRIVER_DESC "National Instruments NI USB-6008"

/**
 * comedi/drivers/ni_usb6008.c
 * Copyright (C) 2011 Prashant Shah, pshah.mumbai@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/fcntl.h>
#include <linux/compiler.h>
#include <linux/firmware.h>

#include "../comedidev.h"

#define BOARDNAME "ni_usb6008"

/* Total number of ni_usb6008 devices */
#define NUM_NI_USB6008             16

/* Input endpoint number: ISO/IRQ */
#define ISOINEP		6

/* Output endpoint number: ISO/IRQ */
#define ISOOUTEP	2

/*
 * size of the input buffer IN BYTES
 */
#define SIZE_IN_BUF	512

/*
 * size of the single input buffer IN BYTES
 */
#define SIZE_INSN_BUF	512

/*
 * size of the output buffer IN BYTES
 */
#define SIZE_OUT_BUF	512

/*
 * size of the dux buffer IN BYTES
 */
#define SIZE_DUX_BUF	256

/*
 * number of DA command channels
 */
#define NUM_DAC_COMMANDS    8

/*
 * number of subdevices
 */
#define N_SUB_DEVICES	1


/* NI USB6008 STRUCTURE */
struct ni_usb6008_struct {
	int attached;				/* is the device already attached */
	int probed;				/* is the device already probed */

	struct usb_device *usbdev;		/* pointer to the usb-device */
	struct usb_interface *interface;	/* interface structure in 2.6 */
	struct comedi_device *comedidev;	/* comedi device for the interrupt context */
	int ifnum;				/* interface number */

	short int high_speed;			/* is it USB_SPEED_HIGH or not? */

	int16_t *inBuffer;			/* input buffer for the ISO-transfer */
	int16_t *insnBuffer;			/* input buffer for single insn */
	int16_t *outBuffer;			/* output buffer for single DA outputs */
	int8_t *transfer_buffer;

	struct urb *urbIn;			/* BULK-transfer handling: urb */
	struct urb *urbOut;			/* BULK-transfer handling: urb */

	int8_t *dac_commands; /* D/A commands */
	int8_t *dux_commands; /* DUX commands */

	short int ai_cmd_running;
	short int ao_cmd_running;

	/* semaphore for accessing device */
	struct semaphore sem;
};

static struct ni_usb6008_struct ni_usb6008[NUM_NI_USB6008];

static DECLARE_MUTEX(start_stop_sem);


/******************************************************************************/
/************************* READ / WRITE FUNCTIONS *****************************/
/******************************************************************************/

static void ni_usb6008_ai_IsocIrq(struct urb *urb)
{
	printk(KERN_INFO "comedi_: ni_usb6008: ni_usb6008_ai_IsocIrq called\n");
}

static void ni_usb6008_ao_IsocIrq(struct urb *urb)
{
	printk(KERN_INFO "comedi_: ni_usb6008: ni_usb6008_ao_IsocIrq called\n");
}


/******************************************************************************/
/********************** DEVICE SPECIFIC FUNCTIONS *****************************/
/******************************************************************************/

static void clean_up(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	if (!ni_usb6008_tmp)
		return;

	printk(KERN_INFO "comedi_: ni_usb6008: clean_up called\n");

	dev_dbg(&ni_usb6008_tmp->interface->dev, "comedi_: clean up\n");

	/* shows the usb subsystem that the driver is down */
	if (ni_usb6008_tmp->interface)
		usb_set_intfdata(ni_usb6008_tmp->interface, NULL);

	ni_usb6008_tmp->probed = 0;

	if (ni_usb6008_tmp->urbIn) {
		ni_usb6008_tmp->ai_cmd_running = 0;
		usb_kill_urb(ni_usb6008_tmp->urbIn);
		kfree(ni_usb6008_tmp->urbIn->transfer_buffer);
		ni_usb6008_tmp->urbIn->transfer_buffer = NULL;
		usb_free_urb(ni_usb6008_tmp->urbIn);
		ni_usb6008_tmp->urbIn = NULL;
	}
	if (ni_usb6008_tmp->urbOut) {
		ni_usb6008_tmp->ao_cmd_running = 0;
		usb_kill_urb(ni_usb6008_tmp->urbOut);
		kfree(ni_usb6008_tmp->urbOut->transfer_buffer);
		ni_usb6008_tmp->urbOut->transfer_buffer = NULL;
		usb_free_urb(ni_usb6008_tmp->urbOut);
		ni_usb6008_tmp->urbOut = NULL;
	}
	kfree(ni_usb6008_tmp->inBuffer);
	ni_usb6008_tmp->inBuffer = NULL;
	kfree(ni_usb6008_tmp->insnBuffer);
	ni_usb6008_tmp->insnBuffer = NULL;
	kfree(ni_usb6008_tmp->inBuffer);
	ni_usb6008_tmp->inBuffer = NULL;
	kfree(ni_usb6008_tmp->dac_commands);
	ni_usb6008_tmp->dac_commands = NULL;
	kfree(ni_usb6008_tmp->dux_commands);
	ni_usb6008_tmp->dux_commands = NULL;
	ni_usb6008_tmp->ai_cmd_running = 0;
	ni_usb6008_tmp->ao_cmd_running = 0;
}

/* allocate memory for the urbs and initialise them */
static int ni_usb6008_probe(struct usb_interface *uinterf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(uinterf);
	struct device *dev = &uinterf->dev;
	int i;
	int index;

	printk(KERN_INFO "comedi_: ni_usb6008: ni_usb6008_probe called\n");

	dev_dbg(dev, "comedi_: usbdux_: "
		"finding a free structure for the usb-device\n");

	down(&start_stop_sem);
	/* look for a free place in the usbdux array */
	index = -1;
	for (i = 0; i < NUM_NI_USB6008; i++) {
		if (!(ni_usb6008[i].probed)) {
			index = i;
			break;
		}
	}

	/* no more space */
	if (index == -1) {
		dev_err(dev, "Too many usbdux-devices connected.\n");
		up(&start_stop_sem);
		return -EMFILE;
	}
	dev_dbg(dev, "comedi_: usbdux: "
		"ni_usb6008[%d] is ready to connect to comedi.\n", index);

	init_MUTEX(&(ni_usb6008[index].sem));
	/* save a pointer to the usb device */
	ni_usb6008[index].usbdev = udev;

	/* 2.6: save the interface itself */
	ni_usb6008[index].interface = uinterf;
	/* get the interface number from the interface */
	ni_usb6008[index].ifnum = uinterf->altsetting->desc.bInterfaceNumber;
	/* hand the private data over to the usb subsystem */
	usb_set_intfdata(uinterf, &(ni_usb6008[index]));

	dev_dbg(dev, "comedi_: ni_usb6008: ifnum=%d\n", ni_usb6008[index].ifnum);

	/* test if it is high speed (USB 2.0) */
	if (ni_usb6008[index].usbdev->speed == USB_SPEED_HIGH) {
		ni_usb6008[index].high_speed = 1;
		printk(KERN_INFO "comedi_: ni_usb6008: connected as high speed device");
	} else {
		ni_usb6008[index].high_speed = 0;
		printk(KERN_INFO "comedi_: ni_usb6008: connected as normal speed device");
	}

	/* create space for the commands of the DA converter */
	ni_usb6008[index].dac_commands = kzalloc(NUM_DAC_COMMANDS, GFP_KERNEL);
	if (!ni_usb6008[index].dac_commands) {
		dev_err(dev, "comedi_: ni_usb6008: "
			"error alloc space for dac commands\n");
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* create space for the commands going to the usb device */
	ni_usb6008[index].dux_commands = kzalloc(SIZE_DUX_BUF, GFP_KERNEL);
	if (!ni_usb6008[index].dux_commands) {
		dev_err(dev, "comedi_: usbdux: "
			"error alloc space for dac commands\n");
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* create space for the in buffer and set it to zero */
	ni_usb6008[index].inBuffer = kzalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].inBuffer)) {
		dev_err(dev, "comedi_: ni_usb6008: "
			"could not alloc space for inBuffer\n");
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* create space of the instruction buffer */
	ni_usb6008[index].insnBuffer = kzalloc(SIZE_INSN_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].insnBuffer)) {
		dev_err(dev, "comedi_: ni_usb6008: "
			"could not alloc space for insnBuffer\n");
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* create space for the outbuffer */
	ni_usb6008[index].outBuffer = kzalloc(SIZE_OUT_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].outBuffer)) {
		dev_err(dev, "comedi_: ni_usb6008: "
			"could not alloc space for outBuffer\n");
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* setting to alternate setting 3: enabling iso ep and bulk ep. */
	i = usb_set_interface(ni_usb6008[index].usbdev,
			      ni_usb6008[index].ifnum, 3);
	if (i < 0) {
		dev_err(dev, "comedi_: ni_usb6008%d: "
			"could not set alternate setting 3 in high speed.\n",
			index);
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENODEV;
	}

	ni_usb6008[index].urbIn = kzalloc(sizeof(struct urb *), GFP_KERNEL);
	if (!(ni_usb6008[index].urbIn)) {
		dev_err(dev, "comedi_: ni_usb6008: Could not alloc. urbIn array\n");
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbIn = usb_alloc_urb(1, GFP_KERNEL);
	if (ni_usb6008[index].urbIn == NULL) {
		dev_err(dev, "comedi_: ni_usb6008%d: "
			"Could not alloc. urb\n", index);
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbIn->dev = ni_usb6008[index].usbdev;
	ni_usb6008[index].urbIn->context = NULL;
	ni_usb6008[index].urbIn->pipe = usb_rcvisocpipe(ni_usb6008[index].usbdev, ISOINEP);
	ni_usb6008[index].urbIn->transfer_flags = URB_ISO_ASAP;
	ni_usb6008[index].urbIn->transfer_buffer = kzalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].urbIn->transfer_buffer)) {
		dev_err(dev, "comedi_: ni_usb6008%d: "
			"could not alloc. transb.\n", index);
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbIn->complete = ni_usb6008_ai_IsocIrq;
	ni_usb6008[index].urbIn->number_of_packets = 1;
	ni_usb6008[index].urbIn->transfer_buffer_length = SIZE_IN_BUF;
	ni_usb6008[index].urbIn->iso_frame_desc[0].offset = 0;
	ni_usb6008[index].urbIn->iso_frame_desc[0].length = SIZE_IN_BUF;

	/* out */
	ni_usb6008[index].urbOut = kzalloc(sizeof(struct urb *), GFP_KERNEL);
	if (!(ni_usb6008[index].urbOut)) {
		dev_err(dev, "comedi_: ni_usb6008: "
			"Could not alloc. urbOut array\n");
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbOut = usb_alloc_urb(1, GFP_KERNEL);
	if (ni_usb6008[index].urbOut == NULL) {
		dev_err(dev, "comedi_: ni_usb6008%d: "
				"Could not alloc. urb\n", index);
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbOut->dev = ni_usb6008[index].usbdev;
	ni_usb6008[index].urbOut->context = NULL;
	ni_usb6008[index].urbOut->pipe = usb_sndisocpipe(ni_usb6008[index].usbdev, ISOOUTEP);
	ni_usb6008[index].urbOut->transfer_flags = URB_ISO_ASAP;
	ni_usb6008[index].urbOut->transfer_buffer = kzalloc(SIZE_OUT_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].urbOut->transfer_buffer)) {
		dev_err(dev, "comedi_: ni_usb6008%d: "
			"could not alloc. transb.\n", index);
		clean_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbOut->complete = ni_usb6008_ao_IsocIrq;
	ni_usb6008[index].urbOut->number_of_packets = 1;
	ni_usb6008[index].urbOut->transfer_buffer_length = SIZE_OUT_BUF;
	ni_usb6008[index].urbOut->iso_frame_desc[0].offset = 0;
	ni_usb6008[index].urbOut->iso_frame_desc[0].length = SIZE_OUT_BUF;
	if (ni_usb6008[index].high_speed) {
		/* uframes */
		ni_usb6008[index].urbOut->interval = 8;
	} else {
		/* frames */
		ni_usb6008[index].urbOut->interval = 1;
	}

	ni_usb6008[index].ai_cmd_running = 0;
	ni_usb6008[index].ao_cmd_running = 0;

	/* we've reached the bottom of the function */
	ni_usb6008[index].probed = 1;
	up(&start_stop_sem);

	comedi_usb_auto_config(udev, BOARDNAME);

	dev_info(dev, "comedi_: ni_usb6008%d"
		 "has been successfully initialised.\n", index);
	/* success */
	return 0;
}

static void ni_usb6008_disconnect(struct usb_interface *intf)
{
	struct ni_usb6008_struct *ni_usb6008_tmp = usb_get_intfdata(intf);
	struct usb_device *udev = interface_to_usbdev(intf);

	printk(KERN_INFO "comedi_: ni_usb6008: ni_usb6008_disconnect called\n");

	if (!ni_usb6008_tmp) {
		dev_err(&intf->dev,
			"comedi_: disconnect called with null pointer.\n");
		return;
	}
	if (ni_usb6008_tmp->usbdev != udev) {
		dev_err(&intf->dev, "comedi_: BUG! called with wrong ptr!!!\n");
		return;
	}
	comedi_usb_auto_unconfig(udev);
	down(&start_stop_sem);
	down(&ni_usb6008_tmp->sem);
	clean_up(ni_usb6008_tmp);
	up(&ni_usb6008_tmp->sem);
	up(&start_stop_sem);
	dev_dbg(&intf->dev, "comedi_: disconnected from the usb\n");
}

/* is called when comedi-config is called */
static int ni_usb6008_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	printk(KERN_INFO "comedi_: ni_usb6008: ni_usb6008_attach called\n");
	return 0;
}

static int ni_usb6008_detach(struct comedi_device *dev)
{
	printk(KERN_INFO "comedi_: ni_usb6008: ni_usb6008_detach called\n");
	return 0;
}

/* Table with the USB-devices */
static const struct usb_device_id ni_usb6008_table[] = {
	{USB_DEVICE(0x3923, 0x717A)},
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, ni_usb6008_table);

/* main driver struct */
static struct comedi_driver comedidriver_ni_usb6008 = {
	.driver_name = "ni_usb6008",
	.module = THIS_MODULE,
	.attach = ni_usb6008_attach,
	.detach = ni_usb6008_detach,
};

/* The usb driver for ni_usb6008 */
static struct usb_driver usbdriver_ni_usb6008 = {
	.name = BOARDNAME,
	.probe = ni_usb6008_probe,
	.disconnect = ni_usb6008_disconnect,
	.id_table = ni_usb6008_table,
};

/* registering the usb-system _and_ the comedi-driver */
static int __init init_ni_usb6008(void)
{
	int result;

	printk(KERN_INFO "comedi_: ni_usb6008: init called\n");

	result = usb_register(&usbdriver_ni_usb6008);
	if (result < 0) {
	  printk(KERN_ERR "comedi_: ni_usb6008: usb_register failed with error number %d", result);
	  return -1;
	}

	comedi_driver_register(&comedidriver_ni_usb6008);
	return 0;
}

/* deregistering the comedi driver and the usb-subsystem */
static void __exit exit_ni_usb6008(void)
{
	printk(KERN_INFO "comedi_: ni_usb6008: exit called\n");
	comedi_driver_unregister(&comedidriver_ni_usb6008);
	usb_deregister(&usbdriver_ni_usb6008);
}

module_init(init_ni_usb6008);
module_exit(exit_ni_usb6008);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
