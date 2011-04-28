#define DRIVER_VERSION "v2.4"
#define DRIVER_AUTHOR "Bernd Porr, BerndPorr@f2s.com"
#define DRIVER_DESC "Stirling/ITL USB-DUX -- Bernd.Porr@f2s.com"

/**
   comedi/drivers/ni_usb600.c
   Copyright (C) 2003-2007 Bernd Porr, Bernd.Porr@f2s.com

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

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

/********************* comedi constants *****************************/
static const struct comedi_lrange range_ni_usb6008_ai_range = { 2, {
								BIP_RANGE(10.0),
								BIP_RANGE(1.0),
								}
};

static const struct comedi_lrange range_ni_usb6008_ao_range = { 2, {
								UNI_RANGE(5.0),
								}
};

/*
 * private structure of one subdevice
 */

/*
 * This is the structure which holds all the data of
 * this driver one sub device just now: A/D
 */
struct ni_usb6008_struct {
	/* attached? */
	int attached;
	/* is it associated with a subdevice? */
	int probed;
	/* pointer to the usb-device */
	struct usb_device *usbdev;
	/* actual number of in-buffers */
	int numOfInBuffers;
	/* actual number of out-buffers */
	int numOfOutBuffers;
	/* ISO-transfer handling: buffers */
	struct urb **urbIn;
	struct urb **urbOut;
	/* pwm-transfer handling */
	struct urb *urbPwm;
	/* PWM period */
	unsigned int pwmPeriod;
	/* PWM internal delay for the GPIF in the FX2 */
	int8_t pwmDelay;
	/* size of the PWM buffer which holds the bit pattern */
	int sizePwmBuf;
	/* input buffer for the ISO-transfer */
	int16_t *inBuffer;
	/* input buffer for single insn */
	int16_t *insnBuffer;
	/* output buffer for single DA outputs */
	int16_t *outBuffer;
	/* interface number */
	int ifnum;
	/* interface structure in 2.6 */
	struct usb_interface *interface;
	/* comedi device for the interrupt context */
	struct comedi_device *comedidev;
	/* is it USB_SPEED_HIGH or not? */
	short int high_speed;
	/* asynchronous command is running */
	short int ai_cmd_running;
	short int ao_cmd_running;
	/* pwm is running */
	short int pwm_cmd_running;
	/* continous aquisition */
	short int ai_continous;
	short int ao_continous;
	/* number of samples to acquire */
	int ai_sample_count;
	int ao_sample_count;
	/* time between samples in units of the timer */
	unsigned int ai_timer;
	unsigned int ao_timer;
	/* counter between aquisitions */
	unsigned int ai_counter;
	unsigned int ao_counter;
	/* interval in frames/uframes */
	unsigned int ai_interval;
	/* D/A commands */
	int8_t *dac_commands;
	/* commands */
	int8_t *dux_commands;
	struct semaphore sem;
};

static int ni_usb600_probe(struct usb_interface *uinterf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(uinterf);
	struct device *dev = &uinterf->dev;
	int i;
	int index;
	int ret;

	printk(KERN_INFO "comedi_: ni_usb6008_: finding a free structure for the usb-device\n");

	down(&start_stop_sem);
	index = -1;
	for (i = 0; i < NUMUSBDUX; i++) {
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
	dev_dbg(dev, "comedi_: ni_usb6008: "
		"ni_usb6008[%d] is ready to connect to comedi.\n", index);

	init_MUTEX(&(ni_usb6008[index].sem));
	/* save a pointer to the usb device */
	ni_usb6008[index].usbdev = udev;

	/* 2.6: save the interface itself */
	ni_usb6008[index].interface = uinterf;

	ni_usb6008[index].ifnum = uinterf->altsetting->desc.bInterfaceNumber;

	usb_set_intfdata(uinterf, &(ni_usb6008[index]));

	dev_dbg(dev, "comedi_: ni_usb6008: ifnum=%d\n", ni_usb6008[index].ifnum);

	/* create space for the commands going to the usb device */
	ni_usb6008[index].to_device_commands = kzalloc(SIZE_TO_DEVICE_CMD_BUF, GFP_KERNEL);
	if (!ni_usb6008[index].to_device_commands) {
		dev_err(dev, "comedi_: ni_usb6008: error alloc space for device commands\n");
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* create space for the in buffer and set it to zero */
	ni_usb6008[index].inBuffer = kzalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].inBuffer)) {
		dev_err(dev, "comedi_: ni_usb6008: could not alloc space for inBuffer\n");
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* create space for the outbuffer */
	ni_usb6008[index].outBuffer = kzalloc(SIZE_OUT_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].outBuffer)) {
		dev_err(dev, "comedi_: ni_usb6008: could not alloc space for outBuffer\n");
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* create space of the instruction buffer */
	ni_usb6008[index].insnBuffer = kzalloc(SIZE_INSN_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].insnBuffer)) {
		dev_err(dev, "comedi_: ni_usb6008: "
			"could not alloc space for insnBuffer\n");
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}

	ni_usb6008[index].urbIn = kzalloc(sizeof(struct urb *), GFP_KERNEL);
	if (!(ni_usb6008[index].urbIn)) {
		dev_err(dev, "comedi_: ni_usb6008: could not allocate urbIn\n");
		tidy_up(&(usbduxsub[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbIn = usb_alloc_urb(1, GFP_KERNEL);
	if (ni_usb6008[index].urbIn == NULL) {
		printk(KERN_INFO "comedi_: ni_usb6008%d: could not alloc. urbOut\n", index);
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbIn->dev = ni_usb6008[index].usbdev;
	ni_usb6008[index].urbIn->context = NULL;
	ni_usb6008[index].urbIn->pipe = usb_rcvisocpipe(ni_usb6008[index].usbdev, ISOINEP);
	ni_usb6008[index].urbIn->transfer_flags = URB_ISO_ASAP;
	ni_usb6008[index].urbIn->transfer_buffer = kzalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].urbIn->transfer_buffer)) {
		printk(KERN_INFO "comedi_: ni_usb6008%d: could not alloc. transb.\n", index);
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbIn->complete = usbduxsub_ai_IsocIrq;
	ni_usb6008[index].urbIn->number_of_packets = 1;
	ni_usb6008[index].urbIn->transfer_buffer_length = SIZE_IN_BUF;
	ni_usb6008[index].urbIn->iso_frame_desc[0].offset = 0;
	ni_usb6008[index].urbIn->iso_frame_desc[0].length = SIZE_IN_BUF;
	ni_usb6008[index].urbIn->interval = 1;

	ni_usb6008[index].urbOut = kzalloc(sizeof(struct urb *), GFP_KERNEL);
	if (!(ni_usb6008[index].urbOut)) {
		printk(KERN_INFO "comedi_: ni_usb6008%d: could not alloc. urbOut\n", index);
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbOut = usb_alloc_urb(1, GFP_KERNEL);
	if (ni_usb6008[index].urbOut == NULL) {
		printk(KERN_INFO "comedi_: ni_usb6008%d: could not alloc. urb\n", index);
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbOut->dev = ni_usb6008[index].usbdev;
	ni_usb6008[index].urbOut->context = NULL;
	ni_usb6008[index].urbOut->pipe = usb_sndisocpipe(usbduxsub[index].usbdev, ISOOUTEP);
	ni_usb6008[index].urbOut->transfer_flags = URB_ISO_ASAP;
	ni_usb6008[index].urbOut->transfer_buffer = kzalloc(SIZE_OUT_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].urbOut->transfer_buffer)) {
		printk(KERN_INFO "comedi_: ni_usb6008%d: could not alloc. transb.\n", index);
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbOut->complete = usbduxsub_ao_IsocIrq;
	ni_usb6008[index].urbOut->number_of_packets = 1;
	ni_usb6008[index].urbOut->transfer_buffer_length = SIZE_OUT_BUF;
	ni_usb6008[index].urbOut->iso_frame_desc[0].offset = 0;
	ni_usb6008[index].urbOut->iso_frame_desc[0].length = SIZE_OUT_BUF;
	ni_usb6008[index].urbOut->interval = 1;

	ni_usb6008[index].urbPwm = NULL;
	ni_usb6008[index].sizePwmBuf = 0;

	ni_usb6008[index].ai_cmd_running = 0;
	ni_usb6008[index].ao_cmd_running = 0;
	ni_usb6008[index].pwm_cmd_running = 0;

	/* we've reached the bottom of the function */
	ni_usb6008[index].probed = 1;
	up(&start_stop_sem);

	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      "usbdux_firmware1.bin",
				      &udev->dev,
				      GFP_KERNEL,
				      ni_usb6008 + index,
				      ni_usb6008_firmware_request_complete_handler);

	if (ret) {
		dev_err(dev, "Could not load firmware (err=%d)\n", ret);
		return ret;
	}

	printk(KERN_INFO "comedi_: ni_usb6008%d: has been successfully initialised.\n", index);
	/* success */
	return 0;
}

static void ni_usb6008_disconnect(struct usb_interface *intf)
{
	struct ni_usb6008_struct *ni_usb6008_tmp = usb_get_intfdata(intf);
	struct usb_device *udev = interface_to_usbdev(intf);

	if (!ni_usb6008_tmp) {
		printk(KERN_INFO "comedi_: ni_usb600: disconnect called with null pointer.\n");
		return;
	}
	if (ni_usb6008->usbdev != udev) {
		printk(KERN_INFO "comedi_: ni_usb600: BUG! called with wrong pointer !\n");
		return;
	}
	comedi_usb_auto_unconfig(udev);
	down(&start_stop_sem);
	down(&ni_usb6008_tmp->sem);
	tidy_up(ni_usb6008_tmp);
	up(&ni_usb6008_tmp->sem);
	up(&start_stop_sem);
	dev_dbg(&intf->dev, "comedi_: ni_usb600: disconnected from the usb\n");
}

/* is called when comedi-config is called */
static int ni_usb6008_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	int ret;
	int index;
	int i;
	struct ni_usb6008_struct *udev;

	struct comedi_subdevice *s = NULL;
	dev->private = NULL;

	down(&start_stop_sem);
	index = -1;
	for (i = 0; i < NUMUSBDUX; i++) {
		if ((ni_usb6008[i].probed) && (!ni_usb6008[i].attached)) {
			index = i;
			break;
		}
	}

	if (index < 0) {
		printk(KERN_ERR "comedi%d: ni_usb6008: error: attach failed, no "
		       "ni_usb6008 devs connected to the usb bus.\n", dev->minor);
		up(&start_stop_sem);
		return -ENODEV;
	}

	udev = &ni_usb6008[index];

	down(&udev->sem);
	udev->comedidev = dev;

	dev->board_name = BOARDNAME;

	/* set number of subdevices */
	dev->n_subdevices = 1;

	/* allocate space for the subdevices */
	ret = alloc_subdevices(dev, dev->n_subdevices);
	if (ret < 0) {
		printk(KERN_INFO "comedi%d: ni_usb6008: error alloc space for subdev\n", dev->minor);
		up(&start_stop_sem);
		return ret;
	}

	printk(KERN_INFO "comedi%d: ni_usb6008 %d is attached to comedi.\n", dev->minor, index);

	/* private structure is also simply the usb-structure */
	dev->private = udev;

	/* the first subdevice is the A/D converter */
	s = dev->subdevices + 0;
	dev->read_subdev = s;
	s->private = NULL;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ;
	s->n_chan = 8;
	s->len_chanlist = 8;
	s->insn_read = ni_usb6008_ai_insn_read;
	s->do_cmdtest = ni_usb6008_ai_cmdtest;
	s->do_cmd = ni_usb6008_ai_cmd;
	s->cancel = ni_usb6008_ai_cancel;
	s->maxdata = 0xfff;
	s->range_table = (&range_ni_usb6008_ai_range);

	udev->attached = 1;

	up(&udev->sem);

	up(&start_stop_sem);

	printk(KERN_INFO "comedi%d: ni_usb6008: attached to ni_usb6008.\n", dev->minor);

	return 0;
}

static int ni_usb6008_detach(struct comedi_device *dev)
{
	struct ni_usb6008_struct *ni_usb6008_tmp;

	printk(KERN_INFO "comedi_: ni_usb6008: %s\n", __func__);

	if (!dev) {
		printk(KERN_ERR "comedi?: ni_usb6008: cannot detach without dev variable.\n");
		return -EFAULT;
	}

	ni_usb6008_tmp = dev->private;
	if (!ni_usb6008_tmp) {
		printk(KERN_ERR
		       "comedi?: ni_usb6008: cannot  detach without pointer to ni_usb6008_tmp\n");
		return -EFAULT;
	}

	printk(KERN_INFO "comedi%d: ni_usb6008: detach usb device\n", dev->minor);

	down(&ni_usb6008_tmp->sem);
	dev->private = NULL;
	ni_usb6008_tmp->attached = 0;
	ni_usb6008_tmp->comedidev = NULL;
	printk(KERN_INFO "comedi%d: ni_usb6008: detach: successfully removed\n", dev->minor);
	up(&ni_usb6008_tmp->sem);
	return 0;
}

/* main driver struct */
static struct comedi_driver comedi_ni_usb6008_driver = {
	.driver_name = "_ni_usb6008",
	.module = THIS_MODULE,
	.attach = ni_usb6008_usb6008_attach,
	.detach = ni_usb6008_detach,
};

/* Table with the USB-devices: just now only testing IDs */
static const struct usb_device_id ni_usb6008_table[] = {
	{USB_DEVICE(0x3923, 0x717A)},
	{}			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, ni_usb6008_table);

/* The usbduxsub-driver */
static struct usb_driver usb_ni_usb6008_driver = {
	.name = BOARDNAME,
	.probe = ni_usb6008_probe,
	.disconnect = ni_usb6008_disconnect,
	.id_table = ni_usb6008_table,
};

/* Can't use the nice macro as I have also to initialise the USB */
/* subsystem: */
/* registering the usb-system _and_ the comedi-driver */
static int __init init_ni_usb6008(void)
{
	printk(KERN_INFO KBUILD_MODNAME ": "
	       DRIVER_VERSION ":" DRIVER_DESC "\n");
	usb_register(&usb_ni_usb6008_driver);
	comedi_driver_register(&comedi_ni_usb6008_driver);
	return 0;
}

/* deregistering the comedi driver and the usb-subsystem */
static void __exit exit_ni_usb6008(void)
{
	comedi_driver_unregister(&comedi_ni_usb6008_driver);
	usb_deregister(&usb_ni_usb6008_driver);
}

module_init(init_ni_usb6008);
module_exit(exit_ni_usb6008);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");