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

#undef DPRINTK
//#ifdef PCI171X_EXTDEBUG
#define DPRINTK(fmt, args...) printk(fmt, ## args)
//#else
//#define DPRINTK(fmt, args...)
//#endif

#define BOARDNAME		"ni_usb6008"

#define SIZE_IN_BUF		64		
#define SIZE_OUT_BUF		64

#define INPUT_ENDPOINT_NUM	0x81

#define BULK_TIMEOUT		1000
#define RETRIES			10

static DECLARE_MUTEX(start_stop_sem);

/********************* comedi constants *****************************/
static const struct comedi_lrange range_ni_usb6008_ai_range = {2, {
								BIP_RANGE(10.0),
								BIP_RANGE(1.0),
								}
};

static const struct comedi_lrange range_ni_usb6008_ao_range = {1, {
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
	int attached;
	int probed;

	struct usb_device *usbdev;
	struct comedi_device *comedidev;
	struct usb_interface *interface;
	int ifnum;

	struct urb *urbIn;

	int16_t *inBuffer;

	short int ai_cmd_running;

	short int ai_continous;
	int ai_sample_count;
	unsigned int ai_timer;
	unsigned int ai_counter;
	unsigned int ai_interval;

	struct semaphore sem;
};

static struct ni_usb6008_struct ni_usb6008;

/******************************************************************************/
/************************ STOP OR CANCEL FUNCTIONS ****************************/
/******************************************************************************/

static int ni_usb6008_unlink_InURBs(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	int ret = 0;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (ni_usb6008_tmp && ni_usb6008_tmp->urbIn) {
		usb_kill_urb(ni_usb6008_tmp->urbIn);
	}
	return ret;
}

static int ni_usb6008_ai_stop(struct ni_usb6008_struct *ni_usb6008_tmp, int do_unlink)
{
	int ret = 0;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp) {
		printk(KERN_ERR "comedi_: ni_usb6008: ai_stop: ni_usb6008_tmp=NULL!\n");
		return -EFAULT;
	}

	if (do_unlink)
		ret = ni_usb6008_unlink_InURBs(ni_usb6008_tmp);

	ni_usb6008_tmp->ai_cmd_running = 0;

	return ret;
}

static int ni_usb6008_ai_cancel(struct comedi_device *dev,
			    struct comedi_subdevice *s)
{
	struct ni_usb6008_struct *ni_usb6008_tmp;
	int res = 0;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	/* force unlink of all urbs */
	ni_usb6008_tmp = dev->private;
	if (!ni_usb6008_tmp)
		return -EFAULT;

	/* prevent other CPUs from submitting new commands just now */
	down(&ni_usb6008_tmp->sem);
	if (!(ni_usb6008_tmp->probed)) {
		up(&ni_usb6008_tmp->sem);
		return -ENODEV;
	}

	/* unlink only if the urb really has been submitted */
	res = ni_usb6008_ai_stop(ni_usb6008_tmp, ni_usb6008_tmp->ai_cmd_running);
	up(&ni_usb6008_tmp->sem);
	return res;
}

static void tidy_up(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp)
		return;

	/* shows the usb subsystem that the driver is down */
	if (ni_usb6008_tmp->interface)
		usb_set_intfdata(ni_usb6008_tmp->interface, NULL);

	ni_usb6008_tmp->probed = 0;

	if (ni_usb6008_tmp->urbIn) {
		if (ni_usb6008_tmp->ai_cmd_running) {
			ni_usb6008_tmp->ai_cmd_running = 0;
			ni_usb6008_unlink_InURBs(ni_usb6008_tmp);
		}
		if (ni_usb6008_tmp->urbIn->transfer_buffer) {
			kfree(ni_usb6008_tmp->urbIn->transfer_buffer);
			ni_usb6008_tmp->urbIn->transfer_buffer = NULL;
		}
		usb_kill_urb(ni_usb6008_tmp->urbIn);
		usb_free_urb(ni_usb6008_tmp->urbIn);
		ni_usb6008_tmp->urbIn = NULL;
		kfree(ni_usb6008_tmp->urbIn);
		ni_usb6008_tmp->urbIn = NULL;
	}

	kfree(ni_usb6008_tmp->inBuffer);
	ni_usb6008_tmp->inBuffer = NULL;
	ni_usb6008_tmp->ai_cmd_running = 0;
}

/************************ URB SUBMISSION FUNCTIONS ****************************/

static int ni_usb6008_submit_InURBs(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	int ret;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp)
		return -EFAULT;

	/* Submit all URBs and start the transfer on the bus */
	ni_usb6008_tmp->urbIn->interval = ni_usb6008_tmp->ai_interval;
	ni_usb6008_tmp->urbIn->context = ni_usb6008_tmp->comedidev;
	ni_usb6008_tmp->urbIn->dev = ni_usb6008_tmp->usbdev;
	ni_usb6008_tmp->urbIn->status = 0;
	ni_usb6008_tmp->urbIn->transfer_flags = URB_ISO_ASAP;

	ret = usb_submit_urb(ni_usb6008_tmp->urbIn, GFP_ATOMIC);
	if (ret) {
		printk(KERN_ERR "comedi_: ni_usb6008: usb_submit_urb error %d\n", ret);
		return ret;
	}
	return 0;
}

/************************ MISCELLANEOUS FUNCTIONS *****************************/


/************************ DATA TRANSFER FUNCTIONS *****************************/

static int ni_usb6008_ai_cmdtest(struct comedi_device *dev,
			     struct comedi_subdevice *s, struct comedi_cmd *cmd)
{
	int err = 0, tmp;
	unsigned int tmpTimer;
	struct ni_usb6008_struct *ni_usb6008_tmp = dev->private;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!(ni_usb6008_tmp->probed))
		return -ENODEV;

	/* make sure triggers are valid */
	/* Only immediate triggers are allowed */
	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_INT;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	/* trigger should happen timed */
	tmp = cmd->scan_begin_src;
	/* start a new _scan_ with a timer */
	cmd->scan_begin_src &= TRIG_TIMER;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	/* scanning is continous */
	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	/* issue a trigger when scan is finished and start a new scan */
	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	/* trigger at the end of count events or not, stop condition or not */
	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/*
	 * step 2: make sure trigger sources are unique and mutually compatible
	 * note that mutual compatibility is not an issue here
	 */
	if (cmd->scan_begin_src != TRIG_FOLLOW &&
	    cmd->scan_begin_src != TRIG_EXT &&
	    cmd->scan_begin_src != TRIG_TIMER)
		err++;
	if (cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		err++;

	if (err)
		return 2;

	/* step 3: make sure arguments are trivially compatible */
	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}

	if (cmd->scan_begin_src == TRIG_FOLLOW) {
		/* internal trigger */
		if (cmd->scan_begin_arg != 0) {
			cmd->scan_begin_arg = 0;
			err++;
		}
	}

	if (cmd->scan_begin_src == TRIG_TIMER) {
		/* full speed */
		/* 1kHz scans every USB frame */
		if (cmd->scan_begin_arg < 1000000) {
			cmd->scan_begin_arg = 1000000;
			err++;
		}
		/*
		 * calc the real sampling rate with the rounding errors
		 */
		tmpTimer = ((unsigned int)(cmd->scan_begin_arg /
					   1000000)) * 1000000;
		if (cmd->scan_begin_arg != tmpTimer) {
			cmd->scan_begin_arg = tmpTimer;
			err++;
		}
	}
	/* the same argument */
	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	if (cmd->stop_src == TRIG_COUNT) {
		/* any count is allowed */
	} else {
		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (err)
		return 3;

	return 0;
}

static int ni_usb6008_ai_cmd(struct comedi_device *dev, struct comedi_subdevice *s)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	int ret;
	struct ni_usb6008_struct *ni_usb6008_tmp = dev->private;
	int minor = ni_usb6008_tmp->comedidev->minor;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp)
		return -EFAULT;

	/* block other CPUs from starting an ai_cmd */
	down(&ni_usb6008_tmp->sem);

	if (!(ni_usb6008_tmp->probed)) {
		up(&ni_usb6008_tmp->sem);
		return -ENODEV;
	}
	if (ni_usb6008_tmp->ai_cmd_running) {
		printk(KERN_ERR "comedi%d: ni_usb6008: ai_cmd not possible. another ai_cmd is running.\n", minor);
		up(&ni_usb6008_tmp->sem);
		return -EBUSY;
	}
	/* set current channel of the running aquisition to zero */
	s->async->cur_chan = 0;

	DPRINTK(KERN_INFO "comedi%d: ni_usb6008: sending commands to the usb device\n", minor);

	/* interval always 1ms */
	ni_usb6008_tmp->ai_interval = 1;
	ni_usb6008_tmp->ai_timer = cmd->scan_begin_arg / 1000000;

	if (ni_usb6008_tmp->ai_timer < 1) {
		printk(KERN_ERR "comedi%d: ni_usb6008: timer=%d, scan_begin_arg=%d. not properly tested by cmdtest?\n",
			minor, ni_usb6008_tmp->ai_timer, cmd->scan_begin_arg);
		up(&ni_usb6008_tmp->sem);
		return -EINVAL;
	}
	ni_usb6008_tmp->ai_counter = ni_usb6008_tmp->ai_timer;

	if (cmd->stop_src == TRIG_COUNT) {
		/* data arrives as one packet */
		ni_usb6008_tmp->ai_sample_count = cmd->stop_arg;
		ni_usb6008_tmp->ai_continous = 0;
	} else {
		/* continous aquisition */
		ni_usb6008_tmp->ai_continous = 1;
		ni_usb6008_tmp->ai_sample_count = 0;
	}

	if (cmd->start_src == TRIG_NOW) {
		/* enable this acquisition operation */
		ni_usb6008_tmp->ai_cmd_running = 1;
		ret = ni_usb6008_submit_InURBs(ni_usb6008_tmp);
		if (ret < 0) {
			ni_usb6008_tmp->ai_cmd_running = 0;
			/* fixme: unlink here?? */
			up(&ni_usb6008_tmp->sem);
			return ret;
		}
		s->async->inttrig = NULL;
	} else {
		printk(KERN_ERR "cannot enter this region!\n");
	}
	up(&ni_usb6008_tmp->sem);
	return 0;
}

/******************************************************************************/
/************************** DATA TARANSFER FUNCTIONS **************************/
/******************************************************************************/

static void ni_usb6008_ai_completion(struct urb *urb)
{
	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);
	printk(KERN_INFO "comedi: ni_usb6008: URB actual_length = %d\n", urb->actual_length);
	printk(KERN_INFO "comedi: ni_usb6008: URB error_count = %d\n", urb->error_count);
}

/******************************************************************************/
/************************ INITIALIZATION FUNCTIONS ****************************/
/******************************************************************************/

static int ni_usb6008_attach(struct comedi_device *comdev, struct comedi_devconfig *it)
{
	struct comedi_subdevice *s = NULL;
	int ret;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	comdev->private = NULL;

	down(&start_stop_sem);

	down(&ni_usb6008.sem);
	ni_usb6008.comedidev = comdev;

	comdev->board_name = BOARDNAME;
	comdev->n_subdevices = 1;

	/* allocate space for the 1 subdevices */
	ret = alloc_subdevices(comdev, comdev->n_subdevices);
	if (ret < 0) {
		printk(KERN_INFO "comedi%d: ni_usb6008: error alloc space for subdev\n", comdev->minor);
		up(&start_stop_sem);
		return ret;
	}

	printk(KERN_INFO "comedi: ni_usb6008 %d is attached to comedi.\n", comdev->minor);

	/* private structure is also simply the usb-structure */
	comdev->private = &ni_usb6008;

	/* the first subdevice is the A/D converter */
	s = comdev->subdevices;
	comdev->read_subdev = s;
	s->private = NULL;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ;
	s->n_chan = 8;
	s->len_chanlist = 8;
	s->do_cmdtest = ni_usb6008_ai_cmdtest;
	s->do_cmd = ni_usb6008_ai_cmd;
	s->cancel = ni_usb6008_ai_cancel;
	s->maxdata = 0xfff;
	s->range_table = (&range_ni_usb6008_ai_range);

	comdev->attached = 1;

	up(&ni_usb6008.sem);

	up(&start_stop_sem);

	printk(KERN_INFO "comedi%d: ni_usb6008: attached to ni_usb6008.\n", comdev->minor);

	return 0;
}

static int ni_usb6008_detach(struct comedi_device *dev)
{
	struct ni_usb6008_struct *ni_usb6008_tmp;

	DPRINTK(KERN_INFO "comedi_: ni_usb6008: %s\n", __func__);

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

static void ni_usb6008_firmware_request_complete_handler(const struct firmware *fw,
						     void *context)
{
	struct ni_usb6008_struct *ni_usb6008_tmp = context;
	struct usb_device *usbdev = ni_usb6008_tmp->usbdev;
	int ret;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	ret = comedi_usb_auto_config(usbdev, BOARDNAME);
	printk(KERN_INFO "comedi_: ni_usb6008_: comedi_usb_auto_config: %d\n", ret);
}


static int ni_usb6008_probe(struct usb_interface *interf,
			   const struct usb_device_id *id)
{
	struct usb_device *usbdev = interface_to_usbdev(interf);
	int ret;

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	down(&start_stop_sem);

	printk(KERN_INFO "comedi_: ni_usb6008: "
		"ready to connect to comedi.\n");

	init_MUTEX(&(ni_usb6008.sem));

	ni_usb6008.usbdev = usbdev;
	ni_usb6008.interface = interf;
	ni_usb6008.ifnum = interf->cur_altsetting->desc.bInterfaceNumber;

	usb_set_intfdata(interf, &(ni_usb6008));

	/* create space for the in buffer and set it to zero */
	ni_usb6008.inBuffer = kzalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!(ni_usb6008.inBuffer)) {
		printk(KERN_ERR "comedi: ni_usb6008: could not alloc space for inBuffer\n");
		tidy_up(&(ni_usb6008));
		up(&start_stop_sem);
		return -ENOMEM;
	}

	ni_usb6008.urbIn = usb_alloc_urb(1, GFP_KERNEL);
	if (ni_usb6008.urbIn == NULL) {
		printk(KERN_ERR "comedi: ni_usb6008: could not alloc. urbIn\n");
		tidy_up(&(ni_usb6008));
		up(&start_stop_sem);
		return -ENOMEM;
	}

	ni_usb6008.urbIn->dev = ni_usb6008.usbdev;
	ni_usb6008.urbIn->context = NULL;
	ni_usb6008.urbIn->pipe = usb_rcvbulkpipe(ni_usb6008.usbdev, INPUT_ENDPOINT_NUM);
	ni_usb6008.urbIn->transfer_buffer = kzalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!(ni_usb6008.urbIn->transfer_buffer)) {
		printk(KERN_ERR "comedi: ni_usb6008: could not alloc. transb.\n");
		tidy_up(&(ni_usb6008));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008.urbIn->complete = ni_usb6008_ai_completion;
	ni_usb6008.urbIn->transfer_buffer_length = SIZE_IN_BUF;
	ni_usb6008.urbIn->interval = 1;

	ni_usb6008.ai_cmd_running = 0;

	ni_usb6008.probed = 1;

	up(&start_stop_sem);

	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      "usbdux_firmware1.bin",
				      &usbdev->dev,
				      GFP_KERNEL,
				      (void *)(&ni_usb6008),
				      ni_usb6008_firmware_request_complete_handler);

	if (ret) {
		printk(KERN_ERR "comedi: ni_usb6008: Could not load firmware (err=%d)\n", ret);
		return ret;
	}

	printk(KERN_INFO "comedi: ni_usb6008: has been successfully initialised.\n");

	return 0;
}

static void ni_usb6008_disconnect(struct usb_interface *interf)
{
	struct ni_usb6008_struct *ni_usb6008_tmp = usb_get_intfdata(interf);
	struct usb_device *usbdev = interface_to_usbdev(interf);

	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp) {
		printk(KERN_INFO "comedi_: ni_usb600: disconnect called with null pointer.\n");
		return;
	}
	if (ni_usb6008_tmp->usbdev != usbdev) {
		printk(KERN_INFO "comedi_: ni_usb600: BUG! called with wrong pointer !\n");
		return;
	}
	comedi_usb_auto_unconfig(usbdev);
	down(&start_stop_sem);
	down(&ni_usb6008_tmp->sem);
	tidy_up(ni_usb6008_tmp);
	up(&ni_usb6008_tmp->sem);
	up(&start_stop_sem);
	printk(KERN_INFO "comedi_: ni_usb6008: disconnected from the usb\n");
}

/* main driver struct */
static struct comedi_driver comedi_ni_usb6008_driver = {
	.driver_name = "ni_usb6008",
	.module = THIS_MODULE,
	.attach = ni_usb6008_attach,
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
	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);
	printk(KERN_INFO KBUILD_MODNAME ": "
	       DRIVER_VERSION ":" DRIVER_DESC "\n");
	usb_register(&usb_ni_usb6008_driver);
	comedi_driver_register(&comedi_ni_usb6008_driver);
	return 0;
}

/* deregistering the comedi driver and the usb-subsystem */
static void __exit exit_ni_usb6008(void)
{
	DPRINTK(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);
	comedi_driver_unregister(&comedi_ni_usb6008_driver);
	usb_deregister(&usb_ni_usb6008_driver);
}

module_init(init_ni_usb6008);
module_exit(exit_ni_usb6008);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
