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
#define ISO_IN_EP           6

/* Output endpoint number: ISO/IRQ */
#define ISO_OUT_EP          2


/*
 * size of the input-buffer IN BYTES
 */
#define SIZE_IN_BUF	512

/*
 * 16 bytes
 */
#define SIZE_INSN_BUF	512

/*
 * size of the buffer for the dux commands in bytes
 */
#define SIZE_DUX_BUF	256


/* Number of in-URBs which receive the data: min=2 */
#define NUM_IN_BUFFERS_FULL     5

/* Number of out-URBs which send the data: min=2 */
#define NUM_OUT_BUFFERS_FULL    5

/* Number of in-URBs which receive the data: min=5 */
/* must have more buffers due to buggy USB ctr */
#define NUM_IN_BUFFERS_HIGH     10

/* Number of out-URBs which send the data: min=5 */
/* must have more buffers due to buggy USB ctr */
#define NUM_OUT_BUFFERS_HIGH    10

/* Size of one A/D value */
#define SIZE_AD_IN          ((sizeof(int16_t)))


/* Analogue in subdevice */
#define SUBDEV_AD             0

/* Analogue out subdevice */
#define SUBDEV_DA             1

/* Digital I/O */
#define SUBDEV_DIO            2

/* counter */
#define SUBDEV_COUNTER        3

/* timer aka pwm output */
#define SUBDEV_PWM            4

/* number of retries to get the right dux command */
#define RETRIES 10

/*
 * number of subdevices
 */
#define N_SUB_DEVICES	1

/* 300Hz max frequ under PWM */
#define MIN_PWM_PERIOD  ((long)(1E9/300))

/* Default PWM frequency */
#define PWM_DEFAULT_PERIOD ((long)(1E9/100))


/* NI USB6008 STRUCTURE */
struct ni_usb6008_struct {
	int attached;				/* is the device already attached */
	int probed;				/* is the device already probed */

	struct usb_device *usbdev;		/* pointer to the usb-device */
	struct usb_interface *interface;	/* interface structure in 2.6 */
	struct comedi_device *comedidev;	/* comedi device for the interrupt context */
	int ifnum;				/* interface number */

	short int ai_cmd_running;		/* asynchronous command is running */
	short int ai_continous;			/* continous aquisition */
	long int ai_sample_count;		/* number of samples to acquire */

	short int high_speed;			/* is it USB_SPEED_HIGH or not? */

	struct urb *urbIn;			/* BULK-transfer handling: urb */
	int8_t *transfer_buffer;
	int16_t *insnBuffer;			/* input buffer for single insn */
	uint8_t *dux_commands;			/* commands */

	/* semaphore for accessing device */
	struct semaphore sem;
};

static struct ni_usb6008_struct ni_usb6008[NUM_NI_USB6008];

static DECLARE_MUTEX(start_stop_sem);


/******************************************************************************/
/************************* READ / WRITE FUNCTIONS *****************************/
/******************************************************************************/

/* Mode 0 is used to get a single conversion on demand */
static int ni_usb6008_ai_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	int i;
	unsigned int one = 0;
	int chan, range;
	int err = 0;
	struct ni_usb6008_struct *this_usbduxsub = dev->private;

	printk(KERN_INFO "comedi_: ni_usb6008: read analog device\n");

	if (!this_usbduxsub)
		return 0;

	dev_dbg(&this_usbduxsub->interface->dev,
		"comedi%d: ai_insn_read, insn->n=%d, insn->subdev=%d\n",
		dev->minor, insn->n, insn->subdev);

	down(&this_usbduxsub->sem);
	if (!(this_usbduxsub->probed)) {
		up(&this_usbduxsub->sem);
		return -ENODEV;
	}
	if (this_usbduxsub->ai_cmd_running) {
		dev_err(&this_usbduxsub->interface->dev,
			"comedi%d: ai_insn_read not possible. "
			"Async Command is running.\n", dev->minor);
		up(&this_usbduxsub->sem);
		return 0;
	}

	/* sample one channel */
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);
	/* set command for the first channel */
	//this_usbduxsub->dux_commands[1] = create_adc_command(chan, range);

	/* adc commands */
	//err = send_dux_commands(this_usbduxsub, SENDSINGLEAD);
	if (err < 0) {
		up(&this_usbduxsub->sem);
		return err;
	}

	for (i = 0; i < insn->n; i++) {
		//err = receive_dux_commands(this_usbduxsub, SENDSINGLEAD);
		if (err < 0) {
			up(&this_usbduxsub->sem);
			return 0;
		}
		one = le16_to_cpu(this_usbduxsub->insnBuffer[1]);
		if (CR_RANGE(insn->chanspec) <= 1)
			one = one ^ 0x800;

		data[i] = one;
	}
	up(&this_usbduxsub->sem);
	return i;
}

/******************************************************************************/
/********************** DEVICE SPECIFIC FUNCTIONS *****************************/
/******************************************************************************/

/*
 * Stops the data acquision
 * It should be safe to call this function from any context
 */
static int ni_usb6008_unlink_InURBs(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	printk(KERN_INFO "comedi_: ni_usb6008: unlink InURBs\n");

	if (ni_usb6008_tmp && ni_usb6008_tmp->urbIn) {
		ni_usb6008_tmp->ai_cmd_running = 0;
		usb_kill_urb(ni_usb6008_tmp->urbIn);
	}
	printk(KERN_INFO "comedi_: ni_usb6008: "
			"unlinked InURB\n");
	return 0;
}

static void clean_up(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	if (!ni_usb6008_tmp)
		return;

	printk(KERN_INFO "comedi_: ni_usb6008: clean up\n");

	/* shows the usb subsystem that the driver is down */
	if (ni_usb6008_tmp->interface)
		usb_set_intfdata(ni_usb6008_tmp->interface, NULL);

	ni_usb6008_tmp->probed = 0;

	/* cleaning up urbIn */
	if (ni_usb6008_tmp->urbIn) {
		/* waits until a running transfer is over */
		usb_kill_urb(ni_usb6008_tmp->urbIn);

		kfree(ni_usb6008_tmp->transfer_buffer);
		ni_usb6008_tmp->transfer_buffer = NULL;

		usb_free_urb(ni_usb6008_tmp->urbIn);
		ni_usb6008_tmp->urbIn = NULL;
	}

	kfree(ni_usb6008_tmp->insnBuffer);
	ni_usb6008_tmp->insnBuffer = NULL;

	kfree(ni_usb6008_tmp->dux_commands);
	ni_usb6008_tmp->dux_commands = NULL;

	ni_usb6008_tmp->ai_cmd_running = 0;
}



static int ni_usb6008_probe(struct usb_interface *uinterf,
	const struct usb_device_id *id)
{
	int device_index = -1;
	int counter = 0;
	int result = 0;
	struct usb_device *udev = interface_to_usbdev(uinterf);

	printk(KERN_INFO "comedi_: ni_usb6008: probe called\n");

	printk(KERN_INFO "comedi_: ni_usb6008: "
		"locating a free structure for the usb-device\n");

	down(&start_stop_sem);

	/* look for a free place in the ni_usb6008 array */
	device_index = -1;
	for (counter = 0; counter < NUM_NI_USB6008; counter++) {
		if (!(ni_usb6008[counter].probed)) {
			device_index = counter;
			break;
		}
	}

	/* no more space available in the ni_usb6008 array */
	if (device_index == -1) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"too many devices ni_usb6008 connected.\n");
		up(&start_stop_sem);
		return -EMFILE;
	}
	printk(KERN_INFO "comedi_: ni_usb6008: "
		"ni_usb6008[%d] is ready to connect to comedi.\n", device_index);

	init_MUTEX(&(ni_usb6008[device_index].sem));

	/* save a pointer to the usb device */
	ni_usb6008[device_index].usbdev = udev;

	/* save the interface itself */
	ni_usb6008[device_index].interface = uinterf;

	/* get the interface number from the interface */
	ni_usb6008[device_index].ifnum = uinterf->altsetting->desc.bInterfaceNumber;
	printk(KERN_INFO "comedi_: ni_usb6008: "
		"ifnum=%d\n", ni_usb6008[device_index].ifnum);

	/* test if it is high speed (USB 2.0) */
	if (ni_usb6008[device_index].usbdev->speed == USB_SPEED_HIGH) {
		ni_usb6008[device_index].high_speed = USB_SPEED_HIGH;
		printk(KERN_INFO "comedi_: ni_usb6008: "
			"USB high speed device\n");
	}

	/* hand the private data over to the usb subsystem */
	usb_set_intfdata(uinterf, &(ni_usb6008[device_index]));


	
	/* create space for the commands going to the usb device */
	ni_usb6008[device_index].dux_commands = kmalloc(SIZE_DUX_BUF, GFP_KERNEL);
	if (!ni_usb6008[device_index].dux_commands) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"could not allocate space for dac commands\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* create space of the instruction buffer */
	ni_usb6008[device_index].insnBuffer = kmalloc(SIZE_INSN_BUF, GFP_KERNEL);
	if (!ni_usb6008[device_index].insnBuffer) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"could not allocate space for insnBuffer\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}

	
	/* setting to default setting 0. */
	result = usb_set_interface(ni_usb6008[device_index].usbdev,
			      ni_usb6008[device_index].ifnum, 0);
	if (result < 0) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"could not set alternate setting 0\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENODEV;
	}


	ni_usb6008[device_index].urbIn = usb_alloc_urb(0, GFP_KERNEL);
	if (!ni_usb6008[device_index].urbIn) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"could not allocate In urb\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[device_index].transfer_buffer = kmalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!ni_usb6008[device_index].transfer_buffer) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"could not allocate transfer buffer\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}


	ni_usb6008[device_index].ai_cmd_running = 0;

	/* we have reached the bottom of the function */
	ni_usb6008[device_index].probed = 1;
	up(&start_stop_sem);

	result = comedi_usb_auto_config(udev, BOARDNAME);
	printk(KERN_INFO "result of comedi_usb_auto_config is %d", result);

	printk(KERN_INFO "comedi_: ni_usb6008: " 
		 "%d has been successfully initialised\n", device_index);
	/* success */
	return 0;
}

static void ni_usb6008_disconnect(struct usb_interface *uinterf)
{
	struct ni_usb6008_struct *ni_usb6008_tmp = usb_get_intfdata(uinterf);
	struct usb_device *udev = interface_to_usbdev(uinterf);

	printk(KERN_INFO "comedi_: ni_usb6008: disconnect called\n");

	if (!ni_usb6008_tmp) {
		printk(KERN_ERR "comedi_: ni_usb6008: " 
			"disconnect called with null pointer.\n");
		return;
	}
	if (ni_usb6008_tmp->usbdev != udev) {
		printk(KERN_ERR "comedi_: ni_usb6008: " 
			"BUG! called with wrong ponter!\n");
		return;
	}

	comedi_usb_auto_unconfig(udev);

	down(&start_stop_sem);
	down(&ni_usb6008_tmp->sem);
	clean_up(ni_usb6008_tmp);
	up(&ni_usb6008_tmp->sem);
	up(&start_stop_sem);
	printk(KERN_INFO "comedi_: ni_usb6008: disconnected from the usb\n");
}

static int ni_usb6008_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
 	int ret;
	int device_index;
	int counter;
	struct comedi_subdevice *s = NULL;
	dev->private = NULL;

	printk(KERN_INFO "comedi_: ni_usb6008: attach called\n");

	down(&start_stop_sem);
	device_index = -1;
	for (counter = 0; counter < NUM_NI_USB6008; counter++) {
		if (ni_usb6008[counter].probed && !ni_usb6008[counter].attached) {
			device_index = counter;
			break;
		}
	}

	if (device_index < 0) {
		printk(KERN_ERR "comedi%d: ni_usb6008: error: attach failed, "
		       "no usbduxfast devs connected to the usb bus.\n",
		       dev->minor);
		up(&start_stop_sem);
		return -ENODEV;
	}

	down(&(ni_usb6008[device_index].sem));
	/* pointer back to the corresponding comedi device */
	ni_usb6008[device_index].comedidev = dev;

	dev->board_name = BOARDNAME;

	/* set number of subdevices */
	dev->n_subdevices = N_SUB_DEVICES;

	/* allocate space for the subdevices */
	ret = alloc_subdevices(dev, N_SUB_DEVICES);
	if (ret < 0) {
		printk(KERN_ERR "comedi%d: ni_usb6008: error alloc space for "
		       "subdev\n", dev->minor);
		up(&(ni_usb6008[device_index].sem));
		up(&start_stop_sem);
		return ret;
	}

	printk(KERN_INFO "comedi%d: ni_usb6008: usb-device %d is attached to "
	       "comedi.\n", dev->minor, device_index);

	/* private structure is also simply the usb-structure */
	dev->private = ni_usb6008 + device_index;
	/* the first subdevice is the A/D converter */
	s = dev->subdevices + SUBDEV_AD;
	/*
	 * the URBs get the comedi subdevice which is responsible for reading
	 * this is the subdevice which reads data
	 */
	dev->read_subdev = s;
	/* the subdevice receives as private structure the usb-structure */
	s->private = NULL;
	/* analog input */
	s->type = COMEDI_SUBD_AI;
	/* readable and ref is to ground */
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ;
	/* 16 channels */
	s->n_chan = 16;
	/* length of the channellist */
	s->len_chanlist = 16;
	/* callback functions */
	//s->insn_read = usbduxfast_ai_insn_read;
	//s->do_cmdtest = usbduxfast_ai_cmdtest;
	//s->do_cmd = usbduxfast_ai_cmd;
	//s->cancel = usbduxfast_ai_cancel;
	/* max value from the A/D converter (12bit+1 bit for overflow) */
	s->maxdata = 0x1000;
	/* range table to convert to physical units */
	//s->range_table = &range_usbduxfast_ai_range;

	/* finally decide that it's attached */
	ni_usb6008[device_index].attached = 1;

	up(&(ni_usb6008[device_index].sem));
	up(&start_stop_sem);
	printk(KERN_INFO "comedi%d: successfully attached to usbduxfast.\n",
	       dev->minor);

	return 0;
}

static int ni_usb6008_detach(struct comedi_device *dev)
{
	struct ni_usb6008_struct *ni_usb6008_tmp;

	printk(KERN_INFO "comedi_: ni_usb6008: detach called\n");

	if (!dev) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"detach without dev variable\n");
		return -EFAULT;
	}

	ni_usb6008_tmp = dev->private;

	if (!ni_usb6008_tmp) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"detach without ptr to ni_usb6008\n");
		return -EFAULT;
	}

	printk(KERN_INFO "comedi_: ni_usb6008: detached usb device %d\n", dev->minor);

	down(&ni_usb6008_tmp->sem);
	/* Don't allow detach to free the private structure */
	/* It's one entry of of ni_usb6008[] */
	dev->private = NULL;
	ni_usb6008_tmp->attached = 0;
	ni_usb6008_tmp->comedidev = NULL;
	printk(KERN_INFO "comedi_: ni_usb6008: "
		"successfully removed device %d\n", dev->minor);
	up(&ni_usb6008_tmp->sem);
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
