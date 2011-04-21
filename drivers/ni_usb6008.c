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

/*
 * Size of the input-buffer IN BYTES
 * Always multiple of 8 for 8 microframes which is needed in the highspeed mode
 */
#define SIZE_IN_BUF         ((8*SIZE_AD_IN))

/* size of one value for the D/A converter: channel and value */
#define SIZE_DA_OUT          ((sizeof(int8_t)+sizeof(int16_t)))

/*
 * Size of the output-buffer in bytes
 * Actually only the first 4 triplets are used but for the
 * high speed mode we need to pad it to 8 (microframes).
 */
#define SIZE_OUT_BUF         ((8*SIZE_DA_OUT))



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



/* 300Hz max frequ under PWM */
#define MIN_PWM_PERIOD  ((long)(1E9/300))

/* Default PWM frequency */
#define PWM_DEFAULT_PERIOD ((long)(1E9/100))


/* NI USB6008 STRUCTURE */
struct ni_usb6008_struct {
	/* is the device already attached */
	int attached;
	/* is the device already probed */
	int probed;

	/* pointer to the usb-device */
	struct usb_device *usbdev;
	/* interface structure in 2.6 */
	struct usb_interface *interface;
	/* comedi device for the interrupt context */
	struct comedi_device *comedidev;
	/* interface number */
	int ifnum;

	/* asynchronous command is running */
	short int ai_cmd_running;
	short int ao_cmd_running;
	/* pwm is running */
	short int pwm_cmd_running;

	/* input buffer for the ISO-transfer */
	int16_t *inBuffer;
	/* input buffer for single insn */
	int16_t *insnBuffer;
	/* output buffer for single DA outputs */
	int16_t *outBuffer;

	/* is it USB_SPEED_HIGH or not? */
	short int high_speed;
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
	int counter = 0;

	printk(KERN_INFO "comedi_: ni_usb6008: unlink InURBs\n");

	if (ni_usb6008_tmp && ni_usb6008_tmp->urbIn) {
		for (counter = 0; counter < ni_usb6008_tmp->numOfInBuffers; counter++) {
			if (ni_usb6008_tmp->urbIn[counter]) {
				/* We wait here until all transfers have been cancelled. */
				usb_kill_urb(ni_usb6008_tmp->urbIn[counter]);
			}
			printk(KERN_INFO "comedi_: ni_usb6008: "
				"unlinked InURB %d\n", counter);
		}
	}
	return 0;
}

/*
 * Stops the data acquision
 * It should be safe to call this function from any context
 */
static int ni_usb6008_unlink_OutURBs(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	int counter = 0;

	printk(KERN_INFO "comedi_: ni_usb6008: unlink OutURBs\n");

	if (ni_usb6008_tmp && ni_usb6008_tmp->urbOut) {
		for (counter = 0; counter < ni_usb6008_tmp->numOfOutBuffers; counter++) {
			if (ni_usb6008_tmp->urbOut[counter]) {
				/* We wait here until all transfers have been cancelled. */
				usb_kill_urb(ni_usb6008_tmp->urbOut[counter]);
			}

			printk(KERN_INFO "comedi_: ni_usb6008: "
				"unlinked OutURB %d\n", counter);
		}
	}
	return 0;
}


static void clean_up(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	int counter;

	if (!ni_usb6008_tmp)
		return;

	printk(KERN_INFO "comedi_: ni_usb6008: clean up\n");

	/* shows the usb subsystem that the driver is down */
	if (ni_usb6008_tmp->interface)
		usb_set_intfdata(ni_usb6008_tmp->interface, NULL);

	ni_usb6008_tmp->probed = 0;

	/* cleaning up urbIn */
	if (ni_usb6008_tmp->urbIn) {
		if (ni_usb6008_tmp->ai_cmd_running) {
			ni_usb6008_tmp->ai_cmd_running = 0;
			ni_usb6008_unlink_InURBs(ni_usb6008_tmp);
		}
		/* cleaning up urbIn elements */
		for (counter = 0; counter < ni_usb6008_tmp->numOfInBuffers; counter++) {
			if (ni_usb6008_tmp->urbIn[counter]->transfer_buffer) {
				kfree(ni_usb6008_tmp->urbIn[counter]->transfer_buffer);
				ni_usb6008_tmp->urbIn[counter]->transfer_buffer = NULL;
			}
			if (ni_usb6008_tmp->urbIn[counter]) {
				usb_kill_urb(ni_usb6008_tmp->urbIn[counter]);
				usb_free_urb(ni_usb6008_tmp->urbIn[counter]);
				ni_usb6008_tmp->urbIn[counter] = NULL;
			}
		}
		kfree(ni_usb6008_tmp->urbIn);
		ni_usb6008_tmp->urbIn = NULL;
	}

	/* cleaning up urbOut */
	if (ni_usb6008_tmp->urbOut) {
		if (ni_usb6008_tmp->ao_cmd_running) {
			ni_usb6008_tmp->ao_cmd_running = 0;
			ni_usb6008_unlink_OutURBs(ni_usb6008_tmp);
		}
		/* cleaning up urbOut elements */
		for (counter = 0; counter < ni_usb6008_tmp->numOfOutBuffers; counter++) {
			if (ni_usb6008_tmp->urbOut[counter]->transfer_buffer) {
				kfree(ni_usb6008_tmp->urbOut[counter]->transfer_buffer);
				ni_usb6008_tmp->urbOut[counter]->transfer_buffer = NULL;
			}
			if (ni_usb6008_tmp->urbOut[counter]) {
				usb_kill_urb(ni_usb6008_tmp->urbOut[counter]);
				usb_free_urb(ni_usb6008_tmp->urbOut[counter]);
				ni_usb6008_tmp->urbOut[counter] = NULL;
			}
		}
		kfree(ni_usb6008_tmp->urbOut);
		ni_usb6008_tmp->urbOut = NULL;
	}

	kfree(ni_usb6008_tmp->inBuffer);
	ni_usb6008_tmp->inBuffer = NULL;
	//kfree(ni_usb6008_tmp->insnBuffer);
	//ni_usb6008_tmp->insnBuffer = NULL;
	//kfree(ni_usb6008_tmp->dac_commands);
	//ni_usb6008_tmp->dac_commands = NULL;
	//kfree(ni_usb6008_tmp->dux_commands);
	//ni_usb6008_tmp->dux_commands = NULL;
	kfree(ni_usb6008_tmp->outBuffer);
	ni_usb6008_tmp->outBuffer = NULL;

	ni_usb6008_tmp->ai_cmd_running = 0;
	ni_usb6008_tmp->ao_cmd_running = 0;
	ni_usb6008_tmp->pwm_cmd_running = 0;
}



static int ni_usb6008_probe(struct usb_interface *uinterf,
	const struct usb_device_id *id)
{
	int device_index = -1;
	int counter = 0;
	int result = 0;
	struct usb_device *udev = interface_to_usbdev(uinterf);
	//struct device *dev = &uinterf->dev;

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

	/* hand the private data over to the usb subsystem */
	usb_set_intfdata(uinterf, &(ni_usb6008[device_index]));

	printk(KERN_INFO "comedi_: ni_usb6008: "
		"ifnum=%d\n", ni_usb6008[device_index].ifnum);

	/* test if it is high speed (USB 2.0) */
	ni_usb6008[device_index].high_speed =
	    (ni_usb6008[device_index].usbdev->speed == USB_SPEED_HIGH);
	printk(KERN_INFO "comedi_: ni_usb6008: "
		"USB high speed device\n");

	/* create space for the in buffer and set it to zero */
	ni_usb6008[device_index].inBuffer = kzalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!(ni_usb6008[device_index].inBuffer)) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"could not allocate space for inBuffer\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}

	/* create space for the out buffer and set it to zero */
	ni_usb6008[device_index].outBuffer = kzalloc(SIZE_OUT_BUF, GFP_KERNEL);
	if (!(ni_usb6008[device_index].outBuffer)) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"could not allocate space for outBuffer\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}

	/* setting to alternate default setting 0. */
	result = usb_set_interface(ni_usb6008[device_index].usbdev,
			      ni_usb6008[device_index].ifnum, 0);
	if (result < 0) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"could not set alternate setting 0\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENODEV;
	}

	/******************************* in buffers ***************************/
	if (ni_usb6008[device_index].high_speed)
		ni_usb6008[device_index].numOfInBuffers = NUM_IN_BUFFERS_HIGH;
	else
		ni_usb6008[device_index].numOfInBuffers = NUM_IN_BUFFERS_FULL;

	/* allocating entire urbIn array */
	ni_usb6008[device_index].urbIn =
	    kzalloc(sizeof(struct urb *) * ni_usb6008[device_index].numOfInBuffers,
		    GFP_KERNEL);
	if (!(ni_usb6008[device_index].urbIn)) {
		printk(KERN_ERR "comedi_: ni_usb6008: " 
			"could not allocate urbIn array\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* creating urb for each element of urbIn array */
	for (counter = 0; counter < ni_usb6008[device_index].numOfInBuffers; counter++) {
		/* one frame: 1ms */
		ni_usb6008[device_index].urbIn[counter] = usb_alloc_urb(1, GFP_KERNEL);
		if (ni_usb6008[device_index].urbIn[counter] == NULL) {
			printk(KERN_ERR "comedi_: ni_usb6008: " 
				"could not allocate urb (%d)\n", counter);
			clean_up(&(ni_usb6008[device_index]));
			up(&start_stop_sem);
			return -ENOMEM;
		}
		ni_usb6008[device_index].urbIn[counter]->dev = ni_usb6008[device_index].usbdev;
		/* will be filled later with a pointer to the comedi-device */
		/* and ONLY then the urb should be submitted */
		ni_usb6008[device_index].urbIn[counter]->context = NULL;
		ni_usb6008[device_index].urbIn[counter]->pipe =
		    usb_rcvisocpipe(ni_usb6008[device_index].usbdev, ISO_IN_EP);
		ni_usb6008[device_index].urbIn[counter]->transfer_flags = URB_ISO_ASAP;
		ni_usb6008[device_index].urbIn[counter]->transfer_buffer =
		    kzalloc(SIZE_IN_BUF, GFP_KERNEL);
		if (!(ni_usb6008[device_index].urbIn[counter]->transfer_buffer)) {
			printk(KERN_ERR "comedi_: ni_usb6008: " 
				"could not allocate transfer buffer (%d)\n", counter);
			clean_up(&(ni_usb6008[device_index]));
			up(&start_stop_sem);
			return -ENOMEM;
		}
		//ni_usb6008[device_index].urbIn[counter]->complete = usbduxsub_ai_IsocIrq;
		ni_usb6008[device_index].urbIn[counter]->number_of_packets = 1;
		ni_usb6008[device_index].urbIn[counter]->transfer_buffer_length = SIZE_IN_BUF;
		ni_usb6008[device_index].urbIn[counter]->iso_frame_desc[0].offset = 0;
		ni_usb6008[device_index].urbIn[counter]->iso_frame_desc[0].length = SIZE_IN_BUF;
	}

	/****************************** out buffers ***************************/
	if (ni_usb6008[device_index].high_speed)
		ni_usb6008[device_index].numOfOutBuffers = NUM_OUT_BUFFERS_HIGH;
	else
		ni_usb6008[device_index].numOfOutBuffers = NUM_OUT_BUFFERS_FULL;

	/* allocating entire urbOut array */
	ni_usb6008[device_index].urbOut =
	    kzalloc(sizeof(struct urb *) * ni_usb6008[device_index].numOfOutBuffers,
		    GFP_KERNEL);
	if (!(ni_usb6008[device_index].urbOut)) {
		printk(KERN_ERR "comedi_: ni_usb6008: " 
			"could not allocate urbOut array\n");
		clean_up(&(ni_usb6008[device_index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	/* creating urb for each element of urbOut array */
	for (counter = 0; counter < ni_usb6008[device_index].numOfOutBuffers; counter++) {
		/* one frame: 1ms */
		ni_usb6008[device_index].urbOut[counter] = usb_alloc_urb(1, GFP_KERNEL);
		if (ni_usb6008[device_index].urbOut[counter] == NULL) {
			printk(KERN_ERR "comedi_: ni_usb6008: " 
				"could not allocate urb (%d)\n", counter);
			clean_up(&(ni_usb6008[device_index]));
			up(&start_stop_sem);
			return -ENOMEM;
		}
		ni_usb6008[device_index].urbOut[counter]->dev = ni_usb6008[device_index].usbdev;
		/* will be filled later with a pointer to the comedi-device */
		/* and ONLY then the urb should be submitted */
		ni_usb6008[device_index].urbOut[counter]->context = NULL;
		ni_usb6008[device_index].urbOut[counter]->pipe =
		    usb_sndisocpipe(ni_usb6008[device_index].usbdev, ISO_OUT_EP);
		ni_usb6008[device_index].urbOut[counter]->transfer_flags = URB_ISO_ASAP;
		ni_usb6008[device_index].urbOut[counter]->transfer_buffer =
		    kzalloc(SIZE_OUT_BUF, GFP_KERNEL);
		if (!(ni_usb6008[device_index].urbOut[counter]->transfer_buffer)) {
			printk(KERN_ERR "comedi_: ni_usb6008: " 
				"could not allocate transfer buffer (%d)\n", counter);
			clean_up(&(ni_usb6008[device_index]));
			up(&start_stop_sem);
			return -ENOMEM;
		}
		//ni_usb6008[device_index].urbOut[counter]->complete = usbduxsub_ao_IsocIrq;
		ni_usb6008[device_index].urbOut[counter]->number_of_packets = 1;
		ni_usb6008[device_index].urbOut[counter]->transfer_buffer_length = SIZE_OUT_BUF;
		ni_usb6008[device_index].urbOut[counter]->iso_frame_desc[0].offset = 0;
		ni_usb6008[device_index].urbOut[counter]->iso_frame_desc[0].length =
		    SIZE_OUT_BUF;
		if (ni_usb6008[device_index].high_speed) {
			/* uframes */
			ni_usb6008[device_index].urbOut[counter]->interval = 8;
		} else {
			/* frames */
			ni_usb6008[device_index].urbOut[counter]->interval = 1;
		}
	}

	ni_usb6008[device_index].ai_cmd_running = 0;
	ni_usb6008[device_index].ao_cmd_running = 0;
	ni_usb6008[device_index].pwm_cmd_running = 0;

	/* we have reached the bottom of the function */
	ni_usb6008[device_index].probed = 1;
	up(&start_stop_sem);

	/*
	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      "usbdux_firmware.bin",
				      &udev->dev,
				      GFP_KERNEL,
				      usbduxsub + index,
				      usbdux_firmware_request_complete_handler);
				     
	if (ret) {
		dev_err(dev, "Could not load firmware (err=%d)\n", ret);
		return ret;
	}
	*/

	comedi_usb_auto_config(udev, BOARDNAME);

	printk(KERN_INFO "comedi_: ni_usb6008: " 
		 "%d has been successfully initialised\n", device_index);
	/* success */
	return 0;
}

static void ni_usb6008_disconnect(struct usb_interface *uinterf)
{
	int counter = 0;
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
	for (counter = 0; counter < NUM_NI_USB6008; counter++) {
		clean_up(&(ni_usb6008_tmp[counter]));
	}
	up(&ni_usb6008_tmp->sem);
	up(&start_stop_sem);
	printk(KERN_INFO "comedi_: ni_usb6008: disconnected from the usb\n");
}

static int ni_usb6008_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	int ret;
	int device_index;
	int counter;
	struct ni_usb6008_struct *udev;
	struct comedi_subdevice *s = NULL;

	printk(KERN_INFO "comedi_: ni_usb6008: attach called\n");

	dev->private = NULL;

	down(&start_stop_sem);

	/* find a valid device which has been detected by the probe function of the usb */
	device_index = -1;
	for (counter = 0; counter < NUM_NI_USB6008; counter++) {
		if ((ni_usb6008[counter].probed) && (!ni_usb6008[counter].attached)) {
			device_index = counter;
			break;
		}
	}

	if (device_index < 0) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"attach failed, no ni_usb6008 devs %d connected to the usb bus.\n", dev->minor);
		up(&start_stop_sem);
		return -ENODEV;
	}

	udev = &ni_usb6008[device_index];

	down(&udev->sem);

	/* pointer back to the corresponding comedi device */
	udev->comedidev = dev;

	/* trying to upload the firmware into the chip
	if (comedi_aux_data(it->options, 0) &&
	    it->options[COMEDI_DEVCONF_AUX_DATA_LENGTH]) {
		firmwareUpload(udev, comedi_aux_data(it->options, 0),
			       it->options[COMEDI_DEVCONF_AUX_DATA_LENGTH]);
	} */

	dev->board_name = BOARDNAME;

	/* set number of subdevices */
	if (udev->high_speed) {
		/* with pwm */
		dev->n_subdevices = 5;
	} else {
		/* without pwm */
		dev->n_subdevices = 4;
	}

	/* allocate space for the subdevices */
	ret = alloc_subdevices(dev, dev->n_subdevices);
	if (ret < 0) {
		printk(KERN_ERR "comedi_: ni_usb6008: "
			"error alloc space for subdev %d\n", dev->minor);
		up(&start_stop_sem);
		return ret;
	}

	printk(KERN_INFO "comedi_: ni_usb6008: "
		 "usb-device %d is attached to comedi.\n", device_index);

	/* private structure is also simply the usb-structure */
	dev->private = udev;

	/* the first subdevice is the A/D converter */
	s = dev->subdevices + SUBDEV_AD;
	/* the URBs get the comedi subdevice */
	/* which is responsible for reading */
	/* this is the subdevice which reads data */
	dev->read_subdev = s;
	/* the subdevice receives as private structure the */
	/* usb-structure */
	s->private = NULL;
	/* analog input */
	s->type = COMEDI_SUBD_AI;
	/* readable and ref is to ground */
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ;
	/* 8 channels */
	s->n_chan = 8;
	/* length of the channellist */
	s->len_chanlist = 8;
	/* callback functions */
	s->insn_read = ni_usb6008_ai_insn_read;
	//s->do_cmdtest = usbdux_ai_cmdtest;
	//s->do_cmd = usbdux_ai_cmd;
	//s->cancel = usbdux_ai_cancel;
	/* max value from the A/D converter (12bit) */
	s->maxdata = 0xfff;
	/* range table to convert to physical units */
	//s->range_table = (&range_usbdux_ai_range);

	/* analog out */
	s = dev->subdevices + SUBDEV_DA;
	/* analog out */
	s->type = COMEDI_SUBD_AO;
	/* backward pointer */
	dev->write_subdev = s;
	/* the subdevice receives as private structure the */
	/* usb-structure */
	s->private = NULL;
	/* are writable */
	s->subdev_flags = SDF_WRITABLE | SDF_GROUND | SDF_CMD_WRITE;
	/* 4 channels */
	s->n_chan = 4;
	/* length of the channellist */
	s->len_chanlist = 4;
	/* 12 bit resolution */
	s->maxdata = 0x0fff;
	/* bipolar range */
	//s->range_table = (&range_usbdux_ao_range);
	/* callback */
	//s->do_cmdtest = usbdux_ao_cmdtest;
	//s->do_cmd = usbdux_ao_cmd;
	//s->cancel = usbdux_ao_cancel;
	//s->insn_read = usbdux_ao_insn_read;
	//s->insn_write = usbdux_ao_insn_write;

	/* digital I/O */
	s = dev->subdevices + SUBDEV_DIO;
	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	s->n_chan = 8;
	s->maxdata = 1;
	s->range_table = (&range_digital);
	//s->insn_bits = usbdux_dio_insn_bits;
	//s->insn_config = usbdux_dio_insn_config;
	/* we don't use it */
	s->private = NULL;

	/* counter */
	s = dev->subdevices + SUBDEV_COUNTER;
	s->type = COMEDI_SUBD_COUNTER;
	s->subdev_flags = SDF_WRITABLE | SDF_READABLE;
	s->n_chan = 4;
	s->maxdata = 0xFFFF;
	//s->insn_read = usbdux_counter_read;
	//s->insn_write = usbdux_counter_write;
	//s->insn_config = usbdux_counter_config;

	if (udev->high_speed) {
		/* timer / pwm */
		s = dev->subdevices + SUBDEV_PWM;
		s->type = COMEDI_SUBD_PWM;
		s->subdev_flags = SDF_WRITABLE | SDF_PWM_HBRIDGE;
		s->n_chan = 8;
		/* this defines the max duty cycle resolution */
		s->maxdata = udev->sizePwmBuf;
		//s->insn_write = usbdux_pwm_write;
		//s->insn_read = usbdux_pwm_read;
		//s->insn_config = usbdux_pwm_config;
		//usbdux_pwm_period(dev, s, PWM_DEFAULT_PERIOD);
	}
	/* finally decide that it's attached */
	udev->attached = 1;

	up(&udev->sem);

	up(&start_stop_sem);

	printk(KERN_ERR "comedi_: ni_usb6008: "
		"attached to ni_usb6008 %d.\n",
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
