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

#define BOARDNAME		"ni_usb6008"
#define MAX_DEVICES		16

#define SIZE_IN_BUF		64		
#define SIZE_OUT_BUF		64
#define SIZE_INSN_BUF		64
#define SIZE_DUX_CMD_BUF	8
#define SIZE_DAX_CMD_BUF	8

/* Input endpoint number */
#define INPUT_ENDPOINT_NUM	6
/* Output endpoint number */
#define OUTPUT_ENDPOINT_NUM	2

#define BULK_TIMEOUT		1000
#define RETRIES			10

#define COMMAND_OUT_EP     	1
#define COMMAND_IN_EP        	8

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
	struct urb *urbOut;
	struct urb *urbPwm;

	/* PWM period */
	unsigned int pwmPeriod;
	/* PWM internal delay for the GPIF in the FX2 */
	int8_t pwmDelay;
	/* size of the PWM buffer which holds the bit pattern */
	int sizePwmBuf;

	int16_t *inBuffer;
	int16_t *outBuffer;
	int16_t *insnBuffer;

	int8_t *dac_commands;
	int8_t *dux_commands;

	short int ai_cmd_running;
	short int ao_cmd_running;
	short int pwm_cmd_running;

	short int ai_continous;
	short int ao_continous;

	int ai_sample_count;
	int ao_sample_count;

	unsigned int ai_timer;
	unsigned int ao_timer;

	unsigned int ai_counter;
	unsigned int ao_counter;

	unsigned int ai_interval;
	unsigned int a0_interval;

	struct semaphore sem;
};

static struct ni_usb6008_struct ni_usb6008[MAX_DEVICES];


/************************ STOP OR CANCEL FUNCTIONS ****************************/

/*
 * Stops the data acquision
 * It should be safe to call this function from any context
 */
static int ni_usb6008_unlink_InURBs(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	int ret = 0;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (ni_usb6008_tmp && ni_usb6008_tmp->urbIn) {
		usb_kill_urb(ni_usb6008_tmp->urbIn);
	}
	return ret;
}

/*
 * This will stop a running acquisition operation
 * Is called from within this driver from both the
 * interrupt context and from comedi
 */
static int ni_usb6008_ai_stop(struct ni_usb6008_struct *ni_usb6008_tmp, int do_unlink)
{
	int ret = 0;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp) {
		pr_err("comedi?: ni_usb6008_ai_stop: ni_usb6008_tmp=NULL!\n");
		return -EFAULT;
	}

	if (do_unlink)
		ret = ni_usb6008_unlink_InURBs(ni_usb6008_tmp);

	ni_usb6008_tmp->ai_cmd_running = 0;

	return ret;
}

/*
 * This will cancel a running acquisition operation.
 * This is called by comedi but never from inside the driver.
 */
static int ni_usb6008_ai_cancel(struct comedi_device *dev,
			    struct comedi_subdevice *s)
{
	struct ni_usb6008_struct *ni_usb6008_tmp;
	int res = 0;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

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

static int ni_usb6008_unlink_OutURBs(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	int ret = 0;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (ni_usb6008_tmp && ni_usb6008_tmp->urbOut)
		usb_kill_urb(ni_usb6008_tmp->urbOut);

	return ret;
}

/* This will cancel a running acquisition operation
 * in any context.
 */
static int ni_usb6008_ao_stop(struct ni_usb6008_struct *ni_usb6008_tmp, int do_unlink)
{
	int ret = 0;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp)
		return -EFAULT;

	if (do_unlink)
		ret = ni_usb6008_unlink_OutURBs(ni_usb6008_tmp);

	ni_usb6008_tmp->ao_cmd_running = 0;

	return ret;
}

static void tidy_up(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp)
		return;

	printk(KERN_INFO "comedi_: ni_usb6008: tiding up\n");

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
	if (ni_usb6008_tmp->urbOut) {
		if (ni_usb6008_tmp->ao_cmd_running) {
			ni_usb6008_tmp->ao_cmd_running = 0;
			ni_usb6008_unlink_OutURBs(ni_usb6008_tmp);
		}
		if (ni_usb6008_tmp->urbOut->transfer_buffer) {
			kfree(ni_usb6008_tmp->urbOut->transfer_buffer);
			ni_usb6008_tmp->urbOut->transfer_buffer = NULL;
		}
		usb_kill_urb(ni_usb6008_tmp->urbOut);
		usb_free_urb(ni_usb6008_tmp->urbOut);
		ni_usb6008_tmp->urbOut = NULL;
		kfree(ni_usb6008_tmp->urbOut);
		ni_usb6008_tmp->urbOut = NULL;
	}
	if (ni_usb6008_tmp->urbPwm) {
		if (ni_usb6008_tmp->pwm_cmd_running) {
			ni_usb6008_tmp->pwm_cmd_running = 0;
		}
		if (ni_usb6008_tmp->urbPwm->transfer_buffer) {
			kfree(ni_usb6008_tmp->urbPwm->transfer_buffer);
			ni_usb6008_tmp->urbPwm->transfer_buffer = NULL;
		}
		ni_usb6008_tmp->urbPwm = NULL;
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
	ni_usb6008_tmp->pwm_cmd_running = 0;
}

/************************ URB SUBMISSION FUNCTIONS ****************************/

static int ni_usb6008_submit_InURBs(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	int ret;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

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
		printk(KERN_INFO "comedi_: ni_usb6008: usb_submit_urb error %d\n", ret);
		return ret;
	}
	return 0;
}

static int ni_usb6008_submit_OutURBs(struct ni_usb6008_struct *ni_usb6008_tmp)
{
	int ret;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp)
		return -EFAULT;

	ni_usb6008_tmp->urbOut->context = ni_usb6008_tmp->comedidev;
	ni_usb6008_tmp->urbOut->dev = ni_usb6008_tmp->usbdev;
	ni_usb6008_tmp->urbOut->status = 0;
	ni_usb6008_tmp->urbOut->transfer_flags = URB_ISO_ASAP;

	ret = usb_submit_urb(ni_usb6008_tmp->urbOut, GFP_ATOMIC);
	if (ret) {
		printk(KERN_INFO "comedi_: ni_usb6008: usb_submit_urb error %d\n", ret);
		return ret;
	}
	return 0;
}

/************************ MISCELLANEOUS FUNCTIONS *****************************/

/*
 * creates the ADC command for the MAX1271
 * range is the range value from comedi
 */
static int8_t create_adc_command(unsigned int chan, int range)
{
	int8_t p = (range <= 1);
	int8_t r = ((range % 2) == 0);
	return (chan << 4) | ((p == 1) << 2) | ((r == 1) << 3);
}

static int send_dux_commands(struct ni_usb6008_struct *ni_usb6008_tmp, int cmd_type)
{
	int result, nsent;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	ni_usb6008_tmp->dux_commands[0] = cmd_type;

	result = usb_bulk_msg(ni_usb6008_tmp->usbdev,
		usb_sndbulkpipe(ni_usb6008_tmp->usbdev, COMMAND_OUT_EP),
		ni_usb6008_tmp->dux_commands, SIZE_DUX_CMD_BUF,
		&nsent,	BULK_TIMEOUT);
	if (result < 0)
		printk(KERN_INFO "comedi%d: ni_usb6008: could not transmit dux_command to the usb-device, err=%d\n", ni_usb6008_tmp->comedidev->minor, result);

	return result;
}

static int receive_dux_commands(struct ni_usb6008_struct *ni_usb6008_tmp, int command)
{
	int result = (-EFAULT);
	int nrec;
	int i;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	for (i = 0; i < RETRIES; i++) {
		result = usb_bulk_msg(ni_usb6008_tmp->usbdev,
				      usb_rcvbulkpipe(ni_usb6008_tmp->usbdev,
						      COMMAND_IN_EP),
				      ni_usb6008_tmp->insnBuffer, SIZE_INSN_BUF,
				      &nrec, BULK_TIMEOUT);
		if (result < 0) {
			dev_err(&ni_usb6008_tmp->interface->dev, "comedi%d: "
				"insn: USB error %d while receiving DUX command"
				"\n", ni_usb6008_tmp->comedidev->minor, result);
			return result;
		}
		if (le16_to_cpu(ni_usb6008_tmp->insnBuffer[0]) == command)
			return result;
	}
	/* this is only reached if the data has been requested a couple of
	 * times */
	dev_err(&ni_usb6008_tmp->interface->dev, "comedi%d: insn: "
		"wrong data returned from firmware: want cmd %d, got cmd %d.\n",
		ni_usb6008_tmp->comedidev->minor, command,
		le16_to_cpu(ni_usb6008_tmp->insnBuffer[0]));
	return -EFAULT;
}

/************************ DATA TRANSFER FUNCTIONS *****************************/

static int ni_usb6008_ai_cmdtest(struct comedi_device *dev,
			     struct comedi_subdevice *s, struct comedi_cmd *cmd)
{
	int err = 0, tmp;
	unsigned int tmpTimer;
	struct ni_usb6008_struct *this_usbduxsub = dev->private;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!(this_usbduxsub->probed))
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

static int ni_usb6008_ai_inttrig(struct comedi_device *dev,
			     struct comedi_subdevice *s, unsigned int trignum)
{
	int ret;
	struct ni_usb6008_struct *ni_usb6008_tmp = dev->private;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp)
		return -EFAULT;

	down(&ni_usb6008_tmp->sem);
	if (!(ni_usb6008_tmp->probed)) {
		up(&ni_usb6008_tmp->sem);
		return -ENODEV;
	}

	if (trignum != 0) {
		printk(KERN_INFO "comedi%d: ni_usb6008: ni_usb6008_ai_inttrig: invalid trignum\n", dev->minor);
		up(&ni_usb6008_tmp->sem);
		return -EINVAL;
	}
	if (!(ni_usb6008_tmp->ai_cmd_running)) {
		ni_usb6008_tmp->ai_cmd_running = 1;
		ret = ni_usb6008_submit_InURBs(ni_usb6008_tmp);
		if (ret < 0) {
			ni_usb6008_tmp->ai_cmd_running = 0;
			up(&ni_usb6008_tmp->sem);
			return ret;
		}
		s->async->inttrig = NULL;
	} else {
		printk(KERN_INFO "comedi%d: ni_usb6008: ai_inttrig but acqu is already running\n", dev->minor);
	}
	up(&ni_usb6008_tmp->sem);
	return 1;
}

static int ni_usb6008_ai_cmd(struct comedi_device *dev, struct comedi_subdevice *s)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	unsigned int chan, range;
	int i, ret;
	struct ni_usb6008_struct *this_usbduxsub = dev->private;
	int result;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!this_usbduxsub)
		return -EFAULT;

	dev_dbg(&this_usbduxsub->interface->dev,
		"comedi%d: usbdux_ai_cmd\n", dev->minor);

	/* block other CPUs from starting an ai_cmd */
	down(&this_usbduxsub->sem);

	if (!(this_usbduxsub->probed)) {
		up(&this_usbduxsub->sem);
		return -ENODEV;
	}
	if (this_usbduxsub->ai_cmd_running) {
		dev_err(&this_usbduxsub->interface->dev, "comedi%d: "
			"ai_cmd not possible. Another ai_cmd is running.\n",
			dev->minor);
		up(&this_usbduxsub->sem);
		return -EBUSY;
	}
	/* set current channel of the running aquisition to zero */
	s->async->cur_chan = 0;

	this_usbduxsub->dux_commands[1] = cmd->chanlist_len;
	for (i = 0; i < cmd->chanlist_len; ++i) {
		chan = CR_CHAN(cmd->chanlist[i]);
		range = CR_RANGE(cmd->chanlist[i]);
		if (i >= SIZE_DUX_CMD_BUF) {
			dev_err(&this_usbduxsub->interface->dev,
				"comedi%d: channel list too long\n",
				dev->minor);
			break;
		}
		this_usbduxsub->dux_commands[i + 2] =
		    create_adc_command(chan, range);
	}

	dev_dbg(&this_usbduxsub->interface->dev,
		"comedi %d: sending commands to the usb device: size=%u\n",
		dev->minor, SIZE_DUX_CMD_BUF);

	result = send_dux_commands(this_usbduxsub, SIZE_DUX_CMD_BUF);
	if (result < 0) {
		up(&this_usbduxsub->sem);
		return result;
	}

	/* interval always 1ms */
	this_usbduxsub->ai_interval = 1;
	this_usbduxsub->ai_timer = cmd->scan_begin_arg / 1000000;

	if (this_usbduxsub->ai_timer < 1) {
		dev_err(&this_usbduxsub->interface->dev, "comedi%d: ai_cmd: "
			"timer=%d, scan_begin_arg=%d. "
			"Not properly tested by cmdtest?\n", dev->minor,
			this_usbduxsub->ai_timer, cmd->scan_begin_arg);
		up(&this_usbduxsub->sem);
		return -EINVAL;
	}
	this_usbduxsub->ai_counter = this_usbduxsub->ai_timer;

	if (cmd->stop_src == TRIG_COUNT) {
		/* data arrives as one packet */
		this_usbduxsub->ai_sample_count = cmd->stop_arg;
		this_usbduxsub->ai_continous = 0;
	} else {
		/* continous aquisition */
		this_usbduxsub->ai_continous = 1;
		this_usbduxsub->ai_sample_count = 0;
	}

	if (cmd->start_src == TRIG_NOW) {
		/* enable this acquisition operation */
		this_usbduxsub->ai_cmd_running = 1;
		ret = ni_usb6008_submit_InURBs(this_usbduxsub);
		if (ret < 0) {
			this_usbduxsub->ai_cmd_running = 0;
			/* fixme: unlink here?? */
			up(&this_usbduxsub->sem);
			return ret;
		}
		s->async->inttrig = NULL;
	} else {
		/* TRIG_INT */
		/* don't enable the acquision operation */
		/* wait for an internal signal */
		s->async->inttrig = ni_usb6008_ai_inttrig;
	}
	up(&this_usbduxsub->sem);
	return 0;
}

/* Mode 0 is used to get a single conversion on demand */
static int ni_usb6008_ai_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	int i;
	unsigned int one = 0;
	int chan, range;
	int err;
	struct ni_usb6008_struct *ni_usb6008_tmp = dev->private;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	if (!ni_usb6008_tmp)
		return 0;

	down(&ni_usb6008_tmp->sem);
	if (!(ni_usb6008_tmp->probed)) {
		up(&ni_usb6008_tmp->sem);
		return -ENODEV;
	}
	if (ni_usb6008_tmp->ai_cmd_running) {
		dev_err(&ni_usb6008_tmp->interface->dev,
			"comedi%d: ai_insn_read not possible. "
			"Async Command is running.\n", dev->minor);
		up(&ni_usb6008_tmp->sem);
		return 0;
	}

	/* sample one channel */
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);
	/* set command for the first channel */
	ni_usb6008_tmp->dux_commands[1] = create_adc_command(chan, range);

	/* adc commands */
	err = send_dux_commands(ni_usb6008_tmp, 4); /**** SENDSINGLEAD ****/
	if (err < 0) {
		up(&ni_usb6008_tmp->sem);
		return err;
	}

	for (i = 0; i < insn->n; i++) {
		err = receive_dux_commands(ni_usb6008_tmp, 4); /**** SENDSINGLEAD ****/
		if (err < 0) {
			up(&ni_usb6008_tmp->sem);
			return 0;
		}
		one = le16_to_cpu(ni_usb6008_tmp->insnBuffer[1]);
		if (CR_RANGE(insn->chanspec) <= 1)
			one = one ^ 0x800;

		data[i] = one;
	}
	up(&ni_usb6008_tmp->sem);
	return i;
}

/******************** INTERRUPT DATA TARANSFER FUNCTIONS **********************/

/* analogue IN - interrupt service routine */
static void ni_usb6008_ai_IsocIrq(struct urb *urb)
{
	int i, err, n;
	struct ni_usb6008_struct *this_usbduxsub;
	struct comedi_device *this_comedidev;
	struct comedi_subdevice *s;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	/* the context variable points to the subdevice */
	this_comedidev = urb->context;
	/* the private structure of the subdevice is struct usbduxsub */
	this_usbduxsub = this_comedidev->private;
	/* subdevice which is the AD converter */
	s = this_comedidev->subdevices + 0;

	/* first we test if something unusual has just happened */
	switch (urb->status) {
	case 0:
		/* copy the result in the transfer buffer */
		memcpy(this_usbduxsub->inBuffer,
		       urb->transfer_buffer, SIZE_IN_BUF);
		break;
	case -EILSEQ:
		/* error in the ISOchronous data */
		/* we don't copy the data into the transfer buffer */
		/* and recycle the last data byte */
		dev_dbg(&urb->dev->dev,
			"comedi%d: usbdux: CRC error in ISO IN stream.\n",
			this_usbduxsub->comedidev->minor);

		break;

	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
	case -ECONNABORTED:
		/* happens after an unlink command */
		if (this_usbduxsub->ai_cmd_running) {
			/* we are still running a command */
			/* tell this comedi */
			s->async->events |= COMEDI_CB_EOA;
			s->async->events |= COMEDI_CB_ERROR;
			comedi_event(this_usbduxsub->comedidev, s);
			/* stop the transfer w/o unlink */
			ni_usb6008_ai_stop(this_usbduxsub, 0);
		}
		return;

	default:
		/* a real error on the bus */
		/* pass error to comedi if we are really running a command */
		if (this_usbduxsub->ai_cmd_running) {
			dev_err(&urb->dev->dev,
				"Non-zero urb status received in ai intr "
				"context: %d\n", urb->status);
			s->async->events |= COMEDI_CB_EOA;
			s->async->events |= COMEDI_CB_ERROR;
			comedi_event(this_usbduxsub->comedidev, s);
			/* don't do an unlink here */
			ni_usb6008_ai_stop(this_usbduxsub, 0);
		}
		return;
	}

	/*
	 * at this point we are reasonably sure that nothing dodgy has happened
	 * are we running a command?
	 */
	if (unlikely((!(this_usbduxsub->ai_cmd_running)))) {
		/*
		 * not running a command, do not continue execution if no
		 * asynchronous command is running in particular not resubmit
		 */
		return;
	}

	urb->dev = this_usbduxsub->usbdev;

	/* resubmit the urb */
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (unlikely(err < 0)) {
		dev_err(&urb->dev->dev,
			"comedi_: urb resubmit failed in int-context! err=%d\n",
			err);
		if (err == -EL2NSYNC)
			dev_err(&urb->dev->dev,
				"buggy USB host controller or bug in IRQ "
				"handler!\n");
		s->async->events |= COMEDI_CB_EOA;
		s->async->events |= COMEDI_CB_ERROR;
		comedi_event(this_usbduxsub->comedidev, s);
		/* don't do an unlink here */
		ni_usb6008_ai_stop(this_usbduxsub, 0);
		return;
	}

	this_usbduxsub->ai_counter--;
	if (likely(this_usbduxsub->ai_counter > 0))
		return;

	/* timer zero, transfer measurements to comedi */
	this_usbduxsub->ai_counter = this_usbduxsub->ai_timer;

	/* test, if we transmit only a fixed number of samples */
	if (!(this_usbduxsub->ai_continous)) {
		/* not continous, fixed number of samples */
		this_usbduxsub->ai_sample_count--;
		/* all samples received? */
		if (this_usbduxsub->ai_sample_count < 0) {
			/* prevent a resubmit next time */
			ni_usb6008_ai_stop(this_usbduxsub, 0);
			/* say comedi that the acquistion is over */
			s->async->events |= COMEDI_CB_EOA;
			comedi_event(this_usbduxsub->comedidev, s);
			return;
		}
	}
	/* get the data from the USB bus and hand it over to comedi */
	n = s->async->cmd.chanlist_len;
	for (i = 0; i < n; i++) {
		/* transfer data */
		if (CR_RANGE(s->async->cmd.chanlist[i]) <= 1) {
			err = comedi_buf_put
			    (s->async,
			     le16_to_cpu(this_usbduxsub->inBuffer[i]) ^ 0x800);
		} else {
			err = comedi_buf_put
			    (s->async,
			     le16_to_cpu(this_usbduxsub->inBuffer[i]));
		}
		if (unlikely(err == 0)) {
			/* buffer overflow */
			ni_usb6008_ai_stop(this_usbduxsub, 0);
			return;
		}
	}
	/* tell comedi that data is there */
	s->async->events |= COMEDI_CB_BLOCK | COMEDI_CB_EOS;
	comedi_event(this_usbduxsub->comedidev, s);
}

static void ni_usb6008_ao_IsocIrq(struct urb *urb)
{
	int i, ret;
	int8_t *datap;
	struct ni_usb6008_struct *this_usbduxsub;
	struct comedi_device *this_comedidev;
	struct comedi_subdevice *s;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	/* the context variable points to the subdevice */
	this_comedidev = urb->context;
	/* the private structure of the subdevice is struct usbduxsub */
	this_usbduxsub = this_comedidev->private;

	s = this_comedidev->subdevices + 0;

	switch (urb->status) {
	case 0:
		/* success */
		break;

	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
	case -ECONNABORTED:
		/* after an unlink command, unplug, ... etc */
		/* no unlink needed here. Already shutting down. */
		if (this_usbduxsub->ao_cmd_running) {
			s->async->events |= COMEDI_CB_EOA;
			comedi_event(this_usbduxsub->comedidev, s);
			ni_usb6008_ao_stop(this_usbduxsub, 0);
		}
		return;

	default:
		/* a real error */
		if (this_usbduxsub->ao_cmd_running) {
			dev_err(&urb->dev->dev,
				"comedi_: Non-zero urb status received in ao "
				"intr context: %d\n", urb->status);
			s->async->events |= COMEDI_CB_ERROR;
			s->async->events |= COMEDI_CB_EOA;
			comedi_event(this_usbduxsub->comedidev, s);
			/* we do an unlink if we are in the high speed mode */
			ni_usb6008_ao_stop(this_usbduxsub, 0);
		}
		return;
	}

	/* are we actually running? */
	if (!(this_usbduxsub->ao_cmd_running))
		return;

	/* normal operation: executing a command in this subdevice */
	this_usbduxsub->ao_counter--;
	if (this_usbduxsub->ao_counter <= 0) {
		/* timer zero */
		this_usbduxsub->ao_counter = this_usbduxsub->ao_timer;

		/* handle non continous aquisition */
		if (!(this_usbduxsub->ao_continous)) {
			/* fixed number of samples */
			this_usbduxsub->ao_sample_count--;
			if (this_usbduxsub->ao_sample_count < 0) {
				/* all samples transmitted */
				ni_usb6008_ao_stop(this_usbduxsub, 0);
				s->async->events |= COMEDI_CB_EOA;
				comedi_event(this_usbduxsub->comedidev, s);
				/* no resubmit of the urb */
				return;
			}
		}
		/* transmit data to the USB bus */
		((uint8_t *) (urb->transfer_buffer))[0] =
		    s->async->cmd.chanlist_len;
		for (i = 0; i < s->async->cmd.chanlist_len; i++) {
			short temp;
			if (i >= SIZE_DUX_CMD_BUF)
				break;

			/* pointer to the DA */
			datap =
			    (&(((int8_t *) urb->transfer_buffer)[i * 3 + 1]));
			/* get the data from comedi */
			ret = comedi_buf_get(s->async, &temp);
			datap[0] = temp;
			datap[1] = temp >> 8;
			datap[2] = this_usbduxsub->dac_commands[i];
			/* printk("data[0]=%x, data[1]=%x, data[2]=%x\n", */
			/* datap[0],datap[1],datap[2]); */
			if (ret < 0) {
				dev_err(&urb->dev->dev,
					"comedi: buffer underflow\n");
				s->async->events |= COMEDI_CB_EOA;
				s->async->events |= COMEDI_CB_OVERFLOW;
			}
			/* transmit data to comedi */
			s->async->events |= COMEDI_CB_BLOCK;
			comedi_event(this_usbduxsub->comedidev, s);
		}
	}
	urb->transfer_buffer_length = SIZE_OUT_BUF;
	urb->dev = this_usbduxsub->usbdev;
	urb->status = 0;
	if (this_usbduxsub->ao_cmd_running) {
		urb->interval = 1;
		urb->number_of_packets = 1;
		urb->iso_frame_desc[0].offset = 0;
		urb->iso_frame_desc[0].length = SIZE_OUT_BUF;
		urb->iso_frame_desc[0].status = 0;
		ret = usb_submit_urb(urb, GFP_ATOMIC);
		if (ret < 0) {
			dev_err(&urb->dev->dev,
				"comedi_: ao urb resubm failed in int-cont. "
				"ret=%d", ret);
			if (ret == EL2NSYNC)
				dev_err(&urb->dev->dev,
					"buggy USB host controller or bug in "
					"IRQ handling!\n");

			s->async->events |= COMEDI_CB_EOA;
			s->async->events |= COMEDI_CB_ERROR;
			comedi_event(this_usbduxsub->comedidev, s);
			/* don't do an unlink here */
			ni_usb6008_ao_stop(this_usbduxsub, 0);
		}
	}
}

/************************ INITIALIZATION FUNCTIONS ****************************/

static void ni_usb6008_firmware_request_complete_handler(const struct firmware *fw,
						     void *context)
{
	struct ni_usb6008_struct *ni_usb6008_tmp = context;
	struct usb_device *usbdev = ni_usb6008_tmp->usbdev;
	int ret;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	ret = comedi_usb_auto_config(usbdev, BOARDNAME);
	printk(KERN_INFO "comedi_: ni_usb6008_: comedi_usb_auto_config: %d\n", ret);
}


static int ni_usb6008_probe(struct usb_interface *uinterf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(uinterf);
	struct device *dev = &uinterf->dev;
	int i;
	int index;
	int ret;

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	printk(KERN_INFO "comedi_: ni_usb6008_: finding a free structure for the usb-device\n");

	down(&start_stop_sem);
	index = -1;
	for (i = 0; i < MAX_DEVICES; i++) {
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
	ni_usb6008[index].dux_commands = kzalloc(SIZE_DUX_CMD_BUF, GFP_KERNEL);
	if (!ni_usb6008[index].dux_commands) {
		dev_err(dev, "comedi_: ni_usb6008: error alloc space for device commands\n");
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].dac_commands = kzalloc(SIZE_DAX_CMD_BUF, GFP_KERNEL);
	if (!ni_usb6008[index].dac_commands) {
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
		tidy_up(&(ni_usb6008[index]));
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
	ni_usb6008[index].urbIn->pipe = usb_rcvisocpipe(ni_usb6008[index].usbdev, INPUT_ENDPOINT_NUM);
	ni_usb6008[index].urbIn->transfer_flags = URB_ISO_ASAP;
	ni_usb6008[index].urbIn->transfer_buffer = kzalloc(SIZE_IN_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].urbIn->transfer_buffer)) {
		printk(KERN_INFO "comedi_: ni_usb6008%d: could not alloc. transb.\n", index);
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbIn->complete = ni_usb6008_ai_IsocIrq;
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
	ni_usb6008[index].urbOut->pipe = usb_sndisocpipe(ni_usb6008[index].usbdev, OUTPUT_ENDPOINT_NUM);
	ni_usb6008[index].urbOut->transfer_flags = URB_ISO_ASAP;
	ni_usb6008[index].urbOut->transfer_buffer = kzalloc(SIZE_OUT_BUF, GFP_KERNEL);
	if (!(ni_usb6008[index].urbOut->transfer_buffer)) {
		printk(KERN_INFO "comedi_: ni_usb6008%d: could not alloc. transb.\n", index);
		tidy_up(&(ni_usb6008[index]));
		up(&start_stop_sem);
		return -ENOMEM;
	}
	ni_usb6008[index].urbOut->complete = ni_usb6008_ao_IsocIrq;
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

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

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

	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);

	down(&start_stop_sem);
	index = -1;
	for (i = 0; i < MAX_DEVICES; i++) {
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
	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);
	printk(KERN_INFO KBUILD_MODNAME ": "
	       DRIVER_VERSION ":" DRIVER_DESC "\n");
	usb_register(&usb_ni_usb6008_driver);
	comedi_driver_register(&comedi_ni_usb6008_driver);
	return 0;
}

/* deregistering the comedi driver and the usb-subsystem */
static void __exit exit_ni_usb6008(void)
{
	printk(KERN_INFO "comedi: ni_usb6008: %s\n", __func__);
	comedi_driver_unregister(&comedi_ni_usb6008_driver);
	usb_deregister(&usb_ni_usb6008_driver);
}

module_init(init_ni_usb6008);
module_exit(exit_ni_usb6008);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");