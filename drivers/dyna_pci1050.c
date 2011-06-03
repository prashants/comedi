/*
 * comedi/drivers/dyna_pci1050.c
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

/*
 Driver: dyna_pci1050
 Details : Developed at IIT Bombay http://www.iitb.ac.in
 Devices: Dynalog PCI 1050 DAQ Card
 Author: Prashant Shah <pshah.mumbai@gmail.com>
 Updated: 31 May 2011
 Status: Stable
 Version: 0.1 
*/

#include "../comedidev.h"
#include "comedi_pci.h"
#include "linux/semaphore.h"

#undef DPRINTK
#ifdef DEBUG
#define DPRINTK(format, args...) printk(format, ## args)
#else
#define DPRINTK(format, args...)
#endif

#define PCI_VENDOR_ID_DYNALOG           0x10b5
#define PCI_DEVICE_ID_DYNALOG_PCI_1050  0x1050
#define DRV_NAME                        "dyna_pci1050"

#define READ_TIMEOUT 50

static DECLARE_MUTEX(start_stop_sem);

static DEFINE_PCI_DEVICE_TABLE(dyna_pci1050_pci_table) = {
	{PCI_VENDOR_ID_DYNALOG, PCI_DEVICE_ID_DYNALOG_PCI_1050, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0}
};

MODULE_DEVICE_TABLE(pci, dyna_pci1050_pci_table);

static int dyna_pci1050_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it);
static int dyna_pci1050_detach(struct comedi_device *dev);

static const struct comedi_lrange range_pci1050_ai = { 3, {
							  BIP_RANGE(10),
							  BIP_RANGE(5),
							  UNI_RANGE(10)
							  }
};

static const char range_codes_pci1050_ai[] =
    { 0x00, 0x10, 0x30 };

static const struct comedi_lrange range_pci1050_ao = { 1, {
							  UNI_RANGE(10)
							  }
};

static const char range_codes_pci1050_ao[] =
    { 0x00 };

struct boardtype {
	const char *name;
	int device_id;
	int ai_chans;
	int ai_bits;
	int ao_chans;
	int ao_bits;
	int di_chans;
	int di_bits;
	int do_chans;
	int do_bits;
	const struct comedi_lrange *range_ai;
	const char *range_codes_ai;
	const struct comedi_lrange *range_ao;
	const char *range_codes_ao;
};

static const struct boardtype boardtypes[] = {
	{
	.name = "dyna_pci1050",
	.device_id = PCI_DEVICE_ID_DYNALOG_PCI_1050,
	.ai_chans = 16,
	.ai_bits = 12,
	.ao_chans = 16,
	.ao_bits = 12,
	.di_chans = 16,
	.di_bits = 16,
	.do_chans = 16,
	.do_bits = 16,
	.range_ai = &range_pci1050_ai,
	.range_codes_ai = range_codes_pci1050_ai,
	.range_ao = &range_pci1050_ao,
	.range_codes_ao = range_codes_pci1050_ao,
	},
};

static struct comedi_driver dyna_pci1050_driver = {
	.driver_name = DRV_NAME,
	.module = THIS_MODULE,
	.attach = dyna_pci1050_attach,
	.detach = dyna_pci1050_detach,
	.board_name = &boardtypes[0].name,
	.offset = sizeof(struct boardtype),
	.num_names = ARRAY_SIZE(boardtypes),
};

struct dyna_pci1050_private {
	struct pci_dev *pci_dev;	/*  ptr to PCI device */
	char valid;			/*  card is usable */
	struct semaphore sem;

	/* device base address registers */
	unsigned long BADR0, BADR1, BADR2, BADR3, BADR4, BADR5;
};

#define thisboard ((const struct boardtype *)dev->board_ptr)
#define devpriv ((struct dyna_pci1050_private *)dev->private)

/******************************************************************************/
/************************** READ WRITE FUNCTIONS ******************************/
/******************************************************************************/

/* analog input callback */
static int dyna_pci1050_insn_read_ai(struct comedi_device *dev, struct comedi_subdevice *s,
			 struct comedi_insn *insn, unsigned int *data)
{
	int n, counter;
	u16 d = 0;
	unsigned int chan, range;

	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	down(&devpriv->sem);

	/* get the channel number and range */
	chan = CR_CHAN(insn->chanspec);
	range = thisboard->range_codes_ai[CR_RANGE((insn->chanspec))];

	//printk(KERN_INFO "comedi: dyna_pci1050: channel %d range %d and n %d\n", chan, range, insn->n);

	/* convert n samples */
	for (n = 0; n < insn->n; n++) {
		/* trigger conversion */
		smp_mb(); udelay(10);
		outw_p(0x0000 + range + chan, devpriv->BADR2 + 2);
		smp_mb(); udelay(10);
		/* read data */
		for (counter = 0; counter < READ_TIMEOUT; counter++) {
			d = inw_p(devpriv->BADR2);
			/* check if read is successfull by checking the 16 bit */
			if (d & (1 << 15)) {
				goto conv_finish;
			}
		}
		data[n] = 0;
		printk(KERN_INFO "comedi: dyna_pci1050: timeout reading analog input\n");
		continue;
	conv_finish:
		/* mask the first 4 bits - EOC bits */
		d &= 0x0FFF;
		data[n] = d;
	}

	up(&devpriv->sem);

	/* return the number of samples read/written */
	return n;
}

/* analog output callback */
static int dyna_pci1050_insn_write_ao(struct comedi_device *dev,
				 struct comedi_subdevice *s,
				 struct comedi_insn *insn, unsigned int *data)
{
	int n;
	unsigned int chan, range;

	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	down(&devpriv->sem);
	chan = CR_CHAN(insn->chanspec);
	range = thisboard->range_codes_ai[CR_RANGE((insn->chanspec))];

	for (n = 0; n < insn->n; n++) {
		/* write data */
		outw_p(data[n], devpriv->BADR2);
		smp_mb(); udelay(10);
		/* trigger conversion */
		outw_p(0x0000 + range + chan, devpriv->BADR2 + 2);
		smp_mb(); udelay(10);
	}
	up(&devpriv->sem);
	return n;
}

/* digital input callback */
static int dyna_pci1050_insn_bits_di(struct comedi_device *dev,
				struct comedi_subdevice *s,
				struct comedi_insn *insn, unsigned int *data)
{
	unsigned int chan;
	u16 d = 0;

	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	down(&devpriv->sem);
	chan = CR_CHAN(insn->chanspec);

	smp_mb();
	d = inw_p(devpriv->BADR3);
	data[0] = d & (1 << chan);
	udelay(10);
	up(&devpriv->sem);
	return 2;
}

/* digital output callback */
static int dyna_pci1050_insn_bits_do(struct comedi_device *dev,
				struct comedi_subdevice *s,
				struct comedi_insn *insn, unsigned int *data)
{
	unsigned int chan;
	u16 d = 0;

	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	down(&devpriv->sem);
	chan = CR_CHAN(insn->chanspec);

	if (data[0]) {
		d = data[0] & (1 << chan);
		smp_mb();
		outw_p(d, devpriv->BADR3);
		udelay(10);
	}
	up(&devpriv->sem);
	return 2;
}

/******************************************************************************/
/*********************** INITIALIZATION FUNCTIONS *****************************/
/******************************************************************************/

static int dyna_pci1050_ns_to_timer(unsigned int *ns, int round)
{
	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	return *ns;
}

static int dyna_pci1050_ai_cmdtest(struct comedi_device *dev,
			   struct comedi_subdevice *s, struct comedi_cmd *cmd)
{
	int err = 0;
	int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_EXT;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* note that mutual compatibility is not an issue here */
	if (cmd->scan_begin_src != TRIG_TIMER &&
	    cmd->scan_begin_src != TRIG_EXT)
		err++;
	if (cmd->convert_src != TRIG_TIMER && cmd->convert_src != TRIG_EXT)
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
#define MAX_SPEED	10000	/* in nanoseconds */
#define MIN_SPEED	1000000000	/* in nanoseconds */

	if (cmd->scan_begin_src == TRIG_TIMER) {
		if (cmd->scan_begin_arg < MAX_SPEED) {
			cmd->scan_begin_arg = MAX_SPEED;
			err++;
		}
		if (cmd->scan_begin_arg > MIN_SPEED) {
			cmd->scan_begin_arg = MIN_SPEED;
			err++;
		}
	} else {
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		if (cmd->scan_begin_arg > 9) {
			cmd->scan_begin_arg = 9;
			err++;
		}
	}
	if (cmd->convert_src == TRIG_TIMER) {
		if (cmd->convert_arg < MAX_SPEED) {
			cmd->convert_arg = MAX_SPEED;
			err++;
		}
		if (cmd->convert_arg > MIN_SPEED) {
			cmd->convert_arg = MIN_SPEED;
			err++;
		}
	} else {
		/* external trigger */
		/* see above */
		if (cmd->convert_arg > 9) {
			cmd->convert_arg = 9;
			err++;
		}
	}

	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if (cmd->stop_src == TRIG_COUNT) {
		if (cmd->stop_arg > 0x00ffffff) {
			cmd->stop_arg = 0x00ffffff;
			err++;
		}
	} else {
		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	if (cmd->scan_begin_src == TRIG_TIMER) {
		tmp = cmd->scan_begin_arg;
		dyna_pci1050_ns_to_timer(&cmd->scan_begin_arg,
				 cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->scan_begin_arg)
			err++;
	}
	if (cmd->convert_src == TRIG_TIMER) {
		tmp = cmd->convert_arg;
		dyna_pci1050_ns_to_timer(&cmd->convert_arg,
				 cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->convert_arg)
			err++;
		if (cmd->scan_begin_src == TRIG_TIMER &&
		    cmd->scan_begin_arg <
		    cmd->convert_arg * cmd->scan_end_arg) {
			cmd->scan_begin_arg =
			    cmd->convert_arg * cmd->scan_end_arg;
			err++;
		}
	}

	if (err)
		return 4;

	return 0;
}

static int dyna_pci1050_ai_cmd(struct comedi_device *dev, struct comedi_subdevice *s)
{
	struct comedi_cmd *cmd = &s->async->cmd;

	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	/*  test if cmd is valid */
	if (cmd == NULL) {
		printk(KERN_INFO "comedi: dyna_pci1050: NULL command\n");
		return -EINVAL;
	} else {
		printk(KERN_INFO "comedi: dyna_pci1050: command recieved\n");
	}

	//s->async->cur_chan = 0;
	//s->async->inttrig = NULL;

	return -EINVAL;
}

/******************************************************************************/
/*********************** INITIALIZATION FUNCTIONS *****************************/
/******************************************************************************/

static int dyna_pci1050_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it)
{
	struct comedi_subdevice *s;
	struct pci_dev *pcidev;
	unsigned int opt_bus, opt_slot;
	int board_index, i;

	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	printk(KERN_INFO "comedi: dyna_pci1050: minor number %d\n", dev->minor);

	down(&start_stop_sem);

	if (alloc_private(dev, sizeof(struct dyna_pci1050_private)) < 0) {
		printk(KERN_ERR "comedi: dyna_pci1050: failed to allocate memory!\n");
		up(&start_stop_sem);
		return -ENOMEM;
	}

	opt_bus = it->options[0];
	opt_slot = it->options[1];
	dev->board_name = thisboard->name;
	dev->irq = 0;

	/*
	 * Probe the PCI bus and located the matching device
	 */
	for (pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
		pcidev != NULL;
		pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcidev)) {

		board_index = -1;
		for (i = 0; i < ARRAY_SIZE(boardtypes); ++i) {
			if ((pcidev->vendor == PCI_VENDOR_ID_DYNALOG) &&
				(pcidev->device == boardtypes[i].device_id)) {
					board_index = i;
					break;
				}
		}
		if (board_index < 0)
			continue;

		/* Found matching vendor/device. */
		if (opt_bus || opt_slot) {
			/* Check bus/slot. */
			if (opt_bus != pcidev->bus->number
			    || opt_slot != PCI_SLOT(pcidev->devfn))
				continue;	/* no match */
		}

		goto found;
	}
	printk("comedi: dyna_pci1050: no supported device found!\n");
	up(&start_stop_sem);
	return -EIO;

found:

	if (!pcidev) {
		if (opt_bus || opt_slot) {
			printk(KERN_ERR "comedi: dyna_pci1050: invalid PCI device at b:s %d:%d\n", opt_bus, opt_slot);
		} else {
			printk(KERN_ERR "comedi: dyna_pci1050: invalid PCI device\n");
		}
		up(&start_stop_sem);
		return -EIO;
	}

	if (comedi_pci_enable(pcidev, DRV_NAME)) {
		printk(KERN_ERR "comedi: dyna_pci1050: failed to enable PCI device and request regions!");
		up(&start_stop_sem);
		return -EIO;
	}

	init_MUTEX(&devpriv->sem);
	dev->board_ptr = &boardtypes[board_index];
	devpriv->pci_dev = pcidev;

	printk(KERN_INFO "comedi: dyna_pci1050: device found!\n");

	/* initialize device base address registers */
	devpriv->BADR0 = pci_resource_start(pcidev, 0);
	devpriv->BADR1 = pci_resource_start(pcidev, 1);
	devpriv->BADR2 = pci_resource_start(pcidev, 2);
	devpriv->BADR3 = pci_resource_start(pcidev, 3);
	devpriv->BADR4 = pci_resource_start(pcidev, 4);
	devpriv->BADR5 = pci_resource_start(pcidev, 5);

	printk(KERN_INFO "comedi: dyna_pci1050: iobase 0x%lx : 0x%lx : 0x%lx : 0x%lx : 0x%lx : 0x%lx\n",
               devpriv->BADR0, devpriv->BADR1, devpriv->BADR2, devpriv->BADR3, devpriv->BADR4, devpriv->BADR5);

	if (alloc_subdevices(dev, 4) < 0) {
		printk(KERN_ERR "comedi: dyna_pci1050: failed allocating subdevices\n");
		up(&start_stop_sem);
		return -ENOMEM;
	}

	/* analog input */
	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	s->n_chan = thisboard->ai_chans;
	s->maxdata = 0x0FFF;
	s->range_table = thisboard->range_ai;
	s->len_chanlist = 16;
	s->insn_read = dyna_pci1050_insn_read_ai;
	s->subdev_flags |= SDF_CMD_READ;
	s->do_cmd = dyna_pci1050_ai_cmd;
	s->do_cmdtest = dyna_pci1050_ai_cmdtest;

	/* analog output */
	s = dev->subdevices + 1;
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE;
	s->n_chan = thisboard->ao_chans;
	s->maxdata = 0x0FFF;
	s->range_table = thisboard->range_ao;
	s->len_chanlist = 16;
	s->insn_write = dyna_pci1050_insn_write_ao;

	/* digital input */
	s = dev->subdevices + 2;
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND;
	s->n_chan = thisboard->di_chans;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->len_chanlist = thisboard->di_chans;
	//s->io_bits = 0;	/* all bits input */
	s->insn_read = dyna_pci1050_insn_bits_di;

	/* digital output */
	s = dev->subdevices + 3;
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITABLE | SDF_GROUND;
	s->n_chan = thisboard->do_chans;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->len_chanlist = thisboard->do_chans;
	//s->state = 0;
	//s->io_bits = (1 << thisboard->do_chans) - 1;	/* all bits output */
	s->insn_write = dyna_pci1050_insn_bits_do;

	devpriv->valid = 1;
	up(&start_stop_sem);

	printk(KERN_INFO "comedi: dyna_pci1050: device setup completed\n");

	return 1;
}

static int dyna_pci1050_detach(struct comedi_device *dev)
{
	printk(KERN_DEBUG "comedi: dyna_pci1050: %s\n", __func__);

	if (devpriv->pci_dev) {
		comedi_pci_disable(devpriv->pci_dev);
	}

	return 0;
}

COMEDI_PCI_INITCLEANUP(dyna_pci1050_driver, dyna_pci1050_pci_table);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Prashant Shah <pshah.mumbai@gmail.com>");

