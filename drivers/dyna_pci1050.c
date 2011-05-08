#include <linux/interrupt.h>
#include <linux/sched.h>
#include "../comedidev.h"
#include "comedi_pci.h"

#undef DPRINTK
#define DPRINTK(fmt, args...) printk(KERN_INFO fmt, ## args)


#define PCI_VENDOR_ID_DYNALOG		0x10b5
#define PCI_DEVICE_ID_DYNALOG_PCI_1050	0x1050
#define DRV_NAME			"dyna_pci1050"

static int dyna_pci1050_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it);
static int dyna_pci1050_detach(struct comedi_device *dev);

struct boardtype {
        const char *name;       /*  board name */
        int device_id;
        int n_aichan;           /*  num of A/D chans */
        int n_aochan;           /*  num of D/A chans */
        int n_dichan;           /*  num of DI chans */
        int n_dochan;           /*  num of DO chans */
        int n_counter;          /*  num of counters */
        int ai_maxdata;         /*  resolution of A/D */
        int ao_maxdata;         /*  resolution of D/A */
        const struct comedi_lrange *rangelist_ai;       /*  rangelist for A/D */
        const struct comedi_lrange *rangelist_ao;       /*  rangelist for D/A */
	unsigned int ai_ns_min;	/*  max sample speed of card v ns */
};

static DEFINE_PCI_DEVICE_TABLE(dyna_pci1050_table) = {
	{PCI_VENDOR_ID_DYNALOG, PCI_DEVICE_ID_DYNALOG_PCI_1050, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0}
};

MODULE_DEVICE_TABLE(pci, dyna_pci1050_table);

static const struct comedi_lrange range_ai = {3, {
						BIP_RANGE(10),
						BIP_RANGE(5),
						UNI_RANGE(10)
					 }
};

static const struct comedi_lrange range_ao = {1, {
						UNI_RANGE(10)
					 }
};

static const struct boardtype boardtypes[] = {
	{"dyna_pci1050", 0x1050, 16, 0, 0, 0, 0, 0x0fff, 0x0fff, &range_ai, &range_ao, 100},
	/*  dummy entry corresponding to driver name */
	{.name = DRV_NAME},
};

#define n_boardtypes (sizeof(boardtypes)/sizeof(struct boardtype))

static struct comedi_driver driver_dyna_pci1050 = {
	.driver_name = DRV_NAME,
	.module = THIS_MODULE,
	.attach = dyna_pci1050_attach,
	.detach = dyna_pci1050_detach,
	.num_names = n_boardtypes,
	.board_name = &boardtypes[0].name,
	.offset = sizeof(struct boardtype),
};

struct dyna_pci1050_private {
	struct pci_dev *pcidev;	/*  ptr to PCI device */
	char valid;		/*  card is usable */
};

#define devpriv ((struct dyna_pci1050_private *)dev->private)
#define thisboard ((const struct boardtype *)dev->board_ptr)

/*************************** DATA FUNCTIONS ***********************************/
static int dyna_pci1050_insn_read_ai(struct comedi_device *dev,
                                struct comedi_subdevice *s,
                                struct comedi_insn *insn, unsigned int *data)
{
	printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);
	return 0;
}

static int dyna_pci1050_ai_cancel(struct comedi_device *dev,
                             struct comedi_subdevice *s)
{
	printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);
        return 0;
}

static int dyna_pci1050_ai_cmd(struct comedi_device *dev, struct comedi_subdevice *s)
{
	struct comedi_cmd *cmd = &s->async->cmd;

	return -1;
}


static int dyna_pci1050_ai_cmdtest(struct comedi_device *dev,
			      struct comedi_subdevice *s,
			      struct comedi_cmd *cmd)
{
	int err = 0;
	int tmp;
	unsigned int divisor1 = 0, divisor2 = 0;

	DPRINTK("adv_pci1710 EDBG: BGN: pci171x_ai_cmdtest(...)\n");
#ifdef PCI171X_EXTDEBUG
	pci171x_cmdtest_out(-1, cmd);
#endif
	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW;
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

	if (err) {
#ifdef PCI171X_EXTDEBUG
		pci171x_cmdtest_out(1, cmd);
#endif
		DPRINTK
		    ("adv_pci1710 EDBG: BGN: pci171x_ai_cmdtest(...) err=%d ret=1\n",
		     err);
		return 1;
	}

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if (cmd->start_src != TRIG_NOW && cmd->start_src != TRIG_EXT) {
		cmd->start_src = TRIG_NOW;
		err++;
	}

	if (cmd->scan_begin_src != TRIG_FOLLOW) {
		cmd->scan_begin_src = TRIG_FOLLOW;
		err++;
	}

	if (cmd->convert_src != TRIG_TIMER && cmd->convert_src != TRIG_EXT)
		err++;

	if (cmd->scan_end_src != TRIG_COUNT) {
		cmd->scan_end_src = TRIG_COUNT;
		err++;
	}

	if (cmd->stop_src != TRIG_NONE && cmd->stop_src != TRIG_COUNT)
		err++;

	if (err) {
#ifdef PCI171X_EXTDEBUG
		pci171x_cmdtest_out(2, cmd);
#endif
		DPRINTK
		    ("adv_pci1710 EDBG: BGN: pci171x_ai_cmdtest(...) err=%d ret=2\n",
		     err);
		return 2;
	}

	/* step 3: make sure arguments are trivially compatible */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}

	if (cmd->scan_begin_arg != 0) {
		cmd->scan_begin_arg = 0;
		err++;
	}

	if (cmd->convert_src == TRIG_TIMER) {
		if (cmd->convert_arg < thisboard->ai_ns_min) {
			cmd->convert_arg = thisboard->ai_ns_min;
			err++;
		}
	} else {		/* TRIG_FOLLOW */
		if (cmd->convert_arg != 0) {
			cmd->convert_arg = 0;
			err++;
		}
	}

	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if (cmd->stop_src == TRIG_COUNT) {
		if (!cmd->stop_arg) {
			cmd->stop_arg = 1;
			err++;
		}
	} else {		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (err) {
#ifdef PCI171X_EXTDEBUG
		pci171x_cmdtest_out(3, cmd);
#endif
		DPRINTK
		    ("adv_pci1710 EDBG: BGN: pci171x_ai_cmdtest(...) err=%d ret=3\n",
		     err);
		return 3;
	}

	/* step 4: fix up any arguments */

	if (cmd->convert_src == TRIG_TIMER) {
		tmp = cmd->convert_arg;
		//i8253_cascade_ns_to_timer(devpriv->i8254_osc_base, &divisor1,
		//			  &divisor2, &cmd->convert_arg,
		//			  cmd->flags & TRIG_ROUND_MASK);
		if (cmd->convert_arg < thisboard->ai_ns_min)
			cmd->convert_arg = thisboard->ai_ns_min;
		if (tmp != cmd->convert_arg)
			err++;
	}

	if (err) {
		printk
		    ("adv_pci1710 EDBG: BGN: pci171x_ai_cmdtest(...) err=%d ret=4\n",
		     err);
		return 4;
	}

	/* step 5: complain about special chanlist considerations */

	if (cmd->chanlist) {
		//if (!check_channel_list(dev, s, cmd->chanlist,
		//			cmd->chanlist_len))
			return 5;	/*  incorrect channels list */
	}

	printk("adv_pci1710 EDBG: BGN: pci171x_ai_cmdtest(...) ret=0\n");
	return 0;
}


/************************* CONFIG FUNCTIONS ***********************************/

//static int dyna_pci1050_reset(struct comedi_device *dev)
//{
//
//}

static int dyna_pci1050_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it)
{
	struct comedi_subdevice *s;
	int ret, subdev, n_subdevices;
	unsigned long iobase0, iobase1, iobase2, iobase3, iobase4, iobase5;
	struct pci_dev *pcidev;
	int bus, slot;
	unsigned char pci_bus, pci_slot, pci_func;
	unsigned short data16;
	int counter = 0;
	unsigned long pci_resource;
	unsigned short *addr;

	printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);

	printk(KERN_INFO "comedi: dyna_pci1050: minor number: %d", dev->minor);

	bus = it->options[0]; 
	slot = it->options[1];

	/* adding the struct dyna_pci1050_private to dev->private */
	if (alloc_private(dev, sizeof(struct dyna_pci1050_private)) < 0) {
		printk(KERN_ERR "comedi: dyna_pci1050: allocation failed!\n");
		return -ENOMEM;
	}

	pcidev = NULL;
	pcidev = pci_get_device(PCI_VENDOR_ID_DYNALOG, PCI_ANY_ID, pcidev);
	if (!pcidev) {
		printk(KERN_ERR "comedi: dyna_pci1050: error getting device!\n");
		return -ENOMEM;
	}
        printk(KERN_ERR "comedi: dyna_pci1050: bus number %d\n", pcidev->bus->number);
        printk(KERN_ERR "comedi: dyna_pci1050: device id %x\n", pcidev->device);
        printk(KERN_ERR "comedi: dyna_pci1050: vendor id %x\n", pcidev->vendor);
        printk(KERN_ERR "comedi: dyna_pci1050: slot number %d\n", PCI_SLOT(pcidev->devfn));
        printk(KERN_ERR "comedi: dyna_pci1050: function number %d\n", PCI_FUNC(pcidev->devfn));

	if (comedi_pci_enable(pcidev, DRV_NAME)) {
		printk(KERN_ERR "comedi: adv_pci1710: failed to enable PCI device and request regions!\n");
	}
	dev->board_ptr = &boardtypes[0];
 
	if (!pcidev) {
		if (bus || slot) {
			printk(KERN_ERR "comedi: adv_pci1710: failed to initialize PCI device. card at b:s %d:%d\n",
				bus, slot);
		} else {
			printk(KERN_ERR "comedi: adv_pci1710: failed to initialize PCI device\n");
		}
		return -EIO;
	}
 
	pci_bus = pcidev->bus->number;
	pci_slot = PCI_SLOT(pcidev->devfn);
	pci_func = PCI_FUNC(pcidev->devfn);

	iobase0 = pci_resource_start(pcidev, 0);
	iobase1 = pci_resource_start(pcidev, 1);
	iobase2 = pci_resource_start(pcidev, 2);
	iobase3 = pci_resource_start(pcidev, 3);
	iobase4 = pci_resource_start(pcidev, 4);
	iobase5 = pci_resource_start(pcidev, 5);

	printk(KERN_INFO "comedi: dyna_pci1050: bus %d slot %d func %d\n",
		pci_bus, pci_slot, pci_func);
	printk(KERN_INFO "comedi: dyna_pci1050: iobase 0x%4lx : 0x%4lx : 0x%4lx : 0x%4lx : 0x%4lx : 0x%4lx\n",
		iobase0, iobase1, iobase2, iobase3, iobase4, iobase5);

	dev->iobase = iobase0;
	dev->board_name = thisboard->name;
	devpriv->pcidev = pcidev;

	n_subdevices = 0;
	if (thisboard->n_aichan)
		n_subdevices++;
	if (thisboard->n_aochan)
		n_subdevices++;
	if (thisboard->n_dichan)
		n_subdevices++;
	if (thisboard->n_dochan)
		n_subdevices++;
	if (thisboard->n_counter)
		n_subdevices++;

	printk(KERN_INFO "comedi: dyna_pci1050: sub-devives %d\n", n_subdevices);
 
	ret = alloc_subdevices(dev, n_subdevices);
	if (ret < 0) {
		printk(KERN_INFO "comedi: dyna_pci1050: subdevice allocation failed!\n");
		return ret;
	}
 
	subdev = 0;
 
	if (thisboard->n_aichan) {
		s = dev->subdevices + subdev;
		dev->read_subdev = s;
		s->type = COMEDI_SUBD_AI;
		s->subdev_flags = SDF_READABLE | SDF_COMMON | SDF_GROUND | SDF_CMD_READ;
		s->n_chan = thisboard->n_aichan;
		s->maxdata = thisboard->ai_maxdata;
		s->len_chanlist = thisboard->n_aichan;
		s->range_table = thisboard->rangelist_ai;
		s->do_cmdtest = dyna_pci1050_ai_cmdtest;
		s->do_cmd = dyna_pci1050_ai_cmd;
		s->insn_read = dyna_pci1050_insn_read_ai;
		s->cancel = dyna_pci1050_ai_cancel;
		subdev++;
	}
// 
	// if (this_board->n_aochan) {
		// s = dev->subdevices + subdev;
		// s->type = COMEDI_SUBD_AO;
		// s->subdev_flags = SDF_WRITABLE | SDF_GROUND | SDF_COMMON;
		// s->n_chan = this_board->n_aochan;
		// s->maxdata = this_board->ao_maxdata;
		// s->len_chanlist = this_board->n_aochan;
		// s->range_table = this_board->rangelist_ao;
		// switch (this_board->cardtype) {
		// case TYPE_PCI1720:
			// s->insn_write = pci1720_insn_write_ao;
			// break;
		// default:
			// s->insn_write = pci171x_insn_write_ao;
			// break;
		// }
		// s->insn_read = pci171x_insn_read_ao;
		// subdev++;
	// }
// 
	// if (this_board->n_dichan) {
		// s = dev->subdevices + subdev;
		// s->type = COMEDI_SUBD_DI;
		// s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON;
		// s->n_chan = this_board->n_dichan;
		// s->maxdata = 1;
		// s->len_chanlist = this_board->n_dichan;
		// s->range_table = &range_digital;
		// s->io_bits = 0;	/* all bits input */
		// s->insn_bits = pci171x_insn_bits_di;
		// subdev++;
	// }
// 
	// if (this_board->n_dochan) {
		// s = dev->subdevices + subdev;
		// s->type = COMEDI_SUBD_DO;
		// s->subdev_flags = SDF_WRITABLE | SDF_GROUND | SDF_COMMON;
		// s->n_chan = this_board->n_dochan;
		// s->maxdata = 1;
		// s->len_chanlist = this_board->n_dochan;
		// s->range_table = &range_digital;
		// s->io_bits = (1 << this_board->n_dochan) - 1;	/* all bits output */
		// s->state = 0;
		// s->insn_bits = pci171x_insn_bits_do;
		// subdev++;
	// }
// 
	// if (this_board->n_counter) {
		// s = dev->subdevices + subdev;
		// s->type = COMEDI_SUBD_COUNTER;
		// s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
		// s->n_chan = this_board->n_counter;
		// s->len_chanlist = this_board->n_counter;
		// s->maxdata = 0xffff;
		// s->range_table = &range_unknown;
		// s->insn_read = pci171x_insn_counter_read;
		// s->insn_write = pci171x_insn_counter_write;
		// s->insn_config = pci171x_insn_counter_config;
		// subdev++;
	// }
// 
	devpriv->valid = 1;
	printk(KERN_INFO "comedi: dyna_pci1050 : finished device initilization\n");

	/* claim IO ports */

	for (counter = 0; counter < 6; counter++) {
		pci_resource = pci_resource_start(pcidev, counter);
		printk(KERN_INFO "comedi: dyna_pci1050 : pci_resource_start : %d : %016lX\n", counter, pci_resource);
		pci_resource = pci_resource_end(pcidev, counter);
		printk(KERN_INFO "comedi: dyna_pci1050 : pci_resource_end : %d : %016lX\n", counter, pci_resource);
		pci_resource = pci_resource_flags(pcidev, counter); 
		printk(KERN_INFO "comedi: dyna_pci1050 : pci_resource_flags : %d : %016lX\n", counter, pci_resource);
	}

	addr = ioremap(iobase0, 127);
	if (!addr) {
		printk(KERN_INFO "comedi: dyna_pci1050 : error mapping\n");
	} else {
		printk(KERN_INFO "comedi: dyna_pci1050 : mapping at %p\n", addr);
	}
	for (counter = 0; counter <= 1000; counter++) {
		data16 = ioread16(addr);
		//printk(KERN_INFO "in data : %02x %02x", (data16 >> 8), (data16 & 0xFF));
		printk(KERN_INFO "in data 0 : %x", data16);
		schedule();
	}
	iounmap(addr);

	for (counter = 0; counter <= 1000; counter++) {
		data16 = inw(iobase1);
		printk(KERN_INFO "in data 1 : %x", data16);
		schedule();
	}
	for (counter = 0; counter <= 1000; counter++) {
		data16 = inw(iobase2);
		printk(KERN_INFO "in data 2 : %x", data16);
		schedule();
	}
	for (counter = 0; counter <= 1000; counter++) {
		data16 = inw(iobase3);
		printk(KERN_INFO "in data 3 : %x", data16);
		schedule();
	}
	
	return 0;

}

static int dyna_pci1050_detach(struct comedi_device *dev)
{

	printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);

	if (dev->private) {
		if (devpriv->valid) {
			// pci1710_reset(dev);
		// if (dev->irq)
			// free_irq(dev->irq, dev);
		//if (devpriv->pcidev) {
		//	if (dev->iobase)
			comedi_pci_disable(devpriv->pcidev);
			pci_dev_put(devpriv->pcidev);
		}
	}

	return 0;
}

COMEDI_PCI_INITCLEANUP(driver_dyna_pci1050, dyna_pci1050_table);

