#include "../comedidev.h"
#include "comedi_pci.h"

// adlpci6802 + skel

#define PCI_VENDOR_ID_DYNALOG           0x10b5
#define PCI_DEVICE_ID_DYNALOG_PCI_1050  0x1050
#define DRV_NAME                        "dyna_pci1050"

#define PCI1050_AREAD	 	0	/* ANALOG READ */
#define PCI1050_AWRITE	 	0	/* ANALOG WRITE */
#define PCI1050_ACONTROL	2	/* ANALOG CONTROL */

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

struct boardtype {
	const char *name;
	int device_id;
	int ai_chans;
	int ai_bits;
};

static const struct boardtype boardtypes[] = {
	{
	.name = "pci1050",
	.device_id = PCI_DEVICE_ID_DYNALOG_PCI_1050,
	.ai_chans = 16,
	.ai_bits = 12,
	},
};

#define n_boardtypes (sizeof(boardtypes)/sizeof(struct boardtype))

static struct comedi_driver dyna_pci1050_driver = {
	.driver_name = DRV_NAME,
	.module = THIS_MODULE,
	.attach = dyna_pci1050_attach,
	.detach = dyna_pci1050_detach,
	.num_names = n_boardtypes,
	.board_name = &boardtypes[0].name,
	.offset = sizeof(struct boardtype),
};

struct dyna_pci1050_private {
	struct pci_dev *pci_dev;	/*  ptr to PCI device */
	char valid;			/*  card is usable */
	int data;
};

#define devpriv ((struct dyna_pci1050_private *)dev->private)
#define thisboard ((const struct boardtype *)dev->board_ptr)


/******************************************************************************/
/*********************** INITIALIZATION FUNCTIONS *****************************/
/******************************************************************************/

static int dyna_pci1050_find_device(struct comedi_device *dev, int bus, int slot)
{
	struct pci_dev *pci_dev;
	int i;

	for (pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
	     pci_dev != NULL;
	     pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_dev)) {
		if (pci_dev->vendor == PCI_VENDOR_ID_DYNALOG) {
			for (i = 0; i < n_boardtypes; i++) {
				if (boardtypes[i].device_id ==
					pci_dev->device) {
					/*
					 * was a particular bus/slot requested?
					*/
					if ((bus != 0) || (slot != 0)) {
						/*
						 * are we on the
						 * wrong bus/slot?
						*/
						if (pci_dev->bus->number
						    != bus ||
						    PCI_SLOT(pci_dev->devfn)
						    != slot) {
							continue;
						}
					}
					dev->board_ptr = boardtypes + i;
					goto found;
				}
			}
		}
	}

	printk(KERN_ERR "comedi%d: no supported board found! "
			"(req. bus/slot : %d/%d)\n",
			dev->minor, bus, slot);
	return -EIO;

found:
	printk("comedi%d: found %s (b:s:f=%d:%d:%d) , irq=%d\n",
	       dev->minor,
	       boardtypes[i].name,
	       pci_dev->bus->number,
	       PCI_SLOT(pci_dev->devfn),
	       PCI_FUNC(pci_dev->devfn), pci_dev->irq);

		devpriv->pci_dev = pci_dev;

	return 0;
}

static int
dyna_pci1050_pci_setup(struct pci_dev *pci_dev, unsigned long *io_base_ptr,
		  int dev_minor)
{
	unsigned long io_base, io_range, lcr_io_base, lcr_io_range;

	/*  Enable PCI device and request regions */
	if (comedi_pci_enable(pci_dev, PCI6208_DRIVER_NAME) < 0) {
		printk(KERN_ERR "comedi%d: Failed to enable PCI device "
			"and request regions\n",
			dev_minor);
		return -EIO;
	}
	/* Read local configuration register
	 * base address [PCI_BASE_ADDRESS #1].
	 */
	lcr_io_base = pci_resource_start(pci_dev, 1);
	lcr_io_range = pci_resource_len(pci_dev, 1);

	printk(KERN_INFO "comedi%d: local config registers at address"
			" 0x%4lx [0x%4lx]\n",
			dev_minor, lcr_io_base, lcr_io_range);

	/*  Read PCI6208 register base address [PCI_BASE_ADDRESS #2]. */
	io_base = pci_resource_start(pci_dev, 2);
	io_range = pci_resource_end(pci_dev, 2) - io_base + 1;

	printk("comedi%d: 6208 registers at address 0x%4lx [0x%4lx]\n",
	       dev_minor, io_base, io_range);

	*io_base_ptr = io_base;

	return 0;
}

static int dyna_pci1050_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it)
{
	struct comedi_subdevice *s;
	int ret, subdev, n_subdevices;
	unsigned int irq;
	unsigned long iobase;
	struct pci_dev *pcidev;
	int opt_bus, opt_slot;
	const char *errstr;
	unsigned char pci_bus, pci_slot, pci_func;
	int i;
	int board_index;

	printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);

	printk("comedi%d: dyna_pci1050: ", dev->minor);

	ret = alloc_private(dev, sizeof(struct dyna_pci1050_private));
	if (retval < 0)
		return retval;

	retval = dyna_pci1050_find_device(dev, it->options[0], it->options[1]);
	if (retval < 0)
		return retval;

	retval = pci6208_pci_setup(devpriv->pci_dev, &io_base, dev->minor);
	if (retval < 0)
		return retval;

	dev->iobase = io_base;
	dev->board_name = thisboard->name;

	/*
	 * Allocate the subdevice structures.  alloc_subdevice() is a
	 * convenient macro defined in comedidev.h.
	 */
	if (alloc_subdevices(dev, 1) < 0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog output subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE;	/* anything else to add here?? */
	s->n_chan = thisboard->ao_chans;
	s->maxdata = 0xffff;	/* 16-bit DAC */
	s->range_table = &range_bipolar10;	/* this needs to be checked. */
	s->insn_write = pci6208_ao_winsn;
	s->insn_read = pci6208_ao_rinsn;

	printk(KERN_INFO "attached\n");

	return 1;
}

static int dyna_pci1050_detach(struct comedi_device *dev)
{
	if (dev->private) {
		if (devpriv->valid)
			dyna_pci1050_reset(dev);
		if (devpriv->pcidev) {
			if (dev->iobase)
				comedi_pci_disable(devpriv->pcidev);

			pci_dev_put(devpriv->pcidev);
		}
	}

	return 0;
}

COMEDI_PCI_INITCLEANUP(dyna_pci1050_driver, dyna_pci1050_pci_table);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Prashant Shah");

