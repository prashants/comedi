#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <asm/io.h>

#define PCI_VENDOR_ID_DYNALOG           0x10b5
#define PCI_DEVICE_ID_DYNALOG_PCI_1050  0x1050
#define DRV_NAME                        "dyna_pci1050"

unsigned long iobase0, iobase1, iobase2, iobase3, iobase4, iobase5;
unsigned long iosize0, iosize1, iosize2, iosize3, iosize4, iosize5;


static int dyna_pci1050_read_proc(char *page, char **start, off_t
offset, int count, int *eof, void *data)
{
	u16 data16;
	unsigned int counter = 0;

	printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);

	for (counter = 0; counter <= 15; counter++) {
		mb(); smp_mb(); mdelay(10);
		outw_p(0x0030 + counter, iobase2 + 2);
		mb(); smp_mb(); mdelay(10);
		data16 = inw_p(iobase2);
		//data16 &= 0x0111;
		mb(); smp_mb(); mdelay(10);
		printk(KERN_INFO "reading data iobase2 for channel %2d : %4d %04X\n", counter, data16, data16);
	}

       return 0;
}

/************************* CONFIG FUNCTIONS ***********************************/

static int dyna_pci1050_probe(struct pci_dev *dev, const struct
pci_device_id *id)
{
       int counter = 0;
       unsigned long pci_resource;
	int ret;

       printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);

	ret = pci_enable_device(dev);
	printk(KERN_INFO "comedi: dyna_pci1050: pci_enable_device %d\n", ret);
	

       /* deivce related information */
       printk(KERN_ERR "comedi: dyna_pci1050: bus number %d\n", dev->bus->number);
       printk(KERN_ERR "comedi: dyna_pci1050: device id %x\n", dev->device);
       printk(KERN_ERR "comedi: dyna_pci1050: vendor id %x\n", dev->vendor);
       printk(KERN_ERR "comedi: dyna_pci1050: slot number %d\n", PCI_SLOT(dev->devfn));
       printk(KERN_ERR "comedi: dyna_pci1050: function number %d\n", PCI_FUNC(dev->devfn));

       for (counter = 0; counter < 6; counter++) {
               pci_resource = pci_resource_start(dev, counter);
               printk(KERN_INFO "comedi: dyna_pci1050 : pci_resource_start : %d : %08lX\n", counter, pci_resource);
               pci_resource = pci_resource_end(dev, counter);
               printk(KERN_INFO "comedi: dyna_pci1050 : pci_resource_end : %d : %08lX\n", counter, pci_resource);
               pci_resource = pci_resource_flags(dev, counter);
               printk(KERN_INFO "comedi: dyna_pci1050 : pci_resource_flags : %d : %08lX\n", counter, pci_resource);
       }

       iobase0 = pci_resource_start(dev, 0);
       iobase1 = pci_resource_start(dev, 1);
       iobase2 = pci_resource_start(dev, 2);
       iobase3 = pci_resource_start(dev, 3);
       iobase4 = pci_resource_start(dev, 4);
       iobase5 = pci_resource_start(dev, 5);

	iosize1 = pci_resource_len(dev, 1);
	iosize2 = pci_resource_len(dev, 2);
	iosize3 = pci_resource_len(dev, 3);

       printk(KERN_INFO "comedi: dyna_pci1050: iobase 0x%4lx : 0x%4lx : 0x%4lx : 0x%4lx : 0x%4lx : 0x%4lx\n",
               iobase0, iobase1, iobase2, iobase3, iobase4, iobase5);

       if (request_region(iobase1, iosize1, DRV_NAME))
               printk(KERN_INFO "comedi: dyna_pci1050: acquired iobase1 0x%4lx\n", iobase1);
       else
               printk(KERN_INFO "comedi: dyna_pci1050: failed acquiring iobase1 0x%4lx\n", iobase1);

       if (request_region(iobase2, iosize2, DRV_NAME))
               printk(KERN_INFO "comedi: dyna_pci1050: acquired iobase2 0x%4lx\n", iobase2);
       else
               printk(KERN_INFO "comedi: dyna_pci1050: failed acquiring iobase2 0x%4lx\n", iobase2);

       if (request_region(iobase3, iosize3, DRV_NAME))
               printk(KERN_INFO "comedi: dyna_pci1050: acquired iobase3 0x%4lx\n", iobase3);
       else
               printk(KERN_INFO "comedi: dyna_pci1050: failed acquiring iobase3 0x%4lx\n", iobase3);

       create_proc_read_entry("dynalog", 0, NULL, dyna_pci1050_read_proc, NULL);
       return 0;
}

static void dyna_pci1050_remove(struct pci_dev *dev)
{
       printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);

       release_region(iobase1, iosize1);
       release_region(iobase2, iosize2);
       release_region(iobase3, iosize3);

       remove_proc_entry("dynalog", NULL);
}

static struct pci_device_id ids[] = {
       { PCI_DEVICE(PCI_VENDOR_ID_DYNALOG, PCI_DEVICE_ID_DYNALOG_PCI_1050) },
       { 0, },
};

struct pci_driver dyna_pci1050_driver = {
       .name           = DRV_NAME,
       .probe          = dyna_pci1050_probe,
       .remove         = dyna_pci1050_remove,
       .id_table       = ids,
};

MODULE_DEVICE_TABLE(pci, dyna_pci1050_driver);

static int __init dyna_pci1050_init(void)
{
       printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);
       pci_register_driver(&dyna_pci1050_driver);
       return 0;
}

static void __exit dyna_pci1050_exit(void)
{
       printk(KERN_INFO "comedi: dyna_pci1050: %s\n", __func__);
       pci_unregister_driver(&dyna_pci1050_driver);
}

module_init(dyna_pci1050_init);
module_exit(dyna_pci1050_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Prashant Shah");

