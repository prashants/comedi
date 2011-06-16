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
#include <linux/usb.h>

#define USB_MINOR_BASE	192

static int ni_6008_probe(struct usb_interface *, const struct usb_device_id *);
static void ni_6008_disconnect(struct usb_interface *);
static void ni_6008_delete(struct kref *);
static int ni_6008_open(struct inode *, struct file *);
static int ni_6008_release(struct inode *, struct file *);
static ssize_t ni_6008_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t ni_6008_write(struct file *, const char __user *, size_t, loff_t *);

struct usb_ni_6008 {
	struct usb_device *udev;
	struct usb_interface *interface;
	unsigned char *bulk_in_buffer;
	size_t bulk_in_size;
	__u8 bulk_in_endpoint_addr;
	__u8 bulk_out_endpoint_addr;
	struct kref kref;
};

#define to_ni_dev(d) container_of(d, struct usb_ni_6008, kref)

static struct usb_device_id ni_6008_table[] = {
	{ USB_DEVICE(0x3923, 0x717a) },
	{}
};

MODULE_DEVICE_TABLE(usb, ni_6008_table);

static struct file_operations ni_6008_fops = {
	.owner =	THIS_MODULE,
	.read =		ni_6008_read,
	.write =	ni_6008_write,
	.open =		ni_6008_open,
	.release =	ni_6008_release,
};

static struct usb_class_driver ni_6008_class = {
	.name = "usb/ni%d",
	.fops = &ni_6008_fops,
	.minor_base = USB_MINOR_BASE,
};

static struct usb_driver ni_6008_driver = {
	.name = "ni_6008",
	.id_table = ni_6008_table,
	.probe = ni_6008_probe,
	.disconnect = ni_6008_disconnect,
};

/******************************************************************************/

static int ni_6008_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ni_6008_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t ni_6008_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t ni_6008_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *ppos)
{
	return 0;
}

/******************************************************************************/

static void ni_6008_delete(struct kref *kref)
{	
	struct usb_ni_6008 *dev = to_ni_dev(kref);

	usb_put_dev(dev->udev);
	kfree (dev->bulk_in_buffer);
	kfree (dev);
}

static int ni_6008_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_ni_6008 *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	dev = kmalloc(sizeof(struct usb_ni_6008), GFP_KERNEL);
	if (dev == NULL) {
		err("Out of memory");
		goto error;
	}
	kref_init(&dev->kref);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	iface_desc = interface->cur_altsetting;
	if (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
	}

	if (!(dev->bulk_in_endpoint_addr && dev->bulk_out_endpoint_addr)) {
		printk(KERN_ERR "could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	usb_set_intfdata(interface, dev);

	retval = usb_register_dev(interface, &ni_6008_class);
	if (retval) {
		printk(KERN_ERR "not able to get minor number for the device\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	printk(KERN_INFO "USB attached to #%d\n", interface->minor);
	return 0;

error:
	if (dev)
		kref_put(&dev->kref, ni_6008_delete);
	return retval;
}

static void ni_6008_disconnect(struct usb_interface *interface)
{
	struct ni_6008_dev *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	usb_deregister_dev(interface, &ni_6008_class);

	//kref_put(&dev->kref, ni_6008_delete);

	printk(KERN_INFO "USB #%d now disconnected", minor);
}

static int __init init_ni_6008(void)
{
	int result;
	printk(KERN_INFO "ni_6008: %s\n", __func__);

	result = usb_register(&ni_6008_driver);
	if (result)
		err("usb_register failed. error number %d\n", result);

	return result;
}

static void __exit exit_ni_6008(void)
{
	printk(KERN_INFO "ni_6008: %s\n", __func__);
	usb_deregister(&ni_6008_driver);
	return;
}

module_init(init_ni_6008);
module_exit(exit_ni_6008);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
