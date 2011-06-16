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
	printk(KERN_INFO "ni_6008: %s\n", __func__);
	return 0;
}

static int ni_6008_release(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "ni_6008: %s\n", __func__);
	return 0;
}

static ssize_t ni_6008_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	printk(KERN_INFO "ni_6008: %s\n", __func__);
	return 0;
}

static ssize_t ni_6008_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *ppos)
{
	printk(KERN_INFO "ni_6008: %s\n", __func__);
	return 0;
}

/******************************************************************************/

static void ni_6008_delete(struct kref *kref)
{	
	struct usb_ni_6008 *dev = to_ni_dev(kref);

	printk(KERN_INFO "ni_6008: %s\n", __func__);

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

	printk(KERN_INFO "ni_6008: %s\n", __func__);

	dev = kmalloc(sizeof(struct usb_ni_6008), GFP_KERNEL);
	if (dev == NULL) {
		err("Out of memory");
		goto error;
	}
	kref_init(&dev->kref);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	iface_desc = interface->cur_altsetting;
	printk(KERN_INFO "number of endpoints %d\n", iface_desc->desc.bNumEndpoints);

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		printk(KERN_INFO "endpoint at %d\n", endpoint->bEndpointAddress);

		if (!dev->bulk_in_endpoint_addr
			&& (endpoint->bEndpointAddress & USB_DIR_IN)) {
			//&& ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)) {
			/* found a bulk in endpoint */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpoint_addr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				printk(KERN_ERR "could not allocate bulk in buffer\n");
				goto error;
			}
			printk(KERN_INFO "found bulk in endpoint at %d with size %d\n", dev->bulk_in_endpoint_addr, dev->bulk_in_size);
		}
		if (!dev->bulk_out_endpoint_addr
			&& (endpoint->bEndpointAddress & USB_DIR_OUT)) {
			//&& ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)) {
			/* found a bulk out endpoint */
			dev->bulk_out_endpoint_addr = endpoint->bEndpointAddress;
			printk(KERN_ERR "found bulk out endpoint at %d\n", dev->bulk_out_endpoint_addr);
		}
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

	printk(KERN_INFO "ni_6008: %s\n", __func__);

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

MODULE_AUTHOR("Prashant Shah");
MODULE_DESCRIPTION("USB Test Driver");
MODULE_LICENSE("GPL");
