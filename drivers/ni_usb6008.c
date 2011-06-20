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
//static void ni_6008_delete(struct kref *);
static int ni_6008_open(struct inode *, struct file *);
static int ni_6008_release(struct inode *, struct file *);
static ssize_t ni_6008_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t ni_6008_write(struct file *, const char __user *, size_t, loff_t *);

struct usb_ni_6008 {
	struct usb_device *udev;
	struct usb_interface *interface;

	unsigned char *bulk1_buffer;
	size_t bulk1_size;
	__u8 bulk1_endpoint_addr;

	unsigned char *bulk2_buffer;
	size_t bulk2_size;
	__u8 bulk2_endpoint_addr;

	unsigned char *bulk3_buffer;
	size_t bulk3_size;
	__u8 bulk3_endpoint_addr;

	unsigned char *bulk4_buffer;
	size_t bulk4_size;
	__u8 bulk4_endpoint_addr;

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
	struct usb_ni_6008 *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	printk(KERN_INFO "ni_6008: %s\n", __func__);

	subminor = iminor(inode);

	interface = usb_find_interface(&ni_6008_driver, subminor);
	if (!interface) {
		err ("%s - error, can't find device for minor %d", __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	/* increment our usage count for the device */
	//kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

exit:
	return retval;
}

static int ni_6008_release(struct inode *inode, struct file *file)
{
	struct usb_ni_6008 *dev;

	printk(KERN_INFO "ni_6008: %s\n", __func__);

	dev = (struct usb_ni_6008 *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* decrement the count on our device */
	//kref_put(&dev->kref, skel_delete);
	return 0;
}

static ssize_t ni_6008_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	struct usb_ni_6008 *dev;
	int retval = 0;

	printk(KERN_INFO "ni_6008: %s\n", __func__);

	dev = (struct usb_ni_6008 *)file->private_data;
	
	/* do a blocking bulk read to get data from the device */
	retval = usb_bulk_msg(dev->udev,
			      usb_rcvbulkpipe(dev->udev, dev->bulk1_endpoint_addr),
			      dev->bulk1_buffer,
			      min(dev->bulk1_size, count),
			      &count, HZ*10);

	/* if the read was successful, copy the data to userspace */
	if (!retval) {
		if (copy_to_user(buffer, dev->bulk1_buffer, count))
			retval = -EFAULT;
		else
			retval = count;
	}

	return retval;
}

static ssize_t ni_6008_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *ppos)
{
	printk(KERN_INFO "ni_6008: %s\n", __func__);
	return 0;
}

/******************************************************************************/

/*
static void ni_6008_delete(struct kref *kref)
{	
	struct usb_ni_6008 *dev = to_ni_dev(kref);

	printk(KERN_INFO "ni_6008: %s\n", __func__);

	usb_put_dev(dev->udev);
	kfree (dev->bulk1_buffer);
	kfree (dev->bulk2_buffer);
	kfree (dev->bulk3_buffer);
	kfree (dev->bulk4_buffer);
	kfree (dev);
}
*/

static int ni_6008_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_ni_6008 *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
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

	endpoint = &iface_desc->endpoint[0].desc;
	buffer_size = endpoint->wMaxPacketSize;
	dev->bulk1_size = buffer_size;
	dev->bulk1_endpoint_addr = endpoint->bEndpointAddress;
	dev->bulk1_buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (!dev->bulk1_buffer) {
		printk(KERN_ERR "could not allocate bulk in buffer\n");
		goto error;
	}
	printk(KERN_INFO "found bulk endpoint at %d with size %d\n", dev->bulk1_endpoint_addr, dev->bulk1_size);

	endpoint = &iface_desc->endpoint[1].desc;
	buffer_size = endpoint->wMaxPacketSize;
	dev->bulk2_size = buffer_size;
	dev->bulk2_endpoint_addr = endpoint->bEndpointAddress;
	dev->bulk2_buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (!dev->bulk2_buffer) {
		printk(KERN_ERR "could not allocate bulk in buffer\n");
		goto error;
	}
	printk(KERN_INFO "found bulk endpoint at %d with size %d\n", dev->bulk2_endpoint_addr, dev->bulk2_size);

	endpoint = &iface_desc->endpoint[2].desc;
	buffer_size = endpoint->wMaxPacketSize;
	dev->bulk3_size = buffer_size;
	dev->bulk3_endpoint_addr = endpoint->bEndpointAddress;
	dev->bulk3_buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (!dev->bulk3_buffer) {
		printk(KERN_ERR "could not allocate bulk in buffer\n");
		goto error;
	}
	printk(KERN_INFO "found bulk endpoint at %d with size %d\n", dev->bulk3_endpoint_addr, dev->bulk3_size);

	endpoint = &iface_desc->endpoint[3].desc;
	buffer_size = endpoint->wMaxPacketSize;
	dev->bulk4_size = buffer_size;
	dev->bulk4_endpoint_addr = endpoint->bEndpointAddress;
	dev->bulk4_buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (!dev->bulk4_buffer) {
		printk(KERN_ERR "could not allocate bulk in buffer\n");
		goto error;
	}
	printk(KERN_INFO "found bulk endpoint at %d with size %d\n", dev->bulk4_endpoint_addr, dev->bulk4_size);

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
	//if (dev)
		//kref_get(&dev->kref, ni_6008_delete);
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
