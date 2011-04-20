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

static int ni_usb6008_probe(struct usb_interface *uinterf,
			   const struct usb_device_id *id)
{
  printk(KERN_DEBUG "comedi: ni_usb6008: probe called\n");
}

static void ni_usb6008_disconnect(struct usb_interface *intf)
{
  printk(KERN_DEBUG "comedi: ni_usb6008: disconnect called\n");
}

static int ni_usb6008_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
  printk(KERN_DEBUG "comedi: ni_usb6008: attach called\n");
}

static int ni_usb6008_detach(struct comedi_device *dev)
{
  printk(KERN_DEBUG "comedi: ni_usb6008: detach called\n");
}

/* main driver struct */
static struct comedi_driver comedidriver_ni_usb6008 = {
	.driver_name = "ni_usb6008",
	.module = THIS_MODULE,
	.attach = ni_usb6008_attach,
	.detach = ni_usb6008_detach,
};

/* Table with the USB-devices */
static const struct usb_device_id ni_usb6008_table[] = {
	{USB_DEVICE(0x3923, 0x717A)},
	{}			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, ni_usb6008_table);

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
	printk(KERN_DEBUG "comedi: ni_usb6008: init called\n");
	usb_register(&usbdriver_ni_usb6008);
	comedi_driver_register(&comedidriver_ni_usb6008);
	return 0;
}

/* deregistering the comedi driver and the usb-subsystem */
static void __exit exit_ni_usb6008(void)
{
	printk(KERN_DEBUG "comedi: ni_usb6008: exit called\n");
	comedi_driver_unregister(&comedidriver_ni_usb6008);
	usb_deregister(&usbdriver_ni_usb6008);
}

module_init(init_ni_usb6008);
module_exit(exit_ni_usb6008);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
