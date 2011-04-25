#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/usb.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("Prashant Shah <pshah.mumbai@gmail.com>");
MODULE_DESCRIPTION("Simple USB Driver");
MODULE_VERSION("1");
MODULE_LICENSE("GPL");

typedef struct {
	struct usb_device *usbdev;
	struct usb_interface *interface;
	struct urb *ctrl_urb;
	struct usb_ctrlrequest ctrl_req;
	unsigned char *bulk_in_buf;
	size_t bulk_in_len;
	__u8 bulk_in_addr;
	__u8 bulk_out_addr;
} simpleusb_t;

#define SIMPLEUSB_MINOR_BASE 0xAB

static struct file_operations simpleusb_fops = {
	.owner = THIS_MODULE,
	// .read = simpleusb_read,
	// .write = simpleusb_write,
	// .ioctl = simpleusb_ioctl,
	// .open = simpleusb_open,
	// .release = simpleusb_release,
};

static struct usb_class_driver simpleusb_class = {
	.name = "simpleusb",
	.fops = &simpleusb_fops,
	.minor_base = SIMPLEUSB_MINOR_BASE,
};

static int simpleusb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	simpleusb_t *simpleusb_device;
	int retval = -ENOMEM;
	int i = 0;

	printk(KERN_INFO "simpleusb: %s\n", __func__);
	simpleusb_device = kzalloc(sizeof(simpleusb_t), GFP_KERNEL);
	simpleusb_device->usbdev = usb_get_dev(interface_to_usbdev(interface));
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;
		if (!simpleusb_device->bulk_in_addr && usb_endpoint_is_bulk_in(endpoint)) {
			/* Bulk IN endpoint */
			simpleusb_device->bulk_in_len = le16_to_cpu(endpoint->wMaxPacketSize);
			simpleusb_device->bulk_in_addr = endpoint->bEndpointAddress;
			simpleusb_device->bulk_in_buf = kmalloc(simpleusb_device->bulk_in_len, GFP_KERNEL);
		}
		if (!simpleusb_device->bulk_out_addr && usb_endpoint_is_bulk_out(endpoint)) {
			/* Bulk OUT endpoint */
			simpleusb_device->bulk_out_addr = endpoint->bEndpointAddress;
		}

	}
	if (!(simpleusb_device->bulk_in_addr && simpleusb_device->bulk_out_addr)) {
		return retval;
	}

	/* attach device specific structure to this interface */
	usb_set_intfdata(interface, simpleusb_device);

	/* register the device */
	retval = usb_register_dev(interface, &simpleusb_class);
	if (retval) {
		usb_set_intfdata(interface, NULL);
		return retval;
	}
	printk(KERN_INFO "simpleusb: device now attached to /dev/simpleusb\n");
	return 0;
}

static void simpleusb_disconnect(struct usb_interface *interface)
{
	struct simpleusb_t *simpleusb_device;
	printk(KERN_INFO "simpleusb: %s\n", __func__);

	simpleusb_device = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
	usb_deregister_dev(interface, &simpleusb_class);
	//simpleusb_device->interface = NULL;
}

/* table of devices that work with this driver */
static struct usb_device_id simpleusb_table [] = {
	{ USB_DEVICE(0x3923, 0x717A) },
	//{ USB_DEVICE(0x06E6, 0xC200) }, // Tiger Direct
	//{ USB_DEVICE(0x0951, 0x1607) }, // KINGSTON USB PEN DRIVE 0951:1607
	{ }	/* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, simpleusb_table);

static struct usb_driver simpleusb_driver = {
	.name = "simpleusb",
	.probe = simpleusb_probe,
	.disconnect = simpleusb_disconnect,
	.id_table = simpleusb_table,
};

static int __init simpleusb_init(void)
{
	printk(KERN_INFO "simpleusb: %s\n", __func__);
	usb_register(&simpleusb_driver);
	return 0;
}

static void __exit simpleusb_exit(void)
{
	printk(KERN_INFO "simpleusb: %s\n", __func__);
	usb_deregister(&simpleusb_driver);
}

module_init(simpleusb_init);
module_exit(simpleusb_exit);

