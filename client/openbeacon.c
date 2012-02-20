/*
 * openbeacon.c
 *
 *  Created on: Jan 16, 2012
 *      Author: jason
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */

#define OPENBEACON_CLIENT			1

#include "../firmware/modes.h"
#include "../firmware/requests.h"

#define USBDEV_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_SHARED_PRODUCT   0x05DC  /* Obdev's free shared PID */
/* Use obdev's generic shared VID/PID pair and follow the rules outlined
 * in firmware/usbdrv/USBID-License.txt.
 */

static void usage(char *name)
{
    fprintf(stderr, "\nOpenBeacon Usage:\n");
    fprintf(stderr, "  %s status\n", name);
    fprintf(stderr, "      Display the current operating parameters of the OpenBeacon\n\n");
    fprintf(stderr, "  %s modelist\n", name);
    fprintf(stderr, "      List all available mode names and their descriptions\n\n");
    fprintf(stderr, "  %s mode <modename>\n", name);
    fprintf(stderr, "      Change the operating mode to <modename>\n\n");
    fprintf(stderr, "  %s wpm <speed>\n", name);
    fprintf(stderr, "      Change the keying speed to <speed> (only in CW mode)\n\n");
    fprintf(stderr, "  %s msg1 <buffer>\n", name);
    fprintf(stderr, "      Change Message Buffer 1 to <buffer> (enclose in quotation marks)\n\n");
}


static int  usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen)
{
char    buffer[256];
int     rval, i;

    if((rval = usb_control_msg(dev, USB_ENDPOINT_IN, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, langid, buffer, sizeof(buffer), 1000)) < 0)
        return rval;
    if(buffer[1] != USB_DT_STRING)
        return 0;
    if((unsigned char)buffer[0] < rval)
        rval = (unsigned char)buffer[0];
    rval /= 2;
    /* lossy conversion to ISO Latin1 */
    for(i=1;i<rval;i++){
        if(i > buflen)  /* destination buffer overflow */
            break;
        buf[i-1] = buffer[2 * i];
        if(buffer[2 * i + 1] != 0)  /* outside of ISO Latin1 range */
            buf[i-1] = '?';
    }
    buf[i-1] = 0;
    return i-1;
}

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

static int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName)
{
struct usb_bus      *bus;
struct usb_device   *dev;
usb_dev_handle      *handle = NULL;
int                 errorCode = USB_ERROR_NOTFOUND;
static int          didUsbInit = 0;

    if(!didUsbInit){
        didUsbInit = 1;
        usb_init();
    }
    usb_find_busses();
    usb_find_devices();
    for(bus=usb_get_busses(); bus; bus=bus->next){
        for(dev=bus->devices; dev; dev=dev->next){
            if(dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product){
                char    string[256];
                int     len;
                handle = usb_open(dev); /* we need to open the device in order to query strings */
                if(!handle){
                    errorCode = USB_ERROR_ACCESS;
                    fprintf(stderr, "Warning: cannot open USB device: %s\n", usb_strerror());
                    continue;
                }
                if(vendorName == NULL && productName == NULL){  /* name does not matter */
                    break;
                }
                /* now check whether the names match: */
                len = usbGetStringAscii(handle, dev->descriptor.iManufacturer, 0x0409, string, sizeof(string));
                if(len < 0){
                    errorCode = USB_ERROR_IO;
                    fprintf(stderr, "Warning: cannot query manufacturer for device: %s\n", usb_strerror());
                }else{
                    errorCode = USB_ERROR_NOTFOUND;
                    /* fprintf(stderr, "seen device from vendor ->%s<-\n", string); */
                    if(strcmp(string, vendorName) == 0){
                        len = usbGetStringAscii(handle, dev->descriptor.iProduct, 0x0409, string, sizeof(string));
                        if(len < 0){
                            errorCode = USB_ERROR_IO;
                            fprintf(stderr, "Warning: cannot query product for device: %s\n", usb_strerror());
                        }else{
                            errorCode = USB_ERROR_NOTFOUND;
                            /* fprintf(stderr, "seen product ->%s<-\n", string); */
                            if(strcmp(string, productName) == 0)
                                break;
                        }
                    }
                }
                usb_close(handle);
                handle = NULL;
            }
        }
        if(handle)
            break;
    }
    if(handle != NULL){
        errorCode = 0;
        *device = handle;
    }
    return errorCode;
}


int main(int argc, char **argv)
{
usb_dev_handle      *handle = NULL;
unsigned char       buffer[81];
int                 nBytes;

    if(argc < 2){
        usage(argv[0]);
        exit(1);
    }
    usb_init();
    if(usbOpenDevice(&handle, USBDEV_SHARED_VENDOR, "Etherkit", USBDEV_SHARED_PRODUCT, "OpenBeacon") != 0){
        fprintf(stderr, "Could not find USB device \"OpenBeacon\" with vid=0x%x pid=0x%x\n", USBDEV_SHARED_VENDOR, USBDEV_SHARED_PRODUCT);
        exit(1);
    }
/* We have searched all devices on all busses for our USB device above. Now
 * try to open it and perform the vendor specific control operations for the
 * function requested by the user.
 */
    if(strcmp(argv[1], "status") == 0)
    {
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, CUSTOM_RQ_GET_MODE, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        printf("Mode: %s -- %s\n", mode_list[buffer[0]], mode_desc[buffer[0]]);

        if(strcmp(mode_list[buffer[0]], "cw") == 0)
        {
			nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, CUSTOM_RQ_GET_WPM, 0, 0, (char *)buffer, sizeof(buffer), 5000);
			printf("WPM: %d\n", buffer[0]);
        }

        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, CUSTOM_RQ_GET_MSG_1, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        printf("Message Buffer 1: %s\n", buffer);
    }
    else if(strcmp(argv[1], "modelist") == 0)
    {
    	int i;

    	for(i = 0; i < MODE_COUNT; i++)
    	{
    		printf("%12s -- %s\n", mode_list[i], mode_desc[i]);
    	}
    }
    else
    {
        if(argc < 3)
        {
            usage(argv[0]);
            exit(1);
        }

        if(strcmp(argv[1], "mode") == 0)
        {
        	//mode = atoi(argv[2]);
        	int i;
        	int mode = MODE_COUNT;

			for(i = 0; i < MODE_COUNT; i++)
			{
				if(strcmp(argv[2], mode_list[i]) == 0)
					mode = i;
			}

			if(mode < MODE_COUNT)
        	    nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, CUSTOM_RQ_SET_MODE, mode, 0, (char *)buffer, sizeof(buffer), 5000);
			else
			{
				fprintf(stderr, "Mode not found\n");
				exit(1);
			}
        }
        else if(strcmp(argv[1], "msg1") == 0)
        {
        	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, CUSTOM_RQ_SET_MSG_1, 0, 0, argv[2], strlen(argv[2])+1, 5000);
        	printf("Message Buffer 1: %s\n", argv[2]);
        }
        else if(strcmp(argv[1], "wpm") == 0)
        {
        	unsigned char wpm = atoi(argv[2]);

        	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, CUSTOM_RQ_SET_WPM, wpm, 0, (char *)buffer, sizeof(buffer), 5000);
        }
        else
        {
            nBytes = 0;
            usage(argv[0]);
            exit(1);
        }
        if(nBytes < 0)
            fprintf(stderr, "USB error: %s\n", usb_strerror());
    }
    usb_close(handle);
    return 0;
}
