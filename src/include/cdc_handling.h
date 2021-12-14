#ifndef USB_CDC_H
#define USB_CDC_H

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_BUFFER 64
#define CMD_LEN    32

void createCDCHandler();
void reportUSB_CDC();

#ifdef __cplusplus
}
#endif

#endif /*USB_CDC_H*/
