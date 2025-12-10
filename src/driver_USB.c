#include "board_def.h"
#include "emon32_samd.h"

#include "configuration.h"
#include "driver_PORT.h"
#include "driver_USB.h"
#include "tusb_config.h"

#include <string.h>

bool usbCDCIsConnected(void) { return tud_cdc_connected(); }

void usbCDCPutsBlocking(const char *s) {
  /* Write string to CDC buffer without flushing. When buffer is full, wait for
   * space to become available. Flushing is handled by usbCDCTask() and explicit
   * usbCDCTxFlush() calls to maintain line atomicity across multiple
   * printf/serialPuts calls.
   */
  size_t len    = strlen(s);
  size_t offset = 0;

  while (offset < len) {
    size_t available = tud_cdc_write_available();

    if (available == 0) {
      /* Buffer full - service USB to allow it to drain */
      tud_task();
      continue;
    }

    /* Write as much as fits in available space */
    size_t to_send = (len - offset) > available ? available : (len - offset);
    tud_cdc_write(s + offset, to_send);
    offset += to_send;
  }
  /* No flush here - let caller control when to flush */
}

bool usbCDCRxAvailable(void) { return tud_cdc_available(); }

uint8_t usbCDCRxGetChar(void) {
  if (tud_cdc_available()) {
    return tud_cdc_read_char();
  }

  return 0;
}

void usbCDCTask(void) {
  if (!usbCDCIsConnected()) {
    return;
  }

  /* Flush write buffer and read any available characters */
  tud_cdc_write_flush();
  int nrx = tud_cdc_available();
  if (nrx) {
    for (int i = 0; i < nrx; i++) {
      int ch = tud_cdc_read_char();
      if (-1 != ch) {
        /* Check if we're waiting for a confirmation (bootloader, zero, etc.) */
        if (!configHandleConfirmation((uint8_t)ch)) {
          /* Normal command processing */
          configCmdChar(((uint8_t)ch));
          usbCDCTxChar((uint8_t)ch);
        }
      }
    }
  }
}

void usbCDCTxChar(uint8_t c) {
  /* Wait for buffer space if full */
  while (tud_cdc_write_available() == 0) {
    /* Service USB to drain buffer */
    tud_task();
  }
  tud_cdc_write_char(c);
  /* No flush here - let caller control flushing */
}

void usbCDCTxFlush(void) { tud_cdc_write_flush(); }

void usbSetup(void) {
  /* Clocking:
   *  - AHB is enabled by default (16.8.7)
   *  - APB is enabled by default (16.8.9)
   *  - Use the 48 MHz PLL for required accuracy
   */
  PM->APBBMASK.reg |= PM_APBBMASK_USB;
  PM->AHBMASK.reg |= PM_AHBMASK_USB;

  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID_USB | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;

  /* Configure ports (Table 7-1) */
  portPinDir(GRP_USB_DM, PIN_USB_DM, PIN_DIR_OUT);
  portPinDir(GRP_USB_DP, PIN_USB_DP, PIN_DIR_OUT);
  portPinMux(GRP_USB_DM, PIN_USB_DM, PMUX_USB);
  portPinMux(GRP_USB_DP, PIN_USB_DP, PMUX_USB);

  NVIC_SetPriority(USB_IRQn, 1);
  NVIC_EnableIRQ(USB_IRQn);

  tud_init(0);
}

/* Used by TinyUSB for the board's serial number. Use the SAMD's full 128 bit
 * unique code for this. Note that the maximum length is typically 16, so fix at
 * that value. */
size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void)max_len;
  uint32_t *pId = (uint32_t *)id;
  for (int i = 0; i < 4; i++) {
    *pId++ = getUniqueID(i);
  }
  return 16;
}

void irq_handler_usb(void) { tud_int_handler(0); }
