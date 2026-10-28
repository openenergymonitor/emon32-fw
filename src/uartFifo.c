#include <stddef.h>
#include <stdint.h>

#include "board_def.h"
#include "configuration.h"
#include "driver_SERCOM.h"
#include "driver_USB.h"
#include "emon32_assert.h"
#include "uartFifo.h"

void putchar_(char c);

static char uartBuffer[UART_BUFFER_TX_W] = {0};

static const uint32_t idxMsk = UART_BUFFER_TX_W - 1u;
static const uint32_t wMsk   = 1u << UART_BUFFER_TX_B;

static volatile size_t idxRd = 0u;
static volatile size_t idxWr = 0u;

bool ufPushc(const char c) {

  const bool bufferFull = ((idxWr & idxMsk) == (idxRd & idxMsk)) &&
                          ((idxWr & wMsk) != (idxRd & wMsk));

  if (bufferFull) {
    return false;
  }

  const bool uartIdle = (idxRd == idxWr);

  uartBuffer[idxWr & idxMsk] = c;
  idxWr                      = (idxWr + 1u) & (idxMsk | wMsk);

  /* If there isn't an ongoing stream, then re-expose the DRE interrupt and let
   * it drain the buffer. */
  if (uartIdle) {
    uartInterruptEnable(SERCOM_UART, SERCOM_USART_INTENSET_DRE);
  }
  return true;
}

/*! @brief Allows the printf function to print to the debug console. If the
 * USB CDC is connected, characters should be routed there.
 */
void putchar_(char c) {
  if (usbCDCIsConnected()) {
    usbCDCTxChar(c);
  }

  EMON32_ASSERT(__get_PRIMASK() == 0u);
  while (!ufPushc(c))
    ;
}

void uartPutsBlocking(const char *s) {
  EMON32_ASSERT(__get_PRIMASK() == 0u);

  while (*s) {
    while (!ufPushc(*s))
      ;
    s++;
  }
}

void SERCOM_UART_INTERACTIVE_HANDLER {

  /* Reading DATA clears RXC interrupt */
  if (uartGetcReady(SERCOM_UART_INTERACTIVE)) {
    uint8_t rx_char = uartGetc(SERCOM_UART_INTERACTIVE);
    configRxQueueChar(rx_char);
  }

  /* Writing to DATA will clear the DRE interrupt */
  if (SERCOM_UART->USART.INTFLAG.bit.DRE &&
      SERCOM_UART->USART.INTENSET.bit.DRE) {

    if (idxRd != idxWr) {
      SERCOM_UART->USART.DATA.reg = uartBuffer[idxRd & idxMsk];
      idxRd                       = (idxRd + 1u) & (idxMsk | wMsk);
    } else {
      uartInterruptDisable(SERCOM_UART, SERCOM_USART_INTENCLR_DRE);
    }
  }
}
