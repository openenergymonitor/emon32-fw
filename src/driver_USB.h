#pragma once

#include "tusb.h"
#include <stdbool.h>

/*! @brief Check if the CDC serial is connected
 *  @return true if connected, false otherwise
 */
bool usbCDCIsConnected(void);

/*! @brief Write a string out to the CDC serial */
void usbCDCPutsBlocking(const char *s);

/*! @brief Indicate if any characters are in the CDC buffer
 *  @return true if any characters, false otherwise
 */
bool usbCDCRxAvailable(void);

/*! @brief Get a character from the CDC buffer
 *  @return the first character in the FIFO
 */
uint8_t usbCDCRxGetChar(void);

/*! @brief Do any required CDC serial tasks */
void usbCDCTask(void);

/*! @brief Add character to Tx buffer */
void usbCDCTxChar(uint8_t c);

/*! @brief Flush the Tx buffer */
void usbCDCTxFlush(void);

/*! @brief Set up USB hardware and tinyUSB stack */
void usbSetup(void);
