#pragma once

#include <stdbool.h>

/*! @brief Send a string (blocking) on UART
 *  @param [in] s : Pointer to null terminated string
 */
void uartPutsBlocking(const char *s);

/*! @brief Push a character into the UART buffer, start Tx if idle
 *  @param [in] c : character to push
 *  @return true if successful, false otherwise
 */
bool ufPushc(const char c);
