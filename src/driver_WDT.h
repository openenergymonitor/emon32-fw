#pragma once

#include <stdint.h>

/*! @brief Enable the watchdog */
void wdtEnable(void);

/*! @brief Feed the watchdog to reset */
void wdtFeed(void);

/*! @brief Configure the watchdog timer in normal mode */
void wdtSetup(void);
