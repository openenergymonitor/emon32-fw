#pragma once

#include <stdbool.h>

/*! @brief Set up the SAMD clock system
 *  @details The following architecture is used:
 *           OSC32 -> ClkGen 1 -> Mux 0 -> DFLL48 -> ClkGen 0 (core clock)
 *           OSC8M -> ClkGen 3 -> (peripheral clock)
 */
void clkSetup(void);

/*! @brief Set up the timer clock source
 *  @param [in] srcIO : REVISIT for v1.1 h/w
 */
void clkSetupTime(const bool srcIO);
