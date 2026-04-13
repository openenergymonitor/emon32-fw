#pragma once

#include <stdbool.h>
#include <stdint.h>

/*! @brief Get the gain correction value
 *  @return gain correction value
 */
int32_t adcCorrectionGain(void);

/*! @brief Indicates if the correction values are valid
 *  @return true if valid, false otherwise
 */
bool adcCorrectionValid(void);

/*! @brief Starts the DMAC transfer from the ADC */
void adcDMACStart(void);

/*! @brief Stop the DMAC transfer from the ADC */
void adcDMACStop(void);

/*! @brief Configure the ADC for the board */
void adcSetup(void);
