#pragma once

#include <stdbool.h>

#include "configuration.h"
#include "emon_CM.h"

/*! @brief Process the next dataset for any calibration in progress.
 *  @param [in] pECM : pointer to the ECM dataset
 *  @param [in,out] pConfig : pointer to the emon32 configuration
 */
void calibrationProcess(const ECMDataset_t *pECM, Emon32Config_t *pConfig);

/*! @brief Indicate whether to pause normal data transmission.
 *  @return true while phase calibration is in progress
 */
bool calibrationNoTransmit(void);
