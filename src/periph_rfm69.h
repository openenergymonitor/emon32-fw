#pragma once

typedef enum RFM_Freq_ {
  RFM_FREQ_868MHz,
  RFM_FREQ_915MHz,
  RFM_FREQ_433MHz,
  RFM_FREQ_433_92MHz
} RFM_Freq_t;

typedef struct RFMOpt_ {
  RFM_Freq_t freq;
  uint8_t    group;
  uint8_t    nodeID;
  uint8_t    paLevel;
} RFMOpt_t;

typedef enum RFMSend_ {
  RFM_NO_INIT,
  RFM_TIMED_OUT,
  RFM_FAILED,
  RFM_SUCCESS,
  RFM_N_TOO_LARGE
} RFMSend_t;

/*! @brief Get a pointer to the RFM69's data buffer
 *  @return pointer to RFM69 buffer
 */
uint8_t *rfmGetBuffer(void);

/*! @brief Initialise the RFM69 module
 *  @param [in] pOpt : pointer to configuration struct
 *  @return true if successful, false otherwise
 */
bool rfmInit(const RFMOpt_t *pOpt);

/*! @brief The interrupt handler for RFM69 receive */
void rfmInterrupt(void);

/*! @brief Send data through the RFM69
 *  @param [in] n : number of bytes to be sent
 *  @param [in] retries : number of retry attempts
 *  @param [out] pRetryCount : number of retry attempts for logging
 *  @return status of the attempt to send
 */
RFMSend_t rfmSendBuffer(const uint8_t n, const uint8_t retries,
                        uint8_t *pRetryCount);

/*! @brief Sets the RFM69's address
 *  @param [in] addr : address to set the RFM69
 */
void rfmSetAddress(const uint8_t addr);

/*! @brief Set the AES key for encryption
 *  @param [in] aes : 16 character AES key. 0 disables encryption.
 */
void rfmSetAESKey(const char *aes);

/*! @brief Set the RF frequency
 *  @param [in] freq : RFM frequency
 */
void rfmSetFrequency(const RFM_Freq_t freq);

/*! @brief Set the group ID
 *  @param [in] grpID : new group ID
 */
void rfmSetGroupID(const uint8_t grpID);

/*! @brief Sets the RFM69's power level
 *  @param [in] paLevel : power level
 */
void rfmSetPowerLevel(const uint8_t paLevel);
