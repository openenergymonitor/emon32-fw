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
  RFM_NO_INIT,            /* RFM not initialised */
  RFM_NOT_IDLE,           /* Attempted to start when Tx ongoing */
  RFM_TIMED_OUT,          /* An RF feature timed out */
  RFM_FUNCTIONAL_FAILURE, /* An RFM function timed out, requires reset */
  RFM_FAILED,             /* Generic failure */
  RFM_SUCCESS,            /* Successful operation */
  RFM_N_TOO_LARGE         /* Packet too large */
} RFMSend_t;

typedef enum RFMTxState_ {
  RFM_TX_IDLE,
  RFM_TX_AWAIT_CMSA,
  RFM_TX_AWAIT_TX,
  RFM_TX_AWAIT_ACK,
  RFM_TX_ABORT
} RFMTxState_t;

/*! @brief Get a pointer to the RFM69's data buffer
 *  @return pointer to RFM69 buffer
 */
uint8_t *rfmGetBuffer(void);

/*! @brief Initialise the RFM69 module
 *  @param [in] pOpt : pointer to configuration struct
 *  @return true if successful, false otherwise
 */
bool rfmInit(const RFMOpt_t *pOpt);

/*! @brief Advance the RFM Tx state machine
 *  @return the state which the state machine is in
 */
RFMTxState_t rfmTxAdvance(void);

/*! @brief Send data through the RFM69
 *  @param [in] n : number of bytes to be sent
 *  @param [in] retries : number of retry attempts
 *  @return status of the attempt to send
 */
RFMSend_t rfmSendBuffer(const uint8_t n, const uint8_t retries);

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
