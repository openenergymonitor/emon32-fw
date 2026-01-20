#pragma once

#include <stddef.h>

#include "emon32_samd.h"

typedef struct DMACCfgCh {
  uint32_t ctrlb;
} DMACCfgCh_t;

/*! @brief Setup the DMAC peripheral */
void dmacSetup(void);

/*! @brief Returns a pointer to DMA descriptor for the channel.
 *  @param [in] ch : channel
 *  @return Pointer to the DmacDescriptor struct
 */
volatile DmacDescriptor *dmacGetDescriptor(uint8_t ch);

/*! @brief Set the callback when the DMA has filled the sample buffer
 *  @param [in] cb : pointer to the callback function
 */
void dmacCallbackBufferFill(void (*cb)(void));

/*! @brief Set the callback when the DMA has completed the data write
 *  @param [in] cb : pointer to the callback function
 */
void dmacCallbackUartCmpl(void (*cb)(void));

/*! @brief Disable a DMAC channel
 *  @param [in] ch : channel number
 */
void dmacChannelDisable(uint8_t ch);

/*! @brief Enable a DMAC channel
 *  @param [in] ch : channel number
 */
void dmacChannelEnable(uint8_t ch);

/*! @brief Configure a DMA channel
 *  @param [in] ch : channel to configure
 *  @param [in] pCfg : pointer to configuration details
 */
void dmacChannelConfigure(uint8_t ch, const DMACCfgCh_t *pCfg);

/*! @brief Enable DMAC channel interrupt
 *  @param [in] ch : channel to enable interrupt for
 */
void dmacEnableChannelInterrupt(uint8_t ch);

/*! @brief Disable DMAC channel interrupt
 *  @param [in] ch : channel to disable interrupt for
 */
void dmacDisableChannelInterrupt(uint8_t ch);

/*! @brief Clear DMAC channel interrupt flag
 *  @param [in] ch : channel to clear interrupt flag for
 */
void dmacClearChannelInterrupt(uint8_t ch);

/*! @brief Calculate the CRC16 (CCITT - 0x1021)
 *  @param [in] pData : pointer to data
 *  @param [in] n : number of bytes in data
 *  @return CRC16 value
 */
uint16_t calcCRC16_ccitt(const void *pSrc, size_t n);
