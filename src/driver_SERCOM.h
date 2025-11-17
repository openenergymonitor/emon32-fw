#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "emon32_samd.h"

typedef enum I2CM_Ack_ { I2CM_ACK = 0u, I2CM_NACK = 1u } I2CM_Ack_t;

typedef enum I2CM_AckCmd_ {
  I2CM_ACK_CMD_NONE     = 0u,
  I2CM_ACK_CMD_START    = 1u,
  I2CM_ACK_CMD_CONTINUE = 2u,
  I2CM_ACK_CMD_STOP     = 3u
} I2CM_AckCmd_t;

typedef enum I2CM_Status_ {
  I2CM_SUCCESS,
  I2CM_ERROR,
  I2CM_TIMEOUT,
  I2CM_NOACK,
  I2CM_DISABLED
} I2CM_Status_t;

typedef struct Sercom_Cfg_ {
  Sercom *sercomExtI2C;
  Sercom *sercomExtSPI;
} SercomCfg_t;

typedef enum UART_BAUD_ {
  UART_BAUD_9600   = 9600,
  UART_BAUD_19200  = 19200,
  UART_BAUD_28800  = 28800,
  UART_BAUD_38400  = 38400,
  UART_BAUD_57600  = 57600,
  UART_BAUD_76800  = 76800,
  UART_BAUD_115200 = 115200
} UART_BAUD_t;

typedef struct UART_Cfg_ {
  Sercom     *sercom;
  UART_BAUD_t baud;
  int         apbc_mask;
  int         gclk_id;
  int         gclk_gen;
  int         pad_tx;
  int         pad_rx;
  int         port_grp;
  int         pin_tx;
  int         pin_rx;
  int         pmux;
  int         dmaChannel;
  DMACCfgCh_t dmaCfg;
} UART_Cfg_t;

/*! @brief Disable the external SPI and I2C interfaces */
void sercomExtIntfDisable(void);

/*! @brief Status of the external SPI and I2C interfaces */
bool sercomExtIntfEnabled(void);

/*! @brief configure the serial communication modules. */
void sercomSetup(void);

/*! @brief Configure a SERCOM module for UART functions.
 *  @param [in] pCfg : pointer to configuration struct
 */
void sercomSetupUART(const UART_Cfg_t *pCfg);

/*! @brief Set I2C address. If dma is 1, then a packet of len bytes is sent
 *         or received.
 *  @param [in] sercom : SERCOM instance
 *  @param [in] addr : address and RW bit
 */
I2CM_Status_t i2cActivate(Sercom *sercom, uint8_t addr);

/*! @brief Requester acknowledge command
 *  @param [in] sercom : SERCOM instance
 *  @param [in] ack : 0: ACK, 1: NACK
 *  @param [in] cmd : command
 */
void i2cAck(Sercom *sercom, I2CM_Ack_t ack, I2CM_AckCmd_t cmd);

/*! @brief Write to completer
 *  @param [in] sercom : SERCOM instance
 *  @param [in] data : data byte
 */
void i2cDataWrite(Sercom *sercom, uint8_t data);

/*! @brief Read byte from I2C completer
 *  @param [in] sercom : SERCOM instance
 *  @return read data
 */
uint8_t i2cDataRead(Sercom *sercom);

/*! @brief Select an SPI peripheral
 *  @param [in] nSS : grp+pin of chip select line
 */
void spiDeSelect(const Pin_t nSS);

/*! @brief Select an SPI peripheral
 *  @param [in] nSS : grp+pin of chip select line
 */
void spiSelect(const Pin_t nSS);

/*! @brief Send a buffer on the configured SPI channel
 *  @param [in] sercom : pointer to the SERCOM instance
 *  @param [in] pSrc : pointer to the source buffer
 *  @param [in] n : number of bytes to send
 */
void spiSendBuffer(Sercom *sercom, const void *pSrc, int n);

/*! @brief Send a byte on the configured SPI channel
 *  @param [in] sercom : pointer to the SERCOM instance
 *  @param [in] b : byte to send
 *  @return data in the SPI buffer
 */
uint8_t spiSendByte(Sercom *sercom, const uint8_t b);

/*! @brief Configure the DMA for non-blocking transactions */
void uartConfigureDMA(void);

/*! @brief Get a character from the USART data buffer. Only valid when the
 *         INTFLAG.RXC bit it set.
 *  @param [in] sercom : SERCOM instance
 */
char uartGetc(Sercom *sercom);

/*! @brief Indicate if a byte is waiting in the USART data buffer.
 *  @return true if waiting, false otherwise
 */
bool uartGetcReady(const Sercom *sercom);

/*! @brief Clear the interrupt status for the UART instance
 *  @param [in] sercom : SERCOM instance
 *  @param [in] interrupt : interrupt to clear
 */
void uartInterruptClear(Sercom *sercom, uint32_t interrupt);

/*! @brief Disable the an interrupt for the UART instance
 *  @param [in] sercom : SERCOM instance
 *  @param [in] interrupt : interrupt to disable
 */
void uartInterruptDisable(Sercom *sercom, uint32_t interrupt);

/*! @brief Enable the an interrupt for the UART instance
 *  @param [in] sercom : SERCOM instance
 *  @param [in] interrupt : interrupt to enable
 */
void uartInterruptEnable(Sercom *sercom, uint32_t interrupt);

/*! @brief Return the interrupt status for the UART instance
 *  @param [in] sercom : SERCOM instance
 *  @return interrupt status
 */
uint32_t uartInterruptStatus(const Sercom *sercom);

/*! @brief Send a single character (blocking) on UART
 *  @param [in] sercom : pointer to the SERCOM instance
 *  @param [in] c : Single character
 */
void uartPutcBlocking(Sercom *sercom, char c);

/*! @brief Send a string (blocking) on UART
 *  @param [in] sercom : pointer to the SERCOM instance
 *  @param [in] s : Pointer to null terminated string
 */
void uartPutsBlocking(Sercom *sercom, const char *s);

/*! @brief Send a string (non-blocking) on UART by DMA
 *  @param [in] dma_chan : DMA channel to send on
 *  @param [in] s : Pointer to the string
 *  @param [in] len : Length of the string (not including NULL)
 */
void uartPutsNonBlocking(unsigned int dma_chan, const char *const s,
                         uint16_t len);
