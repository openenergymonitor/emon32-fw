#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum eepromWLStatus_ {
  EEPROM_WL_OK,
  EEPROM_WL_CRC_BAD,
  EEPROM_WL_CRC_ALL_BAD,
  EEPROM_WL_BUSY /* Async write in progress, read would be inconsistent */
} eepromWLStatus_t;

typedef enum eepromWrStatus_ {
  EEPROM_WR_PEND,
  EEPROM_WR_BUSY,
  EEPROM_WR_COMPLETE,
  EEPROM_WR_FAIL,
  EEPROM_WR_WL_COMPLETE,
  EEPROM_WR_TOO_SOON /* Too soon since last write, caller should retry later */
} eepromWrStatus_t;

/*! @brief Discover the size of the EEPROM
 *  @return size (in bytes) of the EEPROM. This should be a power-of-2.
 */
uint32_t eepromDiscoverSize(void);

/*! @brief Dump all the EEPROM data out on to the debug UART */
void eepromDump(void);

/*! @brief Set all data within a block to uniform value
 *  @param [in] startAddr : start address, must be on 16 byte boundary
 *  @param [in] val : value to write
 *  @param [in] n : number of bytes to write
 */
void eepromInitBlock(uint32_t startAddr, const uint32_t val, size_t n);

/*! @brief Store values at address 0
 *  @param [in] pCfg : pointer to the data source
 *  @param [in] n : number of bytes to write
 */
void eepromInitConfig(const void *pSrc, const size_t n);

/*! @brief Read data from EEPROM
 *  @param [in] addr : base address of EEPROM read
 *  @param [out] pDst : pointer to read destination
 *  @param [in] n : number of bytes to read
 *  @return true for success, false otherwise
 */
bool eepromRead(uint32_t addr, void *pDst, size_t n);

/*! @brief Read data from EEPROM with wear leveling
 *  @param [out] pPktRd : pointer to read packet
 *  @param [out] pIdx : pointer to the value of index that has read
 *  @return status of the read
 */
eepromWLStatus_t eepromReadWL(void *pPktRd, uint32_t *pIdx);

/*! @brief Do any required setup of the EEPROM */
void eepromSetup(const uint32_t wlOffset);

/*! @brief Wipe all data from the wear limiting block and reset headers */
void eepromWLClear(void);

/*! @brief Reset the wear limited next write index
 *  @param [in] len : length (in bytes) of data in WL area
 */
void eepromWLReset(const size_t len);

/*! @brief Save data asynchronously to EEPROM
 *  @details All writes are contiguous from the base. The implementation should
 *          account for page boundaries. Call with (0, NULL, 0) to continue
 *          an ongoing staged write.
 *  @param [in] addr : base address
 *  @param [in] pSrc : pointer to data
 *  @param [in] n    : number of bytes to send
 *  @return EEPROM_WR_PEND -> data are being written
 *          EEPROM_WR_BUSY -> tried to send data while previous pending
 *          EEPROM_WR_COMPLETE -> tried to continue, but all data sent
 */
eepromWrStatus_t eepromWrite(uint32_t addr, const void *pSrc, const size_t n);

/*! @brief Continue a multi page write to EEPROM
 *  @return status of the write
 */
eepromWrStatus_t eepromWriteContinue(void);

/*! @brief Save data to EEPROM with wear leveling (blocking).
 *  @param [in] pPktWr : pointer to write packet
 *  @param [out] pIdx : pointer to the value of the index written to
 *  @return status of the EEPROM write process
 */
eepromWrStatus_t eepromWriteWL(const void *pPktWr, uint32_t *pIdx);

/*! @brief Start an asynchronous wear-leveled write operation (non-blocking).
 *  @details Uses hardware timer callback queue for precise timing
 *  @param [in] pPktWr : pointer to write packet
 *  @param [out] pIdx : pointer to the value of the index to write to (optional)
 *  @return EEPROM_WR_BUSY if another write is in progress, EEPROM_WR_PEND
 * otherwise
 *  @warning The data pointed to by pPktWr must remain valid until the write
 * completes! Use eepromWriteWLBusy() to check completion.
 */
eepromWrStatus_t eepromWriteWLAsync(const void *pPktWr, uint32_t *pIdx);

/*! @brief Check if an asynchronous wear-leveled write is in progress.
 *  @return true if write is in progress, false otherwise
 */
bool eepromWriteWLBusy(void);
