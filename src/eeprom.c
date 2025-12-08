#include <string.h>

#ifndef HOSTED

#include "driver_DMAC.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emon32_assert.h"

#include "printf.h"

#else

#include "test_eeprom.h"

#endif /* HOSTED */

#include "eeprom.h"
#include "emon32.h"

/* Use WL_PKT_SIZE bytes for the wear limiting packet. This wastes some EEPROM
 * cells, but keeps everything page aligned for simplicity.
 */
#define WL_PKT_SIZE 64

typedef struct __attribute__((__packed__)) WLHeader_ {
  uint8_t  valid;
  uint8_t  res0;
  uint16_t crc16_ccitt;
} WLHeader_t;

_Static_assert(sizeof(WLHeader_t) == 4,
               "EEPROM wear limit header was not 4 bytes.");
_Static_assert((sizeof(Emon32Cumulative_t) + sizeof(WLHeader_t)) <= WL_PKT_SIZE,
               "EEPROM wear level packet >WL_PKT_SIZE bytes.");
_Static_assert((WL_PKT_SIZE % 16 == 0),
               "EEPROM wear level packet is not 16 byte page aligned");

typedef struct Address_ {
  unsigned int msb;
  unsigned int lsb;
} Address_t;

/* Local data for interrupt driven EEPROM write */
typedef struct wrLocal_ {
  unsigned int addr;
  unsigned int n_residual;
  uint8_t     *pData;
} wrLocal_t;

/* FUNCTIONS */
static Address_t        calcAddress(const unsigned int addrFull);
static int              nextValidByte(const uint8_t currentValid);
static eepromWLStatus_t wlFindLast(void);
static I2CM_Status_t    writeBytes(wrLocal_t *wr, unsigned int n);

/* Local values */
static int eepromSizeBytes = EEPROM_SIZE;

/* Precalculate wear limiting addresses. */
static const int wlBlkCnt  = (EEPROM_SIZE - EEPROM_WL_OFFSET) / WL_PKT_SIZE;
static const int wlBlkSize = WL_PKT_SIZE;

static int     wlCurrentValid = 0; /* Current valid byte for wear levelling */
static int     wlIdxNxtWr     = 0; /* Index of the next wear levelled write */
static int     wlData_n       = 0; /* Length of data  stored in the WL area */
static uint8_t wlData[WL_PKT_SIZE];

/*! @brief Calculates the LSB and MSB address bytes
 *  @param [in] addrFull : full address of the EEPROM
 *  @return Address_t struct with MSB and LSB
 */
static Address_t calcAddress(const unsigned int addrFull) {
  Address_t address;

  address.msb = EEPROM_BASE_ADDR | (addrFull >> 8);
  address.msb <<= 1u;

  address.lsb = addrFull & 0xFFu;

  return address;
}

/*! @brief Calculate the next "valid" byte. This is chosen for even set/clear
 *         across all bytes
 *  @param [in] currentValid : value of the current "valid" byte
 *  @return The next "valid" byte
 */
static int nextValidByte(const uint8_t currentValid) {
  uint8_t validByte = currentValid;

  /* The valid byte is calculated to have even bit 0/1 writes. */
  /* Start filling with 1s */
  if (0 == validByte) {
    validByte += 1u;
  }
  /* Start filling with 0s */
  else if (UINT8_MAX == validByte) {
    validByte <<= 1;
  }
  /* Continue filling with 1s */
  else if (validByte & 0x1u) {
    validByte <<= 1;
    validByte += 1u;
  }
  /* Continue filling with 0s */
  else {
    validByte <<= 1;
  }

  return validByte;
}

/*! @brief Find the index of the last valid write to a wear levelled block
 *  @return Status the search.
 */
static eepromWLStatus_t wlFindLast(void) {
  /* Step through from the base address in (data) sized steps. The first
   * byte that is different to the 0-th byte is the oldest block. If all
   * blocks are the same, then the 0-th index is the next to be written.
   */

  WLHeader_t       wlHeader = {0};
  eepromWLStatus_t status   = EEPROM_WL_OK;

  wlIdxNxtWr = 0;
  eepromRead(EEPROM_WL_OFFSET, &wlHeader, 4u);

  for (unsigned int idxBlk = 1u; idxBlk < wlBlkCnt; idxBlk++) {
    int        addr = EEPROM_WL_OFFSET + (idxBlk * wlBlkSize);
    WLHeader_t headerNxt;

    eepromRead(addr, &headerNxt, 4u);
    if (wlHeader.valid != headerNxt.valid) {
      wlIdxNxtWr = idxBlk;
      break;
    }
  }

  return status;
}

/*! @brief Send n bytes over I2C
 *  @param [in] wr : pointer to local address, data, and remaining bytes
 *  @param [in] n : number of bytes to send in this chunk
 *  @return Status from the write
 */
static I2CM_Status_t writeBytes(wrLocal_t *wr, unsigned int n) {
  I2CM_Status_t i2cm_s;
  Address_t     address = calcAddress(wr->addr);

  /* Setup next transaction */
  wr->addr += n;
  wr->n_residual -= n;

  /* Write to select, then lower address */
  i2cm_s = i2cActivate(SERCOM_I2CM, address.msb);
  if (I2CM_SUCCESS != i2cm_s) {
    return i2cm_s;
  }

  i2cDataWrite(SERCOM_I2CM, address.lsb);
  while (n--) {
    i2cDataWrite(SERCOM_I2CM, *wr->pData++);
  }
  i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);

  return i2cm_s;
}

unsigned int eepromDiscoverSize(void) {
  /* Read the first 16 bytes as the key value, then search on each power-of-2
   * boundary for a match. Store the found value so it only has to be done
   * once. Start at index 256, minimum possible size.
   */

  uint8_t      keys[16];
  uint8_t      trial[16];
  uint32_t     index      = 0x80; /* Shifted at least once */
  unsigned int matchbytes = 0;

  if (0 != eepromSizeBytes) {
    return eepromSizeBytes;
  }

  eepromRead(0, keys, 16);

  while (0xFFFF != matchbytes) {
    matchbytes = 0;
    index <<= 1;
    eepromRead(index, trial, 16);
    for (int i = 0; i < 16; i++) {
      if (keys[i] == trial[i]) {
        matchbytes |= (1 << i);
      }
    }
    timerDelay_ms(1);
  }

  return index;
}

void eepromDump(void) {
  /* Write out EEPROM content to debug UART 16 bytes (one page) at a time.
   * Each byte is written out as hex with a space in between
   */
  uint8_t eeprom[16];

  /* Pages */
  for (int i = 0; i < (eepromSizeBytes / 16); i++) {
    /* Bytes in page */
    eepromRead((i * 16), eeprom, 16);
    printf_("%04x: ", (i * 16));
    for (int j = 0; j < 16; j++) {
      printf_("%02x ", eeprom[j]);
    }
    printf_("\r\n");
  }
}

void eepromInitBlock(unsigned int startAddr, const unsigned int val,
                     unsigned int n) {
  /* Page aligned start, divisible by number of page bytes, <= EEPROM size */
  EMON32_ASSERT((startAddr % EEPROM_PAGE_SIZE) == 0);
  EMON32_ASSERT((n % EEPROM_PAGE_SIZE) == 0);
  EMON32_ASSERT((startAddr + n) <= EEPROM_SIZE);

  Address_t address;

  while (n) {
    address = calcAddress(startAddr);
    (void)i2cActivate(SERCOM_I2CM, address.msb);

    i2cDataWrite(SERCOM_I2CM, address.lsb);
    for (unsigned int i = 0; i < EEPROM_PAGE_SIZE; i++) {
      i2cDataWrite(SERCOM_I2CM, (uint8_t)val);
    }
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);

    timerDelay_us(EEPROM_WR_TIME);
    startAddr += EEPROM_PAGE_SIZE;
    n -= EEPROM_PAGE_SIZE;
  }
}

void eepromInitConfig(const void *pSrc, const unsigned int n) {
  /* Write the first line and wait, then loop through until all n bytes have
   * been written.
   */
  const uint8_t *p = (uint8_t *)pSrc;

  eepromWrite(0, p, n);
  while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0))
    ;
}

bool eepromRead(unsigned int addr, void *pDst, unsigned int n) {
  I2CM_Status_t i2cm_s;
  uint8_t      *pData   = pDst;
  Address_t     address = calcAddress(addr);

  /* Write select with address high and ack with another start, then send low
   * byte of address */
  i2cm_s = i2cActivate(SERCOM_I2CM, address.msb);
  if (I2CM_SUCCESS != i2cm_s) {
    return false;
  }
  i2cDataWrite(SERCOM_I2CM, address.lsb);

  /* Send select with read, and then continue to read until complete. On
   * final byte, respond with NACK */
  address.msb += 1u;

  i2cm_s = i2cActivate(SERCOM_I2CM, address.msb);
  if (I2CM_SUCCESS != i2cm_s) {
    return false;
  }

  while (n--) {
    *pData++ = i2cDataRead(SERCOM_I2CM);
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
  }
  i2cAck(SERCOM_I2CM, I2CM_NACK, I2CM_ACK_CMD_STOP);

  return true;
}

eepromWLStatus_t eepromReadWL(void *pPktRd, int *pIdx) {
  /* Check for correct indexing, find it not yet set. Read into struct from
   * correct location.
   */
  int              idxRd;
  unsigned int     addrRd;
  uint16_t         crcData;
  WLHeader_t       header;
  eepromWLStatus_t status = EEPROM_WL_OK;

  if (-1 == wlIdxNxtWr) {
    status = wlFindLast();
  }

  idxRd = wlIdxNxtWr - 1u;

  if (-1 == idxRd) {
    idxRd = (wlBlkCnt - 1);
  }
  if (pIdx) {
    *pIdx = idxRd;
  }
  addrRd = EEPROM_WL_OFFSET + (idxRd * wlBlkSize);
  eepromRead(addrRd, &header, sizeof(header));

  addrRd += 4;
  eepromRead(addrRd, pPktRd, wlData_n);
  crcData = calcCRC16_ccitt(pPktRd, sizeof(Emon32Cumulative_t));

  if (crcData != header.crc16_ccitt) {
    status = EEPROM_WL_CRC_BAD;
  }

  return status;
}

void eepromWLClear(void) {
  WLHeader_t wlHeader;

  eepromInitBlock(EEPROM_WL_OFFSET, 0, (EEPROM_SIZE - EEPROM_WL_OFFSET));

  memset(wlData, 0, WL_PKT_SIZE);
  wlHeader.valid       = 0;
  wlHeader.res0        = 0;
  wlHeader.crc16_ccitt = calcCRC16_ccitt(wlData, wlData_n);

  for (int i = 0; i < wlBlkCnt; i++) {
    int addr = EEPROM_WL_OFFSET + (i * wlBlkSize);
    eepromWrite(addr, &wlHeader, sizeof(wlHeader));
  }
}

void eepromWLReset(int len) {
  EMON32_ASSERT(len && (len <= (wlBlkSize - (int)sizeof(WLHeader_t))));

  wlIdxNxtWr = -1;
  wlData_n   = len;
}

eepromWrStatus_t eepromWrite(unsigned int addr, const void *pSrc,
                             unsigned int n) {

  /* Make byte count and address static to allow re-entrant writes */
  static wrLocal_t    wrLocal;
  static unsigned int tLastWrite_us;

  /* If all parameters are 0, then this is a continuation from ISR */
  const bool continueBlock = (0 == addr) && (0 == pSrc) && (0 == n);

  /* If no ongoing transaction:
   *   - if it is a continuation, then return complete
   *   - otherwise capture required details
   */
  if (0 == wrLocal.n_residual) {

    if (continueBlock) {
      return EEPROM_WR_COMPLETE;
    }

    wrLocal.n_residual = n;
    wrLocal.pData      = (uint8_t *)pSrc;
    wrLocal.addr       = addr;
  }
  /* If there is a pending data, and this is new, indicate BUSY */
  else {
    if (!continueBlock) {
      return EEPROM_WR_BUSY;
    }
  }

  /* Hold off write if too close to the last one. */
  while (timerMicrosDelta(tLastWrite_us) < EEPROM_WR_TIME)
    ;
  tLastWrite_us = timerMicros();

  unsigned int nBytes = 0;
  if (wrLocal.addr % EEPROM_PAGE_SIZE) {
    /* Any EEPROM page unaligned bytes */
    unsigned byteToBoundary =
        EEPROM_PAGE_SIZE - (wrLocal.addr % EEPROM_PAGE_SIZE);
    nBytes = (byteToBoundary < wrLocal.n_residual) ? byteToBoundary
                                                   : wrLocal.n_residual;
  } else if (wrLocal.n_residual > EEPROM_PAGE_SIZE) {
    /* Whole EEPROM pages */
    nBytes = EEPROM_PAGE_SIZE;
  } else {
    /* Residual bytes */
    nBytes = wrLocal.n_residual;
  }

  return (I2CM_SUCCESS == writeBytes(&wrLocal, nBytes)) ? EEPROM_WR_PEND
                                                        : EEPROM_WR_FAIL;
}

eepromWrStatus_t eepromWriteContinue(void) { return eepromWrite(0, 0, 0); }

eepromWrStatus_t eepromWriteWL(const void *pPktWr, int *pIdx) {
  /* Check for correct indexing, find if not yet set; this is indicated by
   * wlIdxNxtWr == -1. Write output to new levelled position.
   */
  int              idxWr;
  unsigned int     addrWr;
  WLHeader_t       header;
  eepromWrStatus_t wrStatus;

  header.res0        = 0;
  header.valid       = wlCurrentValid;
  header.crc16_ccitt = calcCRC16_ccitt(pPktWr, wlData_n);

  if (-1 == wlIdxNxtWr) {
    wlFindLast();
  }

  if (pIdx) {
    *pIdx = wlIdxNxtWr;
  }
  addrWr = EEPROM_WL_OFFSET + (wlIdxNxtWr * wlBlkSize);

  /* Write the header followed by the data.
   * REVISIT : may need to make this asynchronous
   */
  wrStatus = eepromWrite(addrWr, &header, sizeof(header));
  if ((wrStatus != EEPROM_WR_PEND) && (wrStatus != EEPROM_WR_COMPLETE)) {
    return wrStatus;
  }

  timerDelay_us(EEPROM_WR_TIME);

  wrStatus = eepromWrite((addrWr + sizeof(header)), pPktWr, wlData_n);

  while (EEPROM_WR_COMPLETE != wrStatus) {
    timerDelay_us(EEPROM_WR_TIME);
    wrStatus = eepromWrite(0, 0, 0);
  }

  /* Once all blocks with the same "valid" byte have been written out,
   * generate the next "valid" byte to be used and wrap.
   */
  idxWr = wlIdxNxtWr + 1u;
  if (idxWr == wlBlkCnt) {
    unsigned int validByte;
    eepromRead(addrWr, &validByte, 1u);
    wlCurrentValid = nextValidByte(validByte);
    idxWr          = 0;
  }

  wlIdxNxtWr = idxWr;
  return EEPROM_WR_WL_COMPLETE;
}
