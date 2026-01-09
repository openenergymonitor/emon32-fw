#include <limits.h>
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

#include "driver_TIME.h"
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
  uint32_t msb;
  uint32_t lsb;
} Address_t;

/* Local data for interrupt driven EEPROM write */
typedef struct wrLocal_ {
  uint32_t addr;
  uint32_t n_residual;
  uint8_t *pData;
} wrLocal_t;

/* Asynchronous wear-leveled write state machine */
typedef enum wlAsyncState_ {
  WL_ASYNC_IDLE,
  WL_ASYNC_WRITING_HEADER,
  WL_ASYNC_WAITING_HEADER,
  WL_ASYNC_START_DATA_WRITE, /* Initiate data write (may need to wait for
                                timing) */
  WL_ASYNC_WRITING_DATA,
  WL_ASYNC_WAITING_DATA
} wlAsyncState_t;

typedef struct wlAsyncCtx_ {
  wlAsyncState_t   state;
  uint32_t         addrWr;
  WLHeader_t       header;
  const uint8_t   *pData;
  size_t           dataLen;
  uint32_t         idx;
  eepromWrStatus_t lastStatus;
  int32_t          busyRetries; /* Counter for BUSY retries */
} wlAsyncCtx_t;

#define MAX_BUSY_RETRIES 10

/* FUNCTIONS */
static Address_t        calcAddress(const uint32_t addrFull);
static int32_t          nextValidByte(const uint8_t currentValid);
static eepromWLStatus_t wlFindLast(void);
static I2CM_Status_t    writeBytes(wrLocal_t *wr, uint32_t n);

/* Local values */
static int32_t eepromSizeBytes = EEPROM_SIZE;

/* Precalculate wear limiting addresses. */
static const uint32_t wlBlkCnt = (EEPROM_SIZE - EEPROM_WL_OFFSET) / WL_PKT_SIZE;
static const int32_t  wlBlkSize = WL_PKT_SIZE;

static int32_t  wlCurrentValid = 0; /* Current valid byte for wear levelling */
static uint32_t wlIdxNxtWr     = 0; /* Index of the next wear levelled write */
static size_t   wlData_n       = 0; /* Length of data  stored in the WL area */
static uint8_t  wlData[WL_PKT_SIZE];

/* Async write context. Accessed from main loop only (callbacks run in main, not
 * ISR). Marked volatile for defensive programming in case future refactoring
 * changes the execution context.
 */
static volatile wlAsyncCtx_t wlAsyncCtx = {.state = WL_ASYNC_IDLE};

/*! @brief Calculates the LSB and MSB address bytes
 *  @param [in] addrFull : full address of the EEPROM
 *  @return Address_t struct with MSB and LSB
 */
static Address_t calcAddress(const uint32_t addrFull) {
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
static int32_t nextValidByte(const uint8_t currentValid) {
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

  for (uint32_t idxBlk = 1u; idxBlk < wlBlkCnt; idxBlk++) {
    int32_t    addr = EEPROM_WL_OFFSET + (idxBlk * wlBlkSize);
    WLHeader_t headerNxt;

    eepromRead(addr, &headerNxt, 4u);
    if (wlHeader.valid != headerNxt.valid) {
      wlIdxNxtWr = idxBlk;
      break;
    }
  }

  /* Set wlCurrentValid based on what was found. If wlIdxNxtWr is still 0,
   * all blocks have the same valid byte - advance to next valid byte so new
   * writes are distinguishable. Otherwise, use the valid byte from the most
   * recently written block (at index wlIdxNxtWr - 1).
   */
  if (wlIdxNxtWr == 0) {
    /* All blocks identical - advance valid byte for next write */
    wlCurrentValid = nextValidByte(wlHeader.valid);
  } else {
    /* Use valid byte from last written block. If lastWrittenIdx is 0, we
     * already have block 0's header in wlHeader from the initial read.
     */
    int32_t lastWrittenIdx = wlIdxNxtWr - 1;
    if (lastWrittenIdx != 0) {
      int32_t lastWrittenAddr = EEPROM_WL_OFFSET + (lastWrittenIdx * wlBlkSize);
      eepromRead(lastWrittenAddr, &wlHeader, 4u);
    }
    wlCurrentValid = wlHeader.valid;
  }

  return status;
}

/*! @brief Send n bytes over I2C
 *  @param [in] wr : pointer to local address, data, and remaining bytes
 *  @param [in] n : number of bytes to send in this chunk
 *  @return Status from the write
 */
static I2CM_Status_t writeBytes(wrLocal_t *wr, uint32_t n) {
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

  i2cm_s = i2cDataWrite(SERCOM_I2CM, address.lsb);
  if (I2CM_SUCCESS != i2cm_s) {
    return i2cm_s;
  }

  while (n--) {
    i2cm_s = i2cDataWrite(SERCOM_I2CM, *wr->pData++);
    if (I2CM_SUCCESS != i2cm_s) {
      return i2cm_s;
    }
  }
  i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);

  return i2cm_s;
}

uint32_t eepromDiscoverSize(void) {
  /* Read the first 16 bytes as the key value, then search on each power-of-2
   * boundary for a match. Store the found value so it only has to be done
   * once. Start at index 256, minimum possible size.
   */

  uint8_t  keys[16];
  uint8_t  trial[16];
  uint32_t index      = 0x80; /* Shifted at least once */
  uint32_t matchbytes = 0;

  if (0 != eepromSizeBytes) {
    return eepromSizeBytes;
  }

  eepromRead(0, keys, 16);

  while (0xFFFF != matchbytes) {
    matchbytes = 0;
    index <<= 1;
    eepromRead(index, trial, 16);
    for (int32_t i = 0; i < 16; i++) {
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
  for (int32_t i = 0; i < (eepromSizeBytes / 16); i++) {
    /* Bytes in page */
    eepromRead((i * 16), eeprom, 16);
    printf_("%04x: ", (i * 16));
    for (int32_t j = 0; j < 16; j++) {
      printf_("%02x ", eeprom[j]);
    }
    printf_("\r\n");
  }
}

void eepromInitBlock(uint32_t startAddr, const uint32_t val, size_t n) {
  /* Page aligned start, divisible by number of page bytes, <= EEPROM size */
  EMON32_ASSERT((startAddr % EEPROM_PAGE_SIZE) == 0);
  EMON32_ASSERT((n % EEPROM_PAGE_SIZE) == 0);
  EMON32_ASSERT((startAddr + n) <= EEPROM_SIZE);

  Address_t     address;
  I2CM_Status_t i2cm_s;

  while (n) {
    address = calcAddress(startAddr);
    i2cm_s  = i2cActivate(SERCOM_I2CM, address.msb);
    if (I2CM_SUCCESS != i2cm_s) {
      return;
    }

    i2cm_s = i2cDataWrite(SERCOM_I2CM, address.lsb);
    if (I2CM_SUCCESS != i2cm_s) {
      return;
    }

    for (uint32_t i = 0; i < EEPROM_PAGE_SIZE; i++) {
      i2cm_s = i2cDataWrite(SERCOM_I2CM, (uint8_t)val);
      if (I2CM_SUCCESS != i2cm_s) {
        return;
      }
    }
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);

    timerDelay_us(EEPROM_WR_TIME);
    startAddr += EEPROM_PAGE_SIZE;
    n -= EEPROM_PAGE_SIZE;
  }
}

void eepromInitConfig(const void *pSrc, const size_t n) {
  /* Write the first line and wait, then loop through until all n bytes have
   * been written.
   */
  const uint8_t   *p = (uint8_t *)pSrc;
  eepromWrStatus_t wrStatus;

  wrStatus = eepromWrite(0, p, n);
  if ((wrStatus != EEPROM_WR_PEND) && (wrStatus != EEPROM_WR_COMPLETE)) {
    return; /* Write failed immediately */
  }

  /* Wait for write to complete with proper delays (like eepromWriteWL) */
  while (EEPROM_WR_COMPLETE != wrStatus) {
    timerDelay_us(EEPROM_WR_TIME);
    wrStatus = eepromWrite(0, 0, 0);
  }

  /* Wait for final EEPROM internal write cycle to complete (per datasheet: 5ms
   * max) */
  timerDelay_us(EEPROM_WR_TIME);
}

bool eepromRead(uint32_t addr, void *pDst, size_t n) {
  I2CM_Status_t i2cm_s;
  uint8_t      *pData   = pDst;
  Address_t     address = calcAddress(addr);

  /* Write select with address high and ack with another start, then send low
   * byte of address */
  i2cm_s = i2cActivate(SERCOM_I2CM, address.msb);
  if (I2CM_SUCCESS != i2cm_s) {
    return false;
  }

  i2cm_s = i2cDataWrite(SERCOM_I2CM, address.lsb);
  if (I2CM_SUCCESS != i2cm_s) {
    return false;
  }

  /* Send select with read, and then continue to read until complete. On
   * final byte, respond with NACK */
  address.msb += 1u;

  i2cm_s = i2cActivate(SERCOM_I2CM, address.msb);
  if (I2CM_SUCCESS != i2cm_s) {
    return false;
  }

  while (n) {
    i2cm_s = i2cDataRead(SERCOM_I2CM, pData++);
    if (I2CM_SUCCESS != i2cm_s) {
      return false;
    }
    n--;
    if (n == 0) {
      /* Last byte - send NACK and STOP */
      i2cAck(SERCOM_I2CM, I2CM_NACK, I2CM_ACK_CMD_STOP);
    } else {
      /* More bytes to read - send ACK and continue */
      i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
    }
  }

  return true;
}

eepromWLStatus_t eepromReadWL(void *pPktRd, uint32_t *pIdx) {
  /* Check for correct indexing, find it not yet set. Read into struct from
   * correct location.
   */
  uint32_t         idxRd;
  uint32_t         addrRd;
  uint16_t         crcData;
  WLHeader_t       header;
  eepromWLStatus_t status = EEPROM_WL_OK;

  /* If an async write is in progress, the data may be inconsistent */
  if (eepromWriteWLBusy()) {
    if (pIdx) {
      *pIdx = wlIdxNxtWr;
    }
    return EEPROM_WL_BUSY;
  }

  if (UINT32_MAX == wlIdxNxtWr) {
    status = wlFindLast();
  }

  idxRd = wlIdxNxtWr - 1u;

  if (UINT32_MAX == idxRd) {
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

  /* Retry once on CRC mismatch - I2C reads can occasionally be corrupted */
  if (crcData != header.crc16_ccitt) {
    printf_("EEPROM CRC retry: hdr=0x%04X data=0x%04X\r\n", header.crc16_ccitt,
            crcData);
    eepromRead(addrRd, pPktRd, wlData_n);
    crcData = calcCRC16_ccitt(pPktRd, sizeof(Emon32Cumulative_t));
  }

  if (crcData != header.crc16_ccitt) {
    printf_("EEPROM CRC FAIL: hdr=0x%04X data=0x%04X valid=0x%02X\r\n",
            header.crc16_ccitt, crcData, header.valid);
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

  for (int32_t i = 0; i < wlBlkCnt; i++) {
    int32_t addr = EEPROM_WL_OFFSET + (i * wlBlkSize);
    eepromWrite(addr, &wlHeader, sizeof(wlHeader));
  }
}

void eepromWLReset(const size_t len) {
  EMON32_ASSERT(len && (len <= (wlBlkSize - (int32_t)sizeof(WLHeader_t))));

  wlIdxNxtWr = UINT32_MAX;
  wlData_n   = len;
}

eepromWrStatus_t eepromWrite(uint32_t addr, const void *pSrc, const size_t n) {

  /* Make byte count and address static to allow re-entrant writes */
  static wrLocal_t wrLocal;
  static uint32_t  tLastWrite_us;

  /* Special reset call: addr=UINT_MAX clears residual state */
  if (addr == UINT_MAX) {
    wrLocal.n_residual = 0;
    return EEPROM_WR_COMPLETE;
  }

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

    /* Check if enough time has passed since last write. If not, return TOO_SOON
     * instead of busy-waiting. This allows async callers to reschedule.
     */
    if (timerMicrosDelta(tLastWrite_us) < EEPROM_WR_TIME) {
      return EEPROM_WR_TOO_SOON;
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

  /* Update timestamp when starting actual I2C transaction */
  tLastWrite_us = timerMicros();

  uint32_t nBytes = 0;
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

/*! @brief Perform I2C bus recovery for internal EEPROM bus */
static void eepromBusRecovery(void) {
  printf_("I2C RECOVERY\r\n");
  /* Reset software state (clear n_residual) */
  eepromWrite(UINT_MAX, 0, 0);
  /* Recover I2C bus hardware */
  i2cBusRecovery(SERCOM_I2CM, GRP_SERCOM_I2C_INT, PIN_I2C_INT_SDA,
                 PIN_I2C_INT_SCL, PMUX_I2CM_INT);
}

/* Asynchronous wear-leveled write implementation using timer callbacks */

/*! @brief State machine callback for async EEPROM write */
static void eepromWLAsyncCallback(void) {
  eepromWrStatus_t status;

  switch (wlAsyncCtx.state) {
  case WL_ASYNC_IDLE:
    /* Should not be called in idle state */
    return;

  case WL_ASYNC_WRITING_HEADER:
    /* Initiate header write - retry on FAIL with bus recovery */
    status = eepromWrite(wlAsyncCtx.addrWr, (const void *)&wlAsyncCtx.header,
                         sizeof(wlAsyncCtx.header));
    if (status == EEPROM_WR_FAIL) {
      /* I2C failed - perform bus recovery and retry once */
      eepromBusRecovery();
      status = eepromWrite(wlAsyncCtx.addrWr, (const void *)&wlAsyncCtx.header,
                           sizeof(wlAsyncCtx.header));
    }
    if (status == EEPROM_WR_PEND) {
      wlAsyncCtx.busyRetries = 0; /* Reset on success */
      wlAsyncCtx.state       = WL_ASYNC_WAITING_HEADER;
      /* Schedule next callback after EEPROM_WR_TIME */
      if (!timerScheduleCallback(eepromWLAsyncCallback, EEPROM_WR_TIME)) {
        printf_("EEPROM CB FAIL!\r\n");
        /* Drain the lower-level write to prevent stuck state */
        while (eepromWrite(0, 0, 0) != EEPROM_WR_COMPLETE) {
          timerDelay_us(EEPROM_WR_TIME);
        }
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
    } else if (status == EEPROM_WR_COMPLETE) {
      /* Header write completed immediately (small write), start data write */
      wlAsyncCtx.busyRetries = 0; /* Reset on success */
      wlAsyncCtx.state       = WL_ASYNC_START_DATA_WRITE;
      if (!timerScheduleCallback(eepromWLAsyncCallback, EEPROM_WR_TIME)) {
        printf_("EEPROM CB FAIL!\r\n");
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
    } else if (status == EEPROM_WR_TOO_SOON || status == EEPROM_WR_BUSY) {
      /* Not ready yet, retry after delay - stay in same state */
      wlAsyncCtx.busyRetries++;
      if (wlAsyncCtx.busyRetries > MAX_BUSY_RETRIES) {
        /* Stuck in BUSY - perform recovery */
        printf_("BUSY STUCK!\r\n");
        eepromBusRecovery();
        wlAsyncCtx.busyRetries = 0;
        wlAsyncCtx.state       = WL_ASYNC_IDLE;
      } else if (!timerScheduleCallback(eepromWLAsyncCallback,
                                        EEPROM_WR_TIME)) {
        printf_("EEPROM CB FAIL!\r\n");
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
    } else if (status == EEPROM_WR_FAIL) {
      printf_("WR FAIL!\r\n");
      eepromBusRecovery();
      wlAsyncCtx.busyRetries = 0;
      wlAsyncCtx.state       = WL_ASYNC_IDLE;
    }
    break;

  case WL_ASYNC_WAITING_HEADER:
    /* Continue header write */
    status = eepromWrite(0, 0, 0);
    if (status == EEPROM_WR_COMPLETE) {
      /* Header done, transition to START_DATA_WRITE state */
      wlAsyncCtx.state = WL_ASYNC_START_DATA_WRITE;
      /* Schedule callback immediately to attempt data write start */
      if (!timerScheduleCallback(eepromWLAsyncCallback, 0)) {
        printf_("EEPROM CB FAIL!\r\n");
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
    } else if (status == EEPROM_WR_PEND) {
      /* Still pending, schedule next callback */
      if (!timerScheduleCallback(eepromWLAsyncCallback, EEPROM_WR_TIME)) {
        printf_("EEPROM CB FAIL!\r\n");
        /* Drain the lower-level write to prevent stuck state */
        while (eepromWrite(0, 0, 0) != EEPROM_WR_COMPLETE) {
          timerDelay_us(EEPROM_WR_TIME);
        }
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
    } else if (status == EEPROM_WR_FAIL) {
      printf_("WR FAIL!\r\n");
      eepromBusRecovery();
      wlAsyncCtx.state = WL_ASYNC_IDLE;
    }
    break;

  case WL_ASYNC_START_DATA_WRITE:
    /* Attempt to start data write - may return TOO_SOON or BUSY */
    status = eepromWrite(wlAsyncCtx.addrWr + sizeof(WLHeader_t),
                         wlAsyncCtx.pData, wlAsyncCtx.dataLen);
    if (status == EEPROM_WR_TOO_SOON || status == EEPROM_WR_BUSY) {
      /* Not ready yet, reschedule after EEPROM_WR_TIME */
      if (!timerScheduleCallback(eepromWLAsyncCallback, EEPROM_WR_TIME)) {
        printf_("EEPROM CB FAIL!\r\n");
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
      /* Stay in START_DATA_WRITE state to retry */
    } else if (status == EEPROM_WR_PEND) {
      /* Data write started, transition to WAITING_DATA */
      wlAsyncCtx.state = WL_ASYNC_WAITING_DATA;
      if (!timerScheduleCallback(eepromWLAsyncCallback, EEPROM_WR_TIME)) {
        printf_("EEPROM CB FAIL!\r\n");
        /* Drain the lower-level write to prevent stuck state */
        while (eepromWrite(0, 0, 0) != EEPROM_WR_COMPLETE) {
          timerDelay_us(EEPROM_WR_TIME);
        }
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
    } else if (status == EEPROM_WR_COMPLETE) {
      /* Data write completed immediately (shouldn't happen but handle it) */
      wlAsyncCtx.state = WL_ASYNC_WAITING_DATA;
      if (!timerScheduleCallback(eepromWLAsyncCallback, 0)) {
        printf_("EEPROM CB FAIL!\r\n");
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
    } else if (status == EEPROM_WR_FAIL) {
      printf_("WR FAIL!\r\n");
      eepromBusRecovery();
      wlAsyncCtx.state = WL_ASYNC_IDLE;
    }
    break;

  case WL_ASYNC_WRITING_DATA:
    /* Should transition immediately to WAITING_DATA */
    wlAsyncCtx.state = WL_ASYNC_WAITING_DATA;
    if (!timerScheduleCallback(eepromWLAsyncCallback, EEPROM_WR_TIME)) {
      printf_("EEPROM CB FAIL!\r\n");
      /* Drain the lower-level write to prevent stuck state */
      while (eepromWrite(0, 0, 0) != EEPROM_WR_COMPLETE) {
        timerDelay_us(EEPROM_WR_TIME);
      }
      wlAsyncCtx.state = WL_ASYNC_IDLE;
    }
    break;

  case WL_ASYNC_WAITING_DATA:
    /* Continue data write */
    status = eepromWrite(0, 0, 0);
    if (status == EEPROM_WR_COMPLETE) {
      /* Data write complete, update wear leveling index */
      uint32_t idxWr = wlAsyncCtx.idx + 1u;
      if (idxWr == wlBlkCnt) {
        uint32_t validByte;
        if (!eepromRead(wlAsyncCtx.addrWr, &validByte, 1u)) {
          /* Read failed - cannot update valid byte, treat as error */
          wlAsyncCtx.state = WL_ASYNC_IDLE;
          break;
        }
        wlCurrentValid = nextValidByte(validByte);
        idxWr          = 0;
      }
      wlIdxNxtWr       = idxWr;
      wlAsyncCtx.state = WL_ASYNC_IDLE; /* Ready for next write */
    } else if (status == EEPROM_WR_PEND) {
      /* Still pending, schedule next callback */
      if (!timerScheduleCallback(eepromWLAsyncCallback, EEPROM_WR_TIME)) {
        printf_("EEPROM CB FAIL!\r\n");
        /* Drain the lower-level write to prevent stuck state */
        while (eepromWrite(0, 0, 0) != EEPROM_WR_COMPLETE) {
          timerDelay_us(EEPROM_WR_TIME);
        }
        wlAsyncCtx.state = WL_ASYNC_IDLE;
      }
    } else if (status == EEPROM_WR_FAIL) {
      printf_("WR FAIL!\r\n");
      eepromBusRecovery();
      wlAsyncCtx.state = WL_ASYNC_IDLE;
    }
    break;
  }
}

/*! @brief Check if an asynchronous wear-leveled write is in progress
 *  @return true if write is in progress, false otherwise
 */
bool eepromWriteWLBusy(void) { return (wlAsyncCtx.state != WL_ASYNC_IDLE); }

/*! @brief Start an asynchronous wear-leveled write operation
 *  @param [in] pPktWr : pointer to write packet
 *  @param [out] pIdx : pointer to the value of the index to write to (optional)
 *  @return EEPROM_WR_BUSY if another write is in progress, EEPROM_WR_PEND
 * otherwise
 */
eepromWrStatus_t eepromWriteWLAsync(const void *pPktWr, uint32_t *pIdx) {
  EMON32_ASSERT(pPktWr);

  /* Atomic check-and-set to prevent race condition */
  __disable_irq();
  if (wlAsyncCtx.state != WL_ASYNC_IDLE) {
    __enable_irq();
    return EEPROM_WR_BUSY;
  }
  /* Reserve the state machine immediately */
  wlAsyncCtx.state = WL_ASYNC_WRITING_HEADER;
  __enable_irq();

  /* Find the next write location if not yet set */
  if (UINT32_MAX == wlIdxNxtWr) {
    wlFindLast();
  }

  /* Copy data to static buffer to ensure it doesn't change during async write
   */
  memcpy(wlData, pPktWr, wlData_n);

  /* Prepare the header */
  wlAsyncCtx.header.res0        = 0;
  wlAsyncCtx.header.valid       = wlCurrentValid;
  wlAsyncCtx.header.crc16_ccitt = calcCRC16_ccitt(wlData, wlData_n);

  /* Store the context */
  wlAsyncCtx.idx     = wlIdxNxtWr;
  wlAsyncCtx.addrWr  = EEPROM_WL_OFFSET + (wlIdxNxtWr * wlBlkSize);
  wlAsyncCtx.pData   = wlData;
  wlAsyncCtx.dataLen = wlData_n;

  if (pIdx) {
    *pIdx = wlAsyncCtx.idx;
  }

  /* Start the state machine by scheduling the first callback */
  if (!timerScheduleCallback(eepromWLAsyncCallback, 0)) {
    /* Callback queue full - cannot start async write */
    wlAsyncCtx.state = WL_ASYNC_IDLE; /* Release reservation */
    return EEPROM_WR_FAIL;
  }

  return EEPROM_WR_PEND;
}

eepromWrStatus_t eepromWriteWL(const void *pPktWr, uint32_t *pIdx) {
  /* Check for correct indexing, find if not yet set; this is indicated by
   * wlIdxNxtWr == UINT32_MAX. Write output to new levelled position.
   */
  uint32_t         idxWr;
  uint32_t         addrWr;
  WLHeader_t       header;
  eepromWrStatus_t wrStatus;

  if (UINT32_MAX == wlIdxNxtWr) {
    wlFindLast();
  }

  header.res0        = 0;
  header.valid       = wlCurrentValid;
  header.crc16_ccitt = calcCRC16_ccitt(pPktWr, wlData_n);

  if (pIdx) {
    *pIdx = wlIdxNxtWr;
  }
  addrWr = EEPROM_WL_OFFSET + (wlIdxNxtWr * wlBlkSize);

  /* Write the header followed by the data */
  do {
    wrStatus = eepromWrite(addrWr, &header, sizeof(header));
    if (wrStatus == EEPROM_WR_TOO_SOON) {
      timerDelay_us(EEPROM_WR_TIME);
    }
  } while (wrStatus == EEPROM_WR_TOO_SOON);

  if ((wrStatus != EEPROM_WR_PEND) && (wrStatus != EEPROM_WR_COMPLETE)) {
    return wrStatus;
  }

  while (EEPROM_WR_COMPLETE != wrStatus) {
    timerDelay_us(EEPROM_WR_TIME);
    wrStatus = eepromWrite(0, 0, 0);
  }

  timerDelay_us(EEPROM_WR_TIME);

  do {
    wrStatus = eepromWrite((addrWr + sizeof(header)), pPktWr, wlData_n);
    if (wrStatus == EEPROM_WR_TOO_SOON) {
      timerDelay_us(EEPROM_WR_TIME);
    }
  } while (wrStatus == EEPROM_WR_TOO_SOON);

  while (EEPROM_WR_COMPLETE != wrStatus) {
    timerDelay_us(EEPROM_WR_TIME);
    wrStatus = eepromWrite(0, 0, 0);
  }

  /* Wait for EEPROM internal write cycle to complete (per datasheet: 5ms max)
   */
  timerDelay_us(EEPROM_WR_TIME);

  /* Once all blocks with the same "valid" byte have been written out,
   * generate the next "valid" byte to be used and wrap.
   */
  idxWr = wlIdxNxtWr + 1u;
  if (idxWr == wlBlkCnt) {
    uint32_t validByte;
    eepromRead(addrWr, &validByte, 1u);
    wlCurrentValid = nextValidByte(validByte);
    idxWr          = 0;
  }

  wlIdxNxtWr = idxWr;
  return EEPROM_WR_WL_COMPLETE;
}
