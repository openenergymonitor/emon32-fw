#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "emon_CM.h"

_Static_assert((sizeof(bool) == 1), "bool must be 1 byte");

/*********************************
 * Common configurable options
 *********************************/

#define DOWNSAMPLE_DSP     1u   /* 0: no downsampling; 1: half band LPF */
#define NUM_CT_ACTIVE_DEF  6    /* Onboard CTs only */
#define DELTA_WH_STORE_DEF 200u /* Threshold, in Wh, to store to NVM */
#define NODE_ID_DEF        17u  /* Node ID for reports */
#define GROUP_ID_DEF       210u /* Group ID default for OEM */
#define MAINS_FREQ_DEF     50u  /* Mains frequency */
#define REPORT_TIME_DEF    9.8f /* Report time, in seconds */
#define ASSUMED_VRMS_DEF   240  /* Assumed voltage, used if no AC sense */
#define PERF_ENABLED       0u   /* Performance tracing */
#define TX_INDICATE_T      250u /* Transmission indication time (ms) */

/*********************************
 * Firmware version
 *********************************/
#define VERSION_FW_MAJ 0u
#define VERSION_FW_MIN 2u
#define VERSION_FW_REV 0u

/*********************************
 * Remaining
 *********************************/

#define TX_BUFFER_W 512u

/* Configuration key - indicates that the configuration is the default or
 * has been retrieved from non-volatile storage */
#define CONFIG_NVM_KEY 0xca55e77eul

typedef struct Emon32Dataset_ {
  uint32_t      msgNum;
  ECMDataset_t *pECM;
  uint32_t      pulseCnt[NUM_OPA];
  int16_t       temp[TEMP_MAX_ONEWIRE];
} Emon32Dataset_t;

typedef struct __attribute__((__packed__)) Emon32Cumulative_ {
  uint32_t wattHour[NUM_CT];
  uint32_t pulseCnt[NUM_OPA];
} Emon32Cumulative_t;

/* This struct must match the OEM definitions found at:
 * https://docs.openenergymonitor.org/electricity-monitoring/networking/sending-data-between-nodes-rfm.html
 */
typedef struct __attribute__((__packed__)) PackedData_ {
  uint32_t msg;
  int16_t  V[NUM_V];
  int16_t  P[NUM_CT];
  int32_t  E[NUM_CT];
  int16_t  T[TEMP_MAX_ONEWIRE];
  uint32_t pulse[NUM_OPA];
} PackedData_t;

typedef struct __attribute__((__packed__)) PackedDataCommon_ {
  uint32_t msg;
  int16_t  V[NUM_V];
  int16_t  P[NUM_CT / 2];
  int32_t  E[NUM_CT / 2];
} PackedDataCommon_t;

typedef struct __attribute__((__packed__)) PackedDataLower6_ {
  PackedDataCommon_t common;
  uint32_t           pulse[NUM_OPA];
} PackedDataLower6_t;

typedef struct __attribute__((__packed__)) PackedDataUpper6_ {
  PackedDataCommon_t common;
  int16_t            temp[4];
} PackedDataUpper6_t;

/* Maximum size of RFM69CW buffer is 61 bytes. Node, number, and CRC included.
 */
_Static_assert((sizeof(PackedDataLower6_t) + 4) < 62,
               "PackedDataLower6_t > 61 bytes");
_Static_assert((sizeof(PackedDataUpper6_t) + 4) < 62,
               "PackedDataUpper6_t > 61 bytes");

/* EVTSRC_t contains all the event/interrupts sources. This value is shifted
 * to provide a vector of set events as bits.
 */
typedef enum EVTSRC_ {
  EVT_EXT_DISABLE     = 0u,
  EVT_TICK_1kHz       = 1u,
  EVT_ECM_SET_CMPL    = 8u,
  EVT_TEMP_READ       = 15u,
  EVT_CONFIG_CHANGED  = 16u,
  EVT_CONFIG_SAVED    = 17u,
  EVT_SAFE_RESET_REQ  = 18u,
  EVT_PROCESS_CMD     = 19u,
  EVT_PROCESS_DATASET = 20u,
  EVT_CLEAR_ACCUM     = 22u,
  EVT_ECM_PEND_1S     = 23u,
  EVT_ECM_TRIG        = 24u
} EVTSRC_t;

/*! @brief When enabled, output debug message to serial (USB if available, and
 *         hardware UART) including a millisecond timestamp.
 *  @param [in] s: pointer to null terminated string
 */
void debugPuts(const char *s);

/*! @brief Clear a pending event/interrupt flag after the task has been handled
 *  @param [in] Event source in enum
 */
void emon32EventClr(const EVTSRC_t evt);

/*! @brief Set the pending event/interrupt flag for tasks that are not handled
 *         within an ISR
 *  @param [in] evt : Event source in enum
 */
void emon32EventSet(const EVTSRC_t evt);

/*! @brief Output to serial (USB if available, and hardware UART).
 *  @param [in] s: pointer to null terminated string
 */
void serialPuts(const char *s);
