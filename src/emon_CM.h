#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_def.h"

#ifndef RAMFUNC
#define RAMFUNC
#endif

/* Use empty RAMFUNC when running locally */
#ifdef HOSTED
#undef RAMFUNC
#define RAMFUNC
#endif

/******************************************************************************
 * Type definitions
 *****************************************************************************/

typedef enum {
  ECM_INIT_SUCCESS,      /* Init was successful */
  ECM_INIT_FAIL_ENABLED, /* Init failed as currently enabled */
  ECM_ENABLE_SUCCESS,
  ECM_ENABLE_FAIL_ENABLED,
  ECM_CYCLE_ONGOING,   /* A mains cycle is being accumulated */
  ECM_CYCLE_COMPLETE,  /* A full mains cycle has completed */
  ECM_REPORT_ONGOING,  /* A full set is accumulating */
  ECM_REPORT_COMPLETE, /* A full set to report is complete */
  ECM_PEND_1S          /* 1s until the set is due to complete */
} ECM_STATUS_t;

/* Alias integer types for fixed point calculation */
typedef int16_t q15_t;
typedef int32_t q31_t;
typedef int32_t q22_t;
typedef int64_t q63_t;

/* SingleSampleSet_t contains a single set of V + CT ADC samples */
typedef struct __attribute__((__packed__)) SingleSampleSet_ {
  q15_t smp[VCT_TOTAL];
} SingleRawSampleSet_t;

/* RawSampleSetPacked_t contains a set of single sample sets. This allows the
 * DMAC to blit samples across multiple sample sets, depending on processing
 * needs.
 */
typedef struct __attribute__((__packed__)) SampleSetPacked_ {
  SingleRawSampleSet_t samples[SAMPLES_IN_SET];
} RawSampleSetPacked_t;

/* SampleSet_t contains an unpacked set of single sample sets */
typedef struct SampleSet_ {
  q15_t smpV[NUM_V];
  q15_t smpCT[NUM_CT];
} SampleSet_t;

typedef struct GainOffset_ {
  bool    valid;
  int16_t gain;
  int16_t offset;
} GainOffset_t;

typedef struct VCfg_ {
  float voltageCal;
  float voltageCalRaw;
  float phase;
  bool  vActive;
} VCfg_t;

typedef struct CTCfgUnpacked_ {
  float   phaseX;
  float   phaseY;
  float   phCal;
  float   ctCal;
  float   ctCalRaw;
  bool    active;
  int32_t vChan1;
  int32_t vChan2;
  int32_t wattHourInit;
  int32_t idxInterpolateCT;
  int32_t idxInterpolateV;
} CTCfg_t;

typedef struct ECMCfg_ {
  uint32_t (*timeMicros)(void);          /* Time in microseconds now */
  uint32_t (*timeMicrosDelta)(uint32_t); /* Time delta in microseconds */

  bool     downsample;    /* DSP enabled */
  int32_t  reportCycles;  /* Number of cycles before reporting */
  int32_t  mainsFreq;     /* Mains frequency */
  int32_t  samplePeriod;  /* Sampling period for each sample */
  uint32_t reportTime_us; /* Report time in microseconds */
  float    assumedVrms;   /* Assume RMS voltage if not found */

  int_fast8_t mapCTLog[NUM_CT]; /* Map of CT to microcontroller pins */

  GainOffset_t correction; /* Gain and offset correction */

  CTCfg_t ctCfg[NUM_CT]; /* CT Configuration */
  VCfg_t  vCfg[NUM_V];   /* Voltage configuration */
} ECMCfg_t;

typedef struct DataCT_ {
  float   rmsI;
  float   pf;
  int32_t realPower;
  int32_t apparentPower;
  int32_t wattHour;
} DataCT_t;

typedef struct ECMDataset_ {
  float    wallTime;
  uint32_t activeCh;
  float    rmsV[NUM_V * 2]; /* For L-L */
  DataCT_t CT[NUM_CT];
} ECMDataset_t;

typedef struct ECMPerformance_ {
  int32_t numSlices;
  int32_t microsSlices;
  int32_t numCycles;
  int32_t microsCycles;
  int32_t numDatasets;
  int32_t microsDatasets;
} ECMPerformance_t;

typedef struct AutoPhaseRes_ {
  int32_t idxCt;
  float   phase;
  bool    success;
} AutoPhaseRes_t;

/******************************************************************************
 * Function prototypes
 *****************************************************************************/

/*! @brief Clear accumulated energy in dataset */
void ecmClearEnergy(void);

/*! @brief Clear accumulated energy for a single channel
 *  @param [in] idx : channel index (0 to NUM_CT-1)
 */
void ecmClearEnergyChannel(int32_t idx);

/*! @brief Get the pointer to the configuration struct
 *  @return pointer to Emon CM configuration struct
 */
ECMCfg_t *ecmConfigGet(void);

/*! @brief Configure a channel.
 *  @param [in] ch : channel, logical index.
 */
void ecmConfigChannel(int_fast8_t ch);

/*! @brief Having set all configuration values, calculate all required constant
 *         values
 */
void ecmConfigInit(void);

/*! @brief Set cycles between reports
 *  @param [in] reportCycles : cycles between reports
 */
void ecmConfigReportCycles(int32_t reportCycles);

/*! @brief Returns a pointer to the ADC data buffer
 *  @return pointer to the active ADC data buffer.
 */
volatile RawSampleSetPacked_t *ecmDataBuffer(void);

/*! @brief Swap the data sampling buffers. ADC will be filling the other
 *         while it is handled.
 */
void ecmDataBufferSwap(void);

/*! @brief Unpack and optionally low pass filter the raw sample
 *         The struct from the DMA has no partition into V/CT channels, so
 *         alter this function to move data from the implementation specific
 *         DMA addressess to the defined SampleSet_t fields
 *  @param [out] pDst : pointer to the SampleSet_t destination
 */
void ecmFilterSample(SampleSet_t *pDst) RAMFUNC;

/*! @brief Flush all data and reset the equilibration cycle count */
void ecmFlush(void);

/*! @brief Injects a raw sample from the ADC into the accumulators. */
ECM_STATUS_t ecmInjectSample(void) RAMFUNC;

/*! @brief Gets the performance counter
 *  @return pointer to the performance counter
 */
ECMPerformance_t *ecmPerformance(void);

/*! @brief Calibrate a CT sensor's lead against the input voltage
 *  @param [in] pDst : autophase structure
 */
void ecmPhaseCalibrate(AutoPhaseRes_t *pDst);

/*! @brief Processes a whole cycle */
ECM_STATUS_t ecmProcessCycle(void) RAMFUNC;

/*! @brief Processes a whole data set
 *  @param [out] pData : pointer to the processed data structure
 */
ECMDataset_t *ecmProcessSet(void) RAMFUNC;

/*! @brief Force trigger data set processing on next cycle complete */
void ecmProcessSetTrigger(void);
