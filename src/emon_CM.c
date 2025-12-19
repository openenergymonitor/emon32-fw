#include <stdbool.h>
#include <string.h>

#ifndef HOSTED

#include "qfplib-m0-full.h"

#else

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "emonCM_test.h"

#endif /* HOSTED */

#include "emon_CM.h"
#include "emon_CM_coeffs.h"

#define PROC_DEPTH   16 /* Voltage sample buffer depth. Must be power of 2. */
#define ZC_HYST      2  /* Zero crossing hysteresis */
#define ZC_HYST_AV   8  /* Zero crossing hysteresis when using assumd voltage */
#define EQUIL_CYCLES 8  /* Number of cycles to discard at startup */
#define ZC_MIN_VPEAK                                                           \
  40 /* Minimum peak voltage to accept zero-crossings (40 counts = ~14V mains) \
      */
#define ZC_PERIOD_MIN_US                                                       \
  14000 /* Minimum period between crossings (14ms = ~71Hz, 60Hz +13.5%         \
           tolerance) */
#define ZC_PERIOD_MAX_US                                                       \
  25000 /* Maximum period between crossings (25ms = 40Hz) */

_Static_assert(!(PROC_DEPTH & (PROC_DEPTH - 1)),
               "PROC_DEPTH is not a power of 2.");

static const float TWO_PI = (6.2831853072f);

static bool channelActive[VCT_TOTAL] = {0};
static bool threePhase               = false;
static bool useAssumedV              = false;

/*************************************
 * Local typedefs
 *************************************/

/* @typedef
 * @brief This struct contains V and CT phase information
 */
typedef struct PhaseCal_ {
  int   low;       /* RESERVED */
  float phaseLow;  /* Phase error at and below low limit */
  int   high;      /* RESERVED */
  float phaseHigh; /* RESERVED */
} PhaseCal_t;

typedef struct PhaseData_ {
  int   positionOfV;         /* Voltage index */
  float phaseErrorV;         /* Voltage phase error (degrees) */
  int   positionOfC;         /* CT index */
  float phaseErrorC;         /* CT phase error (degrees) */
  int   relativeCSample;     /* Position of current relative to this */
  int   relativeLastVSample; /* Position of previous V */
  int   relativeThisVSample; /* Position of next V */
  float x;                   /* Coefficient for interpolation */
  float y;                   /* Coefficient for interpolation */
} PhaseData_t;

typedef enum Polarity_ { POL_POS, POL_NEG } Polarity_t;

typedef struct VAccumulator_ {
  int64_t sumV_sqr;
  int     sumV_deltas;
} VAccumulator_t;

typedef struct CTAccumulator_ {
  int64_t sumPA[2];
  int64_t sumPB[2];
  int64_t sumI_sqr;
  int     sumI_deltas;
} CTAccumulator_t;

typedef struct Accumulator_ {
  VAccumulator_t  processV[NUM_V * 2]; /* Additional space for 3-phase L-L */
  CTAccumulator_t processCT[NUM_CT];
  int             numSamples;
  int             cycles;
  uint32_t        tStart_us;
  uint32_t        tDelta_us;
} Accumulator_t;

typedef struct CalcRMS_ {
  float   cal;
  int64_t sSqr;
  int     sDelta;
  int     numSamples;
} CalcRMS_t;

typedef struct RawSampleSetUnpacked {
  q15_t smp[VCT_TOTAL];
} RawSampleSetUnpacked_t;

typedef struct vSmp_ {
  q15_t smpV[NUM_V];
} vSmp_t;

/*************************************
 * Function prototypes
 *************************************/

static inline q15_t __STRUNCATE(int32_t val) RAMFUNC;
static q15_t        applyCorrection(q15_t smp) RAMFUNC;
static float        calcRMS(CalcRMS_t *pSrc) RAMFUNC;
static bool         zeroCrossingSW(q15_t smpV, uint32_t timeNow_us) RAMFUNC;

static void  accumSwapClear(void);
static float calibrationAmplitude(float cal, bool isV);
static void  calibrationPhase(CTCfg_t *pCfg, int_fast8_t idxCT);
static void  configChannelV(int_fast8_t ch);
static void  configChannelCT(int_fast8_t ch);
static void  swapPtr(void **pIn1, void **pIn2);

/******************************************************************************
 * Pre-processing
 *****************************************************************************/

static RawSampleSetUnpacked_t dspBuffer[DOWNSAMPLE_TAPS];
static vSmp_t                 vSampleBuffer[PROC_DEPTH];

/******************************************************************************
 * Accumulators
 *****************************************************************************/

static Accumulator_t  accumBuffer[2];
static Accumulator_t *accumCollecting = accumBuffer;
static Accumulator_t *accumProcessing = accumBuffer + 1;

static ECMPerformance_t  perfCounter[2];
static ECMPerformance_t *perfActive = perfCounter;
static ECMPerformance_t *perfIdle   = perfCounter + 1;

static ECMDataset_t datasetProc = {0};

static uint32_t t_ZClast = 0;

/******** FIXED POINT MATHS FUNCTIONS ********
 *
 * Adapted from Arm CMSIS-DSP: https://github.com/ARM-software/CMSIS-DSP
 */

/*! @brief Truncate Q31 to a Q15 fixed point, round to nearest LSB
 *  @param [in] val value to truncate
 *  @return Q15 truncated val
 */
static RAMFUNC inline q15_t __STRUNCATE(int32_t val) {
  int roundUp = 0;
  if (0 != (val & (1u << 14))) {
    roundUp = 1;
  }
  return (q15_t)((val >> 15) + roundUp);
}

/***** END FIXED POINT FUNCIONS *****/

static RAMFUNC float calcRMS(CalcRMS_t *pSrc) {
  int64_t numSamplesSqr = (int64_t)pSrc->numSamples * pSrc->numSamples;
  float   vcal          = pSrc->cal;

  int32_t deltasSqr = pSrc->sDelta * pSrc->sDelta;

  float offsetCorr =
      qfp_fdiv(qfp_int2float(deltasSqr), qfp_int642float(numSamplesSqr));

  float rms =
      qfp_fdiv(qfp_int642float(pSrc->sSqr), qfp_int2float(pSrc->numSamples));

  rms = qfp_fsub(rms, offsetCorr);
  rms = qfp_fsqrt(rms);
  rms = qfp_fmul(vcal, rms);

  return rms;
}

/*! @brief Swap pointers to buffers */
static void swapPtr(void **pIn1, void **pIn2) {
  void *tmp = *pIn1;
  *pIn1     = *pIn2;
  *pIn2     = tmp;
}

/******************************************************************************
 * Configuration
 *****************************************************************************/

static ECMCfg_t    ecmCfg           = {0};
static bool        processTrigger   = false;
static int_fast8_t mapLogCT[NUM_CT] = {0};
static int_fast8_t discardCycles    = EQUIL_CYCLES;
static bool        initDone         = false;
static bool        inAutoPhase      = false;
static int         samplePeriodus;
static float       sampleIntervalRad;

ECMCfg_t *ecmConfigGet(void) { return &ecmCfg; }

void ecmConfigChannel(int_fast8_t ch) {
  if (ch < NUM_V) {
    configChannelV(ch);
    if ((NUM_V == 3) && channelActive[1] && channelActive[2]) {
      threePhase = true;
    } else {
      threePhase = false;
    }
  } else {
    configChannelCT(ch - NUM_V);
  }
}

void configChannelCT(int_fast8_t ch) {
  channelActive[ch + NUM_V] = ecmCfg.ctCfg[ch].active;

  if (ecmCfg.ctCfg[ch].active) {
    datasetProc.activeCh |= 1u << (ch + NUM_V);
  } else {
    datasetProc.activeCh &= ~(1u << (ch + NUM_V));
  }

  ecmCfg.ctCfg[ch].ctCal =
      calibrationAmplitude(ecmCfg.ctCfg[ch].ctCalRaw, false);

  calibrationPhase(&ecmCfg.ctCfg[ch], ch);
}

void configChannelV(int_fast8_t ch) {
  channelActive[ch] = ecmCfg.vCfg[ch].vActive;

  if (ecmCfg.vCfg[ch].vActive) {
    datasetProc.activeCh |= (1u << ch);
  } else {
    datasetProc.activeCh &= ~(1u << ch);
  }

  ecmCfg.vCfg[ch].voltageCal =
      calibrationAmplitude(ecmCfg.vCfg[ch].voltageCalRaw, true);
}

void ecmConfigInit(void) {

  /* Calculate the angular sampling rate in radians.
   *  - Sample rate, f_smp, in Hz: 1E6 / (t_s * OS_R * VCT)
   *  - Sample rate in °: 360 * f_mains / f_smp
   *  - Sample rate in rad: 2π / 360 * °
   *  => 2π / 1E6 * f_mains * t_s * OS_R * VCT (360s cancel)
   */
  samplePeriodus       = ecmCfg.samplePeriod * OVERSAMPLING_RATIO;
  uint32_t setPeriodus = samplePeriodus * VCT_TOTAL;

  sampleIntervalRad =
      qfp_fmul((TWO_PI / 1E6f), qfp_int2float(ecmCfg.mainsFreq * setPeriodus));

  /* Map the logical channel back to the CT to unwind the data */
  for (int_fast8_t i = 0; i < NUM_CT; i++) {
    mapLogCT[ecmCfg.mapCTLog[i]] = i;
  }

  for (int_fast8_t i = 0; i < NUM_V; i++) {
    configChannelV(i);
  }

  /* 3-phase if all present and active */
  if ((NUM_V == 3) && channelActive[1] && channelActive[2]) {
    threePhase = true;
  }

  /* Configure each CT channel and load the initial Wh value from NVM. */
  for (int_fast8_t i = 0; i < NUM_CT; i++) {
    configChannelCT(i);
    datasetProc.CT[i].wattHour = ecmCfg.ctCfg[i].wattHourInit;
  }

  initDone = true;
}

void ecmConfigReportCycles(int reportCycles) {
  ecmCfg.reportCycles = reportCycles;
}

/******************************************************************************
 * Data acquisition
 *****************************************************************************/

static volatile RawSampleSetPacked_t adcSamples[SAMPLE_BUF_DEPTH];
static volatile RawSampleSetPacked_t *volatile adcActive = adcSamples;
static volatile RawSampleSetPacked_t *volatile adcProc   = adcSamples + 1;

static float residualEnergy[NUM_CT] = {0};

void ecmDataBufferSwap(void) {
  swapPtr((void **)&adcActive, (void **)&adcProc);
}

volatile RawSampleSetPacked_t *ecmDataBuffer(void) { return adcActive; }

/******************************************************************************
 * Functions
 *****************************************************************************/

static RAMFUNC q15_t applyCorrection(q15_t smp) {
  if (ecmCfg.correction.valid) {
    int result = smp + ecmCfg.correction.offset;
    result *= ecmCfg.correction.gain;
    result >>= 11;
    return result;
  } else {
    return smp;
  }
}

static void accumSwapClear(void) {
  swapPtr((void **)&accumCollecting, (void **)&accumProcessing);
  memset((void *)accumCollecting, 0, sizeof(*accumCollecting));
}

/*! @brief Zero crossing detection, software
 *  @param [in] smpV : current voltage sample
 *  @param [in] timeNow_us : current time in microseconds
 *  @return true for positive crossing, false otherwise
 */
RAMFUNC bool zeroCrossingSW(q15_t smpV, uint32_t timeNow_us) {
  static Polarity_t  polarityLast     = POL_POS;
  static int_fast8_t hystCnt          = ZC_HYST;
  static q15_t       vPeakSinceLastZC = 0;
  static uint32_t    lastZC_us        = 0;
  Polarity_t         polarityNow      = (smpV < 0) ? POL_NEG : POL_POS;

  /* Track peak voltage magnitude since last zero-crossing */
  q15_t absV = (smpV < 0) ? -smpV : smpV;
  if (absV > vPeakSinceLastZC) {
    vPeakSinceLastZC = absV;
  }

  if (polarityNow != polarityLast) {
    hystCnt--;
    if (0 == hystCnt) {
      hystCnt      = ZC_HYST;
      polarityLast = polarityNow;
      if (POL_POS == polarityNow) {
        /* Validate zero-crossing before accepting it:
         * 1. Must have seen sufficient voltage amplitude
         * 2. Period since last crossing must be reasonable
         */
        bool validAmplitude = (vPeakSinceLastZC >= ZC_MIN_VPEAK) || useAssumedV;
        bool validPeriod    = true;

        if (lastZC_us != 0 && timeNow_us != 0) {
          uint32_t period_us = timeNow_us - lastZC_us;
          /* Accept period if within reasonable bounds for 40-71 Hz */
          validPeriod        = (period_us >= ZC_PERIOD_MIN_US &&
                         period_us <= ZC_PERIOD_MAX_US) ||
                        useAssumedV;
        }

        if (validAmplitude && validPeriod) {
          lastZC_us        = timeNow_us;
          vPeakSinceLastZC = 0; /* Reset for next cycle */
          return true;
        }
        /* Invalid crossing - reset peak tracker anyway */
        vPeakSinceLastZC = 0;
      }
    }
  } else {
    hystCnt = useAssumedV ? ZC_HYST_AV : ZC_HYST;
  }
  return false;
}

/*! @brief Turn an amplitude calibration value into a factor to change the
 *         abstract value into the real value, accounting for ADC width.
 *  @param [in] cal : the calibration value
 *  @param [in] isV : true for voltage, false for CT
 *  @return the scaled calibration value
 */
static float calibrationAmplitude(float cal, bool isV) {
  const float vCalRef  = (CAL_V * ADC_VREF) / (1 << ADC_RES_BITS);
  const float ctCalRef = (CAL_CT * ADC_VREF) / (1 << ADC_RES_BITS);

  if (isV) {
    return qfp_fmul(cal, vCalRef);
  } else {
    return qfp_fmul(cal, ctCalRef);
  }
}

/*! @brief Decompose a floating point CT phase into an X/Y pair for
 *         interpolation.
 *  @param [out] pPh : pointer to phase struct
 *  @param [in] idxCT : physical index (0-based) of the CT
 */
static void calibrationPhase(CTCfg_t *pCfg, int_fast8_t idxCT) {
  int_fast8_t idxMapped = ecmCfg.mapCTLog[idxCT];

  float phaseShift_rad = qfp_fmul(pCfg->phCal, (TWO_PI / 360.0f));

  int   phaseCorr_i   = (idxMapped + NUM_V) * ecmCfg.mainsFreq * samplePeriodus;
  float phaseCorr_rad = qfp_fmul(qfp_int2float(phaseCorr_i), (TWO_PI / 1E6f));

  phaseShift_rad = qfp_fadd(phaseShift_rad, phaseCorr_rad);

  while (phaseShift_rad > sampleIntervalRad) {
    phaseShift_rad = qfp_fsub(phaseShift_rad, sampleIntervalRad);
    pCfg->idxInterpolate++;
  }

  pCfg->phaseY =
      qfp_fdiv(qfp_fsin(phaseShift_rad), qfp_fsin(sampleIntervalRad));

  pCfg->phaseX =
      qfp_fsub(qfp_fcos(phaseShift_rad),
               (qfp_fmul(pCfg->phaseY, qfp_fcos(sampleIntervalRad))));
}

void ecmClearEnergy(void) {
  for (int i = 0; i < NUM_CT; i++) {
    datasetProc.CT[i].wattHour = 0;
    residualEnergy[i]          = 0.0f;
  }
}

void ecmPhaseCalibrate(AutoPhaseRes_t *pDst) {
  pDst->success = false;
  if (!(channelActive[pDst->idxCt + NUM_V])) {
    return;
  }
  inAutoPhase = true;

  pDst->success = true;
  inAutoPhase   = false;
}

void ecmFlush(void) {
  discardCycles = EQUIL_CYCLES;

  memset(accumBuffer, 0, (2 * sizeof(*accumBuffer)));
  memset(dspBuffer, 0, (DOWNSAMPLE_TAPS * sizeof(*dspBuffer)));
  memset(&residualEnergy, 0, (sizeof(*residualEnergy) * NUM_CT));
  t_ZClast = 0;
}

RAMFUNC void ecmFilterSample(SampleSet_t *pDst) {
  if (!ecmCfg.downsample) {
    /* No filtering, discard the second sample in the set */
    for (int_fast8_t idxV = 0; idxV < NUM_V; idxV++) {
      pDst->smpV[idxV] = applyCorrection(adcProc->samples[0].smp[idxV]);
    }

    for (int_fast8_t idxCT = 0; idxCT < NUM_CT; idxCT++) {
      pDst->smpCT[mapLogCT[idxCT - NUM_V]] =
          applyCorrection(adcProc->samples[0].smp[idxCT + NUM_V]);
    }
  } else {
    /* The FIR half band filter is symmetric, so the coefficients are folded.
     * Alternating coefficients are 0, so are not included in any outputs.
     * For an ODD number of taps, the centre coefficent is handled
     * individually, then the other taps in a loop.
     *
     * b_0 | b_2 | .. | b_X | .. | b_2 | b_0
     *
     * For an EVEN number of taps, loop across all the coefficients:
     *
     * b_0 | b_2 | .. | b_2 | b_0
     */
    static uint_fast8_t idxInj         = 0;
    const uint32_t      downsampleTaps = DOWNSAMPLE_TAPS;

    const uint_fast8_t idxInjPrev =
        (0 == idxInj) ? (downsampleTaps - 1u) : (idxInj - 1u);

    /* Copy the packed raw ADC value into the unpacked buffer; samples[1] is
     * the most recent sample.
     */
    for (int_fast8_t idxSmp = 0; idxSmp < VCT_TOTAL; idxSmp++) {
      dspBuffer[idxInjPrev].smp[idxSmp] =
          applyCorrection(adcProc->samples[0].smp[idxSmp]);
      dspBuffer[idxInj].smp[idxSmp] =
          applyCorrection(adcProc->samples[1].smp[idxSmp]);
    }

    /* For an ODD number of taps, take the unique middle value to start. As
     * the filter is symmetric, this is the final element in the array.
     */
    const q15_t  coeffMid = firCoeffs[numCoeffUnique - 1u];
    uint_fast8_t idxMid   = idxInj + (downsampleTaps / 2) + 1u;
    if (idxMid >= downsampleTaps)
      idxMid -= downsampleTaps;

    /* Loop over the FIR coefficients, sub loop through channels. The filter
     * is folded so the symmetric FIR coefficients are used for both samples.
     */
    uint_fast8_t idxSmp[numCoeffUnique - 1][2];
    uint_fast8_t idxSmpStart = idxInj;
    uint_fast8_t idxSmpEnd =
        ((downsampleTaps - 1u) == idxInj) ? 0 : (idxInj + 1u);
    if (idxSmpEnd >= downsampleTaps)
      idxSmpEnd -= downsampleTaps;

    idxSmp[0][0] = idxSmpStart;
    idxSmp[0][1] = idxSmpEnd;

    /* Build a table of indices for the samples and coeffecietns. Converge
     * toward the middle, checking for over/underflow */
    for (int_fast8_t i = 1; i < (numCoeffUnique - 1); i++) {
      idxSmpStart -= 2u;
      if (idxSmpStart > downsampleTaps)
        idxSmpStart += downsampleTaps;

      idxSmpEnd += 2u;
      if (idxSmpEnd >= downsampleTaps)
        idxSmpEnd -= downsampleTaps;

      idxSmp[i][0] = idxSmpStart;
      idxSmp[i][1] = idxSmpEnd;
    }

    for (int_fast8_t ch = 0; ch < VCT_TOTAL; ch++) {
      q15_t result;
      bool  active = (ch < NUM_V) ? channelActive[ch]
                                  : channelActive[mapLogCT[ch - NUM_V] + NUM_V];

      if (active) {
        int32_t intRes = coeffMid * dspBuffer[idxMid].smp[ch];

        for (int fir = 0; fir < (numCoeffUnique - 1); fir++) {
          const q15_t coeff = firCoeffs[fir];
          intRes += coeff * (dspBuffer[idxSmp[fir][0]].smp[ch] +
                             dspBuffer[idxSmp[fir][1]].smp[ch]);
        }
        result = __STRUNCATE(intRes);

      } else {
        result = 0;
      }
      if (ch < NUM_V) {
        pDst->smpV[ch] = result;
      } else {
        /* Map the logical input to the CT channel */
        pDst->smpCT[mapLogCT[ch - NUM_V]] = result;
      }
    }

    /* Each injection is 2 samples */
    idxInj += 2u;
    if (idxInj > (downsampleTaps - 1)) {
      idxInj -= (downsampleTaps);
    }
  }
}

RAMFUNC ECM_STATUS_t ecmInjectSample(void) {
  bool            pend1s      = false;
  bool            reportReady = false;
  SampleSet_t     smpSet      = {0};
  static uint32_t t_RepLast   = 0;
  uint32_t        t_start     = 0;
  bool            zcFlag      = false;

  static int_fast8_t idxInject;
  const int_fast8_t  idxLast = (idxInject - 1) & (PROC_DEPTH - 1);

  /* Hold the previous CT value for interpolation if required */
  static q15_t smpCTLast[NUM_CT];

  if (0 != ecmCfg.timeMicros) {
    t_start = (*ecmCfg.timeMicros)();
  }

  ecmFilterSample(&smpSet);
  for (int_fast8_t i = 0; i < NUM_V; i++) {
    vSampleBuffer[idxInject].smpV[i] = smpSet.smpV[i];
  }
  accumCollecting->numSamples++;

  for (int_fast8_t idxV = 0; idxV < NUM_V; idxV++) {
    if (channelActive[idxV]) {
      int32_t V = smpSet.smpV[idxV];
      accumCollecting->processV[idxV].sumV_sqr += (int64_t)(V * V);
      accumCollecting->processV[idxV].sumV_deltas += V;
    }
  }

  /* 3-phase L-L values. Conventionally, line crossings go 1->2->3, so capture
   * correct differences. */
  if (threePhase) {
    int32_t v1   = smpSet.smpV[0];
    int32_t v2   = smpSet.smpV[1];
    int32_t v3   = smpSet.smpV[2];
    int32_t v1v2 = v1 - v2;
    int32_t v2v3 = v2 - v3;
    int32_t v3v1 = v3 - v1;
    accumCollecting->processV[3].sumV_sqr += (int64_t)(v1v2 * v1v2);
    accumCollecting->processV[3].sumV_deltas += v1v2;
    accumCollecting->processV[4].sumV_sqr += (int64_t)(v2v3 * v2v3);
    accumCollecting->processV[4].sumV_deltas += v2v3;
    accumCollecting->processV[5].sumV_sqr += (int64_t)(v3v1 * v3v1);
    accumCollecting->processV[5].sumV_deltas += v3v1;
  }

  for (int_fast8_t idxCT = 0; idxCT < NUM_CT; idxCT++) {
    if (channelActive[idxCT + NUM_V]) {

      int32_t thisCT   = (0 == ecmCfg.ctCfg[idxCT].idxInterpolate)
                             ? smpSet.smpCT[idxCT]
                             : smpCTLast[idxCT];
      smpCTLast[idxCT] = smpSet.smpCT[idxCT];

      int_fast8_t v1    = ecmCfg.ctCfg[idxCT].vChan1;
      int_fast8_t v2    = ecmCfg.ctCfg[idxCT].vChan2;
      int32_t     thisV = vSampleBuffer[idxInject].smpV[v1];
      int32_t     lastV = vSampleBuffer[idxLast].smpV[v1];

      accumCollecting->processCT[idxCT].sumPA[0] += (int64_t)(thisCT * lastV);
      accumCollecting->processCT[idxCT].sumPB[0] += (int64_t)(thisCT * thisV);
      accumCollecting->processCT[idxCT].sumI_sqr += (int64_t)(thisCT * thisCT);
      accumCollecting->processCT[idxCT].sumI_deltas += thisCT;

      /* L-L load */
      if (v1 != v2) {
        thisV = vSampleBuffer[idxInject].smpV[v2];
        lastV = vSampleBuffer[idxLast].smpV[v2];
        accumCollecting->processCT[idxCT].sumPA[1] += (int64_t)(thisCT * lastV);
        accumCollecting->processCT[idxCT].sumPB[1] += (int64_t)(thisCT * thisV);
      }
    }
  }

  /* Flag if there has been a (-) -> (+) crossing, always on V1. Check for
   * zero-crossing, swap buffers and pend event.
   */
  uint32_t timeNow_us = (ecmCfg.timeMicros != 0) ? (*ecmCfg.timeMicros)() : 0;
  if (zeroCrossingSW(smpSet.smpV[0], timeNow_us)) {

    zcFlag   = true;
    t_ZClast = timeNow_us;

    if (0 == discardCycles) {
      accumCollecting->cycles++;
    } else {
      discardCycles--;
      if (0 == discardCycles) {
        accumSwapClear();
        accumCollecting->tStart_us = (*ecmCfg.timeMicros)();
      }
    }
  }

  /* If no zero-crossing has been detected in 100 ms, fall back to assumed
   * Vrms (or time-based reporting if assumedVrms not configured) */
  if ((*ecmCfg.timeMicrosDelta)(t_ZClast) > 100000) {
    useAssumedV = true;
    /* Force discard phase to complete if stuck waiting for valid crossings */
    if (discardCycles > 0) {
      discardCycles = 0;
      accumSwapClear();
      accumCollecting->tStart_us = (*ecmCfg.timeMicros)();
    }
  } else {
    useAssumedV = false;
  }

  uint32_t tRepLastDelta = (*ecmCfg.timeMicrosDelta)(t_RepLast);

  /* Flag one second before the report is due to allow slow sensors to
   * sample. For example, DS18B20 requires 750 ms to sample.
   */
  bool pend1sNoVAC =
      useAssumedV && (tRepLastDelta > ecmCfg.reportTime_us - 1E6);
  bool pend1sCycles =
      accumCollecting->cycles == (ecmCfg.reportCycles - ecmCfg.mainsFreq);

  pend1s = pend1sNoVAC || pend1sCycles;

  /* Trigger conditions
   *   - Number of detected cycles
   *   - Report time if no V AC sensed
   *   - A manual trigger (sync'd to cycle if VAC present)
   */
  bool repCycles  = (accumCollecting->cycles >= ecmCfg.reportCycles);
  bool repTime    = useAssumedV && (tRepLastDelta > ecmCfg.reportTime_us);
  bool repTrigger = processTrigger && (useAssumedV || zcFlag);

  if (repCycles || repTime || repTrigger) {
    accumSwapClear();

    accumCollecting->tStart_us = (*ecmCfg.timeMicros)();
    accumProcessing->tDelta_us =
        (*ecmCfg.timeMicrosDelta)(accumProcessing->tStart_us);

    t_RepLast      = accumCollecting->tStart_us;
    processTrigger = false;
    reportReady    = true;
  }

  perfActive->numSlices++;
  perfActive->microsSlices += (*ecmCfg.timeMicrosDelta)(t_start);

  /* Advance injection point, masking for overflow */
  idxInject = (idxInject + 1u) & (PROC_DEPTH - 1u);

  return reportReady ? ECM_REPORT_COMPLETE
                     : (pend1s ? ECM_PEND_1S : ECM_CYCLE_ONGOING);
}

ECMPerformance_t *ecmPerformance(void) {
  swapPtr((void **)&perfActive, (void **)&perfIdle);
  memset(perfActive, 0, sizeof(*perfActive));

  return perfIdle;
}

RAMFUNC ECMDataset_t *ecmProcessSet(void) {
  uint32_t  t_start = 0;
  CalcRMS_t rms;

  t_start = (*ecmCfg.timeMicros)();

  /* Reused constants */
  const int     numSamples    = accumProcessing->numSamples;
  const int64_t numSamplesSqr = numSamples * numSamples;
  rms.numSamples              = numSamples;

  /* Choose if using the assumed time, or cycle locked real time */
  const uint32_t t_dividend =
      useAssumedV ? ecmCfg.reportTime_us : accumProcessing->tDelta_us;
  const float timeTotal = qfp_fdiv(qfp_uint2float(t_dividend), 1000000.0f);
  datasetProc.wallTime  = timeTotal;

  for (int_fast8_t idxV = 0; idxV < NUM_V; idxV++) {
    if (channelActive[idxV]) {
      rms.cal    = ecmCfg.vCfg[idxV].voltageCal;
      rms.sDelta = accumProcessing->processV[idxV].sumV_deltas;
      rms.sSqr   = accumProcessing->processV[idxV].sumV_sqr;

      float voltage = useAssumedV ? ecmCfg.assumedVrms : calcRMS(&rms);

      /* Check if signal amplitude is sufficient (not just noise).
       * Threshold: ~0.5V indicates no sensor connected (just ADC noise) */
      if (!useAssumedV && (voltage < 0.5f)) {
        voltage = 0.0f;
      }

      datasetProc.rmsV[idxV] = voltage;
    } else {
      datasetProc.rmsV[idxV] = 0.0f;
    }
  }

  if (threePhase) {
    for (int_fast8_t i = 0; i < 3; i++) {
      rms.cal    = ecmCfg.vCfg[i].voltageCal;
      rms.sDelta = accumProcessing->processV[i + NUM_V].sumV_deltas;
      rms.sSqr   = accumProcessing->processV[i + NUM_V].sumV_sqr;

      float voltage = useAssumedV ? ecmCfg.assumedVrms : calcRMS(&rms);

      /* Check if signal amplitude is sufficient (not just noise) */
      if (!useAssumedV && (voltage < 0.5f)) {
        voltage = 0.0f;
      }

      datasetProc.rmsV[i + NUM_V] = voltage;
    }
  }

  for (int_fast8_t idxCT = 0; idxCT < NUM_CT; idxCT++) {
    if (channelActive[idxCT + NUM_V]) {
      int idxV1 = ecmCfg.ctCfg[idxCT].vChan1;
      int idxV2 = ecmCfg.ctCfg[idxCT].vChan2;

      // RMS Current
      rms.cal    = ecmCfg.ctCfg[idxCT].ctCal;
      rms.sDelta = accumProcessing->processCT[idxCT].sumI_deltas;
      rms.sSqr   = accumProcessing->processCT[idxCT].sumI_sqr;
      datasetProc.CT[idxCT].rmsI = calcRMS(&rms);

      // Power and energy
      float sumEnergy = qfp_fadd(
          (qfp_fmul(qfp_int642float(accumProcessing->processCT[idxCT].sumPA[0]),
                    ecmCfg.ctCfg[idxCT].phaseX)),
          (qfp_fmul(qfp_int642float(accumProcessing->processCT[idxCT].sumPB[0]),
                    ecmCfg.ctCfg[idxCT].phaseY)));

      int vi_offset = rms.sDelta * accumProcessing->processV[idxV1].sumV_deltas;

      float powerNow;
      if (useAssumedV) {
        powerNow = qfp_fmul(datasetProc.CT[idxCT].rmsI, ecmCfg.assumedVrms);
      } else {
        powerNow = qfp_fdiv(sumEnergy, qfp_int2float(numSamples));
        powerNow = qfp_fsub(powerNow, qfp_fdiv(qfp_int2float(vi_offset),
                                               qfp_int642float(numSamplesSqr)));
        powerNow = qfp_fmul(powerNow,
                            qfp_fmul(rms.cal, ecmCfg.vCfg[idxV1].voltageCal));
      }

      if (idxV1 != idxV2) {
        sumEnergy = qfp_fadd(
            (qfp_fmul(
                qfp_int642float(accumProcessing->processCT[idxCT].sumPA[1]),
                ecmCfg.ctCfg[idxCT].phaseX)),
            (qfp_fmul(
                qfp_int642float(accumProcessing->processCT[idxCT].sumPB[1]),
                ecmCfg.ctCfg[idxCT].phaseY)));

        vi_offset = rms.sDelta * accumProcessing->processV[idxV2].sumV_deltas;
        float powerNow2 = qfp_fdiv(sumEnergy, qfp_int2float(numSamples));
        powerNow2 =
            qfp_fsub(powerNow2, qfp_fdiv(qfp_int2float(vi_offset),
                                         qfp_int642float(numSamplesSqr)));
        powerNow2 = qfp_fmul(powerNow2,
                             qfp_fmul(rms.cal, ecmCfg.vCfg[idxV2].voltageCal));
        powerNow  = qfp_fsub(powerNow, powerNow2);
      }

      // Power factor
      float rmsV;
      if (useAssumedV) {
        rmsV = ecmCfg.assumedVrms;
      } else {
        if (idxV1 == idxV2) {
          rmsV = datasetProc.rmsV[idxV1];
        } else {
          if (0 == idxV1) {
            if (1 == idxV2) {
              /* V1-V2 */
              rmsV = datasetProc.rmsV[3];
            } else {
              /* V1-V3 */
              rmsV = datasetProc.rmsV[4];
            }
          } else {
            /* V2-V3 */
            rmsV = datasetProc.rmsV[5];
          }
        }
      }

      float VA   = qfp_fmul(datasetProc.CT[idxCT].rmsI, rmsV);
      float pf   = qfp_fdiv(powerNow, VA);
      bool  pf_b = ((pf > 1.05f) || (pf < -1.05f) || (pf != pf));

      datasetProc.CT[idxCT].pf = pf_b ? 0.0f : pf;

      // Energy and power, rounding to nearest integer
      datasetProc.CT[idxCT].realPower =
          qfp_float2int_z(qfp_fadd(powerNow, 0.5f));
      datasetProc.CT[idxCT].apparentPower = qfp_float2int_z(qfp_fadd(VA, 0.5f));

      // REVISIT : Consider double precision here, some truncation observed
      float energyNow = qfp_fmul(powerNow, timeTotal);
      energyNow       = qfp_fadd(energyNow, residualEnergy[idxCT]);
      int whNow       = qfp_float2int_z(qfp_fdiv(energyNow, 3600.0f));

      datasetProc.CT[idxCT].wattHour += whNow;
      residualEnergy[idxCT] = qfp_fsub(energyNow, qfp_int2float(whNow * 3600));

    } else {
      /* Zero all values otherwise */
      memset(&datasetProc.CT[idxCT], 0, sizeof(*datasetProc.CT));
    }
  }

  perfActive->numCycles++;
  perfActive->microsCycles += (*ecmCfg.timeMicrosDelta)(t_start);

  return &datasetProc;
}

void ecmProcessSetTrigger(void) { processTrigger = true; }
