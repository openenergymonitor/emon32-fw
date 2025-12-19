#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "emon32.h"
#include "emon_CM.h"
#include "emon_CM_coeffs.h"

#define MAINS_FREQ  50.0
#define REPORT_CT   3 /* Number of CT channels to report */
#define REPORT_TIME 9.8f
#define REPORT_V    1 /* Number of V channels to report */
#define SMP_TICK    1000000u / SAMPLE_RATE / (VCT_TOTAL)
#define TEST_TIME   100E6 /* Time to run in microseconds */
#define VRMS_GOLD   240.0f
#define MAX_A       (1 << (ADC_RES_BITS - 1))

typedef struct wave_ {
  double omega;  /* Angular velocity */
  double phi;    /* Phase (rad) */
  double s;      /* Scale (0 < s <= 1.0) */
  int    offset; /* Constant offset, clamped if outside range */
} wave_t;

typedef struct noise_ {
  bool   en;    /* Noise enabled */
  double mu;    /* mean */
  double sigma; /* std dev */
  double alpha; /* skew */
} noise_t;

/*! @brief Check the results from a run
 *  @param [in] pData : pointer to dataset
 *  @param [in] pF : gold power factor
 *  @return true for success, false otherwise
 */
static bool checkDataset(ECMDataset_t *pData, float pF);

/*! @brief Convert a current in CT to a wave description
 *  @param [in] IRMS : RMS current
 *  @param [in] scaleCT : current to produce 333 mV RMS output
 *  @param [in] phase : CT phase (degrees)
 *  @param [out] pW  : pointer to the wave struct
 */
static void currentToWave(double IRMS, int scaleCT, double phase, wave_t *w);

static void dynamicRun(int reports, bool prtReport, noise_t *noise, bool noVAC);

/*! @brief Generates a Q11 [-1024, 1023] wave with configurable parameters
 *  @param [in] w       : pointer to wave information
 *  @param [in] tMicros : time in microseconds
 */
static q15_t generateWave(wave_t *w, int tMicros);

/*! @brief Generate number from normal distribution
 *  @param [in] noise : pointer to noise struct
 *  @return random number from normal distribution
 */
static double randNormal(noise_t *noise);

/*! @brief Generate number from skew normal distribution
 *  @param [in] noise : pointer to noise struct
 *  @return random number from skew normal distribution
 */
static double randSkewNormal(noise_t *noise);

/*! @brief Print the output from the emonCM report
 *  @param [in] reportNum   Report number
 *  @param [in] tick        Simulation tick in us
 *  @param [in] pDataset    Pointer to the dataset
 */
static void printReport(int reportNum, int64_t tick, ECMDataset_t *pDataset);

/*! @brief Convert a voltage into a wave description
 *  @param [in] vRMS : RMS voltage
 *  @param [out] pW  : pointer to the wave struct
 */
static void voltageToWave(double vRMS, wave_t *w);

static uint32_t tick = 0;
ECM_STATUS_t    status;
SampleSet_t     smpProc;
unsigned int    smpIdx = 0;
ECMDataset_t   *dataset;

volatile RawSampleSetPacked_t *volatile smpRaw[2];
wave_t wave[VCT_TOTAL];

static uint32_t timeMicros(void) { return tick; }

static uint32_t timeMicrosDelta(uint32_t tickPrev) {
  uint32_t delta         = 0;
  uint32_t timeMicrosNow = tick;

  /* Check for wrap (every ~1 h) */
  if (tickPrev > timeMicrosNow) {
    delta = (UINT32_MAX - tickPrev) + timeMicrosNow + 1u;
  } else {
    delta = timeMicrosNow - tickPrev;
  }

  return delta;
}

static bool checkDataset(ECMDataset_t *pData, float pF) {

  if (pF != 0.0f) {
    if ((pData->CT[0].pf > (pF + 0.01f)) || (pData->CT[0].pf < (pF - 0.01f))) {
      printf("\nPF Gold: %.2f Test: %.2f\n", pF, pData->CT[0].pf);
      return false;
    }
  }

  if ((pData->rmsV[0] > (VRMS_GOLD + 1.0f)) ||
      (pData->rmsV[0] < (VRMS_GOLD - 1.0f))) {
    printf("\nVrms Gold: %.2f Test: %.2f\n", VRMS_GOLD, pData->rmsV[0]);
    return false;
  }
  return true;
}

static void dynamicRun(int reports, bool prtReport, noise_t *noise,
                       bool noVAC) {
  int reportNum = 0;

  while (reportNum < reports) {
    for (int j = 0; j < 2; j++) {
      for (int i = 0; i < VCT_TOTAL; i++) {
        if (noVAC && (i < NUM_V)) {
          smpRaw[smpIdx]->samples[j].smp[i] = 0;
        } else {
          smpRaw[smpIdx]->samples[j].smp[i] = generateWave(&wave[i], tick);
          smpRaw[smpIdx]->samples[j].smp[i] +=
              (noise->en ? (noise->alpha == 0.0) ? (int)randNormal(noise)
                                                 : (int)randSkewNormal(noise)
                         : 0);
        }
        tick += SMP_TICK;
      }
    }

    smpIdx = !smpIdx;
    ecmDataBufferSwap();

    if (ECM_REPORT_COMPLETE == ecmInjectSample()) {
      dataset = ecmProcessSet();
      if (prtReport) {
        printReport(reportNum, tick, dataset);
      }
      reportNum++;
    }
  }
  ecmFlush();
}

/* usage: cm.test mu sigma alpha */
int main(int argc, char *argv[]) {

  FILE     *fptr;
  ECMCfg_t *pEcmCfg;
  noise_t   noise = {0};

  /* Collect noise parameters*/
  if (argc > 1) {
    noise.en = true;
    noise.mu = atof(argv[1]);
  }
  if (argc > 2) {
    noise.sigma = atof(argv[2]);
  }
  if (argc > 3) {
    noise.alpha = atof(argv[3]);
  }

  srandom(time(NULL));
  /* Copy and fold the half band coefficients */
  const int lutDepth = (numCoeffUnique - 1) * 2;
  int16_t  *coeffLut = malloc(lutDepth * sizeof(int16_t));
  assert(coeffLut);

  for (int i = 0; i < (lutDepth / 2); i++) {
    coeffLut[i]                  = firCoeffs[i];
    coeffLut[(lutDepth - 1 - i)] = firCoeffs[i];
  }

  /* Set all waves to 50 Hz, all CTs to 5 deg offset. The maximum amplitude
   * corresponds to 1.024 V at the emon32 input.
   */
  for (int i = 0; i < NUM_V; i++) {
    voltageToWave(230.0, &wave[i]);
    wave[i].phi = M_PI * 120 * i / 180;
  }

  /* Set CTs 1-3 as 3.5 A, 4-12 active but zero current */
  for (int i = NUM_V; i < VCT_TOTAL; i++) {
    if (i - NUM_V < 3) {
      currentToWave(3.5, 5, 5.0, &wave[i]);
    } else {
      currentToWave(0, 5, 5.0, &wave[i]);
    }
  }

  pEcmCfg = ecmConfigGet();

  /* ecmDataBuffer returns a pointer to the buffer which the DMA is putting
   * data into.
   */
  memset(&smpProc, 0, sizeof(smpProc));
  smpRaw[0] = ecmDataBuffer();
  ecmDataBufferSwap();
  smpRaw[1] = ecmDataBuffer();
  ecmDataBufferSwap();

  pEcmCfg->downsample   = 1u;
  pEcmCfg->reportCycles = (unsigned int)(REPORT_TIME * MAINS_FREQ);
  pEcmCfg->mainsFreq    = 50;
  pEcmCfg->reportTime_us =
      (1000000 / pEcmCfg->mainsFreq) * pEcmCfg->reportCycles;
  pEcmCfg->assumedVrms     = 240;
  pEcmCfg->samplePeriod    = 13;
  pEcmCfg->timeMicros      = &timeMicros;
  pEcmCfg->timeMicrosDelta = &timeMicrosDelta;

  for (int i = 0; i < NUM_V; i++) {
    pEcmCfg->vCfg[i].voltageCalRaw = 100.0f;
    pEcmCfg->vCfg[i].vActive       = (i == 0);
  }

  for (int i = 0; i < NUM_CT; i++) {
    pEcmCfg->ctCfg[i].active   = (i < 6);
    pEcmCfg->ctCfg[i].ctCalRaw = 20.0f;
    pEcmCfg->ctCfg[i].phCal    = 4.2f;
    pEcmCfg->ctCfg[i].vChan1   = 0;
    pEcmCfg->ctCfg[i].vChan2   = 0;
  }

  pEcmCfg->correction.valid  = true;
  pEcmCfg->correction.offset = 0;
  pEcmCfg->correction.gain   = (1 << 11);

  for (int i = 0; i < NUM_CT; i++) {
    pEcmCfg->mapCTLog[i] = i;
  }

  ecmConfigInit();

  printf("---- emon32 CM test ----\n\n");

  /* Sanity check by dumping a CSV of 10 cycles @ mains freq */
  printf("  Generating test sine...");
  fptr = fopen("cm-test-sine.csv", "w");
  if (!fptr) {
    printf("Failed\n  Failed to open output\n");
    return 1;
  }
  for (int t = 0; t < ((1000000 * 10) / MAINS_FREQ);
       t += (SMP_TICK * (VCT_TOTAL))) {
    q15_t a = generateWave(&wave[0], t);
    fprintf(fptr, "%d,%d\n", t, a);
  }
  fclose(fptr);
  printf(" Done!\n\n");

  /* Print out test information */
  printf("  Test configuration:\n");
  printf("    - Number of V     : %d\n", NUM_V);
  printf("    - Number of CT    : %d\n", NUM_CT);
  printf("    - Mains frequency : %.0f Hz\n", MAINS_FREQ);
  printf("    - DSP enabled     : %s\n", pEcmCfg->downsample ? "Yes" : "No");
  printf("    - Report time     : %.2f s\n", REPORT_TIME);
  printf("    - Sample tick     : %d us\n", SMP_TICK);
  printf("    - Assumed voltage : %.2f V\n", pEcmCfg->assumedVrms);
  printf("    - Noise           : %s\n", noise.en ? "Yes" : "No");
  if (noise.en) {
    printf("                        mu    : %f\n", noise.mu);
    printf("                        sigma : %f\n", noise.sigma);
    printf("                        alpha : %f\n", noise.alpha);
  }
  printf("\n");

  /* ============ START : HALF BAND TEST ============ */
  /* Reference : https://dspguru.com/dsp/faqs/fir/implementation/ */

  /* IMPULSE TEST
   * Inject an implulse, should get all the coefficients (except middle) out
   * TODO parameterise this for any size of filter
   */
  if (pEcmCfg->downsample) {
    printf("  Half band filter tests:\n");
    printf("    - Impulse: ");

    for (unsigned int i = 0; i < VCT_TOTAL; i++) {
      smpRaw[smpIdx]->samples[0].smp[i] = 0;
      smpRaw[smpIdx]->samples[1].smp[i] = INT16_MAX;
    }

    ecmDataBufferSwap();
    ecmFilterSample(&smpProc);
    if (coeffLut[0] != smpProc.smpV[0]) {
      printf("\nsmpRaw[smpIdx]->samples[0]: %d\n",
             smpRaw[smpIdx]->samples[0].smp[0]);
      printf("smpRaw[smpIdx]->samples[1]: %d\n",
             smpRaw[smpIdx]->samples[1].smp[0]);
      printf("smpProc.smpV[0]: %d\n", smpProc.smpV[0]);

      printf("Gold: %d Test: %d\n", coeffLut[0], smpProc.smpV[0]);
      return 1;
    }

    for (unsigned int i = 0; i < VCT_TOTAL; i++) {
      smpRaw[smpIdx]->samples[0].smp[0] = 0;
      smpRaw[smpIdx]->samples[1].smp[0] = 0;
    }
    for (unsigned int idxCoeff = 0; idxCoeff < 9u; idxCoeff++) {
      ecmFilterSample(&smpProc);
      if (!(coeffLut[idxCoeff + 1u] == smpProc.smpV[0])) {
        printf("\nGold: %d Test: %d\n", coeffLut[idxCoeff + 1u],
               smpProc.smpV[0]);
        return 1;
      }
    }
    printf("Complete\n\n");
  }

  /* Increment through the sample channels (2x for oversampling)
   * and generate the wave for each channel at each point.
   */
  printf("  Dynamic tests...\n\n");

  printf("    - Phase 0°, PF = 1 ... ");
  dynamicRun(4, false, &noise, false);
  if (!checkDataset(dataset, 1.0f)) {
    return 1;
  }
  printf("Done!\n");

  printf("    - Phase 90°, PF = 0 ... ");
  wave[NUM_V].phi = M_PI / 2;
  tick            = 0;
  dynamicRun(4, false, &noise, false);
  if (!checkDataset(dataset, 0.0f)) {
    return 1;
  }
  printf("Done!\n");

  printf("    - Phase 180°, PF = -1 ... ");
  wave[NUM_V].phi = M_PI;
  tick            = 0;
  dynamicRun(4, false, &noise, false);
  if (!checkDataset(dataset, -1.0f)) {
    return 1;
  }
  printf("Done!\n\n");

  printf("    - No V AC ... ");
  wave[NUM_V].phi = M_PI * 4.2f / 180;
  tick            = 0;
  dynamicRun(4, false, &noise, true);
  if (!checkDataset(dataset, 1.0f)) {
    return 1;
  }
  printf("Done!\n\n");

  printf("    - 600 s report period ... ");
  fflush(stdout);
  pEcmCfg->reportCycles = 600 * 50;
  wave[NUM_V].phi       = M_PI * 4.2f / 180;
  tick                  = 0;
  dynamicRun(1, false, &noise, false);
  checkDataset(dataset, 1.0f);
  printf("Done!\n");

  printf("    - 0.5 s report period ... ");
  pEcmCfg->reportCycles = 25;
  tick                  = 0;
  dynamicRun(1, false, &noise, false);
  checkDataset(dataset, 1.0f);
  printf("Done!\n");

  printf("\n  Finished!\n\n");

  return 0;
}

static void currentToWave(double IRMS, int scaleCT, double phase, wave_t *w) {
  double iPk = IRMS * sqrt(2);
  w->offset  = 0;
  w->omega   = 2 * M_PI * MAINS_FREQ;
  w->phi     = M_PI * phase / 180;
  w->s       = iPk / scaleCT;
}

static q15_t generateWave(wave_t *w, int tMicros) {
  assert((w->s >= 0.0) && (w->s <= 1.0));
  q15_t  wave;
  double a = sin(((w->omega * tMicros) / 1000000.0) + w->phi) * w->s;
  wave     = (q15_t)(a * MAX_A);
  wave += w->offset;

  /* Clip if the offset exceeds the bounds */
  if (wave < -MAX_A) {
    wave = -MAX_A;
  } else if (wave > (MAX_A - 1)) {
    wave = (MAX_A - 1);
  }
  return wave;
}

static void printReport(int reportNum, int64_t tick, ECMDataset_t *pDataset) {
  const int  ct    = 0;
  static int prevE = 0;
  int        thisE;

  thisE = pDataset->CT[ct].wattHour;
  printf("\n    Report %d (t = %.2f s):\r\n", reportNum++, (tick / 1000000.0));
  printf("      Vrms (V) : %.2f\r\n", pDataset->rmsV[0]);
  printf("      Irms (A) : %.2f\r\n", pDataset->CT[ct].rmsI);
  printf("      P    (W) : %d\r\n", pDataset->CT[ct].realPower);
  printf("      E    (Wh): %d (delta: %d)\r\n", thisE, (thisE - prevE));
  printf("      pF       : %.2f\r\n", pDataset->CT[ct].pf);
  prevE = thisE;
}

static double randNormal(noise_t *noise) {
  double x = (double)random() / RAND_MAX;
  double y = (double)random() / RAND_MAX;
  return (noise->mu + noise->sigma * sqrt(-2 * log(x)) * cos(2 * M_PI * y));
}

static double randSkewNormal(noise_t *noise) {
  double sigma = noise->alpha / sqrt(1.0 + noise->alpha * noise->alpha);
  double u0    = randNormal(&(noise_t){.mu = 0, .sigma = 1});
  double v     = randNormal(&(noise_t){.mu = 0, .sigma = 1});
  double u1    = sigma * u0 + sqrt(1.0 - sigma * sigma) * v;
  if (u0 >= 0) {
    return u1 + noise->mu;
  } else {
    return (-u1 + noise->mu);
  }
}

static void voltageToWave(double vRMS, wave_t *w) {
  double vPk = vRMS * sqrt(2);
  w->offset  = 0;
  w->omega   = 2 * M_PI * MAINS_FREQ;
  w->phi     = 0;
  w->s       = vPk / (400.0 * 1.003);
}
