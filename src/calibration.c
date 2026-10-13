#include <stddef.h>

#include "calibration.h"

#include "emon32.h"
#include "util.h"

#include "printf.h"
#include "qfplib-m0-full.h"

#define CAL_AMPL_AVG   4u /* Number of sample sets for calibration */
#define CAL_PHASE_ITER 8u /* Number of points for phase */

typedef struct CalibResult_ {
  float  mean;   /* Mean amplitude */
  float  target; /* Target amplitude */
  float  newCal; /* New calibration value */
  size_t index;  /* V/CT index */
  bool   isCT;   /* indicates if this is a CT */
} CalibResult_t;

static void calibrationPrintAmplitude(const CalibResult_t *pCal);
static void calibrationPrintPhase(float phi, size_t idx, bool failed);

void calibrationProcess(const ECMDataset_t *pECM, Emon32Config_t *pConfig) {
  CalibConfig_t *calibcfg = configCalibStatus();
  if (!calibcfg->inProgress) {
    return;
  }

  /* Amplitude calibration */
  if (calibcfg->mode == 'a') {
    calibcfg->accum =
        calibcfg->isCT ? qfp_fadd(calibcfg->accum, pECM->CT[calibcfg->ch].rmsI)
                       : qfp_fadd(calibcfg->accum, pECM->rmsV[calibcfg->ch]);
    calibcfg->iter++;

    if (CAL_AMPL_AVG == calibcfg->iter) {
      ECMCfg_t   *ecmCfg = ecmConfigGet();
      const float mean   = qfp_fdiv(calibcfg->accum, (float)CAL_AMPL_AVG);

      if (mean < 0.25f) {
        calibcfg->inProgress = false;
        serialPuts("> Measured input too low, calibration failed.\r\n");
        return;
      }

      const float   ratio  = qfp_fdiv(calibcfg->target, mean);
      CalibResult_t result = {.index  = calibcfg->ch + 1u,
                              .isCT   = calibcfg->isCT,
                              .mean   = mean,
                              .target = calibcfg->target,
                              .newCal = 0.0f};

      if (calibcfg->isCT) {
        const float newCal =
            qfp_fmul(ratio, pConfig->ctCfg[calibcfg->ch].ctCal);
        pConfig->ctCfg[calibcfg->ch].ctCal   = newCal;
        ecmCfg->ctCfg[calibcfg->ch].ctCalRaw = newCal;
        ecmConfigChannel(calibcfg->ch + NUM_V);

        result.newCal = newCal;
        calibrationPrintAmplitude(&result);
      } else {
        const float newCal =
            qfp_fmul(ratio, pConfig->voltageCfg[calibcfg->ch].voltageCal);
        pConfig->voltageCfg[calibcfg->ch].voltageCal = newCal;
        ecmCfg->vCfg[calibcfg->ch].voltageCalRaw     = newCal;
        ecmConfigChannel(calibcfg->ch);

        result.newCal = newCal;
        calibrationPrintAmplitude(&result);
      }
      calibcfg->inProgress = false;
    }
  } else {
    /* Phase calibration */
    ECMCfg_t   *pEcmCfg = ecmConfigGet();
    const float thisPF  = pECM->CT[calibcfg->ch].pf;
    float       phiNxt  = 0.0f;

    if (calibcfg->defer) {
      calibcfg->defer = false;
      return;
    }

    if (2u > calibcfg->iter) {
      calibcfg->defer                   = true;
      calibcfg->phipf[calibcfg->iter].Y = thisPF;

      if (0u == calibcfg->iter) {
        phiNxt = 100.0f;
      } else if (1u == calibcfg->iter) {
        XY_t mc = {0};
        utilLinearFit(2u, calibcfg->phipf, &mc);

        /* Take +/-1° around coarse ~0 PF (c / -m) in 8 x 0.25° increments */
        phiNxt = qfp_fsub(qfp_fdiv(mc.Y, qfp_fmul(mc.X, -1.0f)), 1.0f);
        calibcfg->phipf[0].X = phiNxt;
        for (size_t i = 1; i < CAL_PHASE_ITER; i++) {
          calibcfg->phipf[i].X = qfp_fadd(calibcfg->phipf[i - 1u].X, 0.25f);
        }
      }

      calibcfg->iter++;
    } else {
      /* Linear through points to get phase value */
      if (calibcfg->iter == (CAL_PHASE_ITER + 1u)) {
        XY_t mc     = {0};
        bool failed = (bool)utilLinearFit(CAL_PHASE_ITER, calibcfg->phipf, &mc);

        if (!failed) {
          /* Setting Y(PF) = 0, solve for X(PHI) = c / -m */
          phiNxt = qfp_fsub(qfp_fdiv(mc.Y, qfp_fmul(mc.X, -1.0f)), 90.0f);
        } else {
          /* Restore previous value if failed */
          phiNxt = pConfig->ctCfg[calibcfg->ch].phase;
        }

        /* Restore original data log period. */
        pConfig->baseCfg.reportTime = calibcfg->prevDatalog;
        configUpdateDatalog(calibcfg->prevDatalog);

        calibcfg->inProgress = false;
        calibrationPrintPhase(phiNxt, calibcfg->ch, failed);
      } else {
        calibcfg->phipf[calibcfg->iter - 2u].Y = thisPF;

        calibcfg->iter++;
        phiNxt = calibcfg->phipf[calibcfg->iter - 2u].X;
      }
    }

    pEcmCfg->ctCfg[calibcfg->ch].phCal = phiNxt;
    ecmConfigChannel(calibcfg->ch + NUM_V);
  }
}

bool calibrationNoTransmit(void) {
  const CalibConfig_t *calibcfg = configCalibStatus();
  return calibcfg->inProgress && (calibcfg->mode == 'p');
}

static void calibrationPrintAmplitude(const CalibResult_t *pCal) {
  printf_("> Finished calibration for %s%d.\r\n", (pCal->isCT ? "CT" : "V"),
          pCal->index);
  printf_("  - Measured %sRMS  : ", (pCal->isCT ? "I" : "V"));
  putFloat(pCal->mean, 0);
  printf_(" %s.\r\n", (pCal->isCT ? "A" : "V"));
  printf_("  - Actual %sRMS  : ", (pCal->isCT ? "I" : "V"));
  putFloat(pCal->target, 0);
  printf_(" %s.\r\n", (pCal->isCT ? "A" : "V"));
  serialPuts("  - New calibration: ");
  putFloat(pCal->newCal, 0);
  serialPuts(".\r\n");
}

static void calibrationPrintPhase(const float phi, const size_t idx,
                                  const bool failed) {
  if (!failed) {
    printf_("> Finished phase calibration for CT%d.\r\n", idx + 1u);
    serialPuts("  - New phase calibration: ");
    putFloat(phi, 0);
    serialPuts("\r\n");
  } else {
    printf_("> Failed to find phase calibration for CT%d\r\n.", idx + 1u);
    serialPuts("  - Restored original phase value.\r\n");
  }
}
