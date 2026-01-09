#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Types */
typedef struct Pin_ {
  uint8_t grp;
  uint8_t pin;
} Pin_t;

typedef enum PINDIR_ { PIN_DIR_IN, PIN_DIR_OUT } PINDIR_t;

typedef enum PINCFG_ { PIN_CFG_SET, PIN_CFG_CLR } PINCFG_t;

typedef enum PINDRV_ { PIN_DRV_CLR, PIN_DRV_SET, PIN_DRV_TGL } PINDRV_t;

/*! @note Group corresponds to the A, B, .. mapping. A -> 0, B -> 1, .. */

/*! @brief Sets the pin configuration
 *  @param [in] grp : Group number
 *  @param [in] pin : Pin number
 *  @param [in] cfg : configuration option
 *  @param [in] cs  : clear or set configuration
 */
void portPinCfg(const uint8_t grp, const uint8_t pin, const uint8_t cfg,
                const PINCFG_t cs);

/*! @brief Sets a pin as input or output
 *  @param [in] grp : Group number
 *  @param [in] pin : PIN number
 *  @param [in] mode: PIN_DIR_IN for input, PIN_DIR_OUT for output
 */
void portPinDir(const uint8_t grp, const uint8_t pin, const PINDIR_t mode);

/*! @brief Sets the pin driver value
 *  @param [in] grp : Group number
 *  @param [in] pin : Pin number
 *  @param [in] drv : Clear, set, or toggle pin
 */
void portPinDrv(const uint8_t grp, const uint8_t pin, const PINDRV_t drv);

/*! @brief Sets the mux for pin alternate function
 *  @param [in] grp : Group number
 *  @param [in] pin : Pin number
 *  @param [in] mux : Mux mode
 */
void portPinMux(const uint8_t grp, const uint8_t pin, const uint8_t mux);

/*! @brief Clear the mux for pin alternate function
 *  @param [in] grp : Group number
 *  @param [in] pin : Pin number
 */
void portPinMuxClear(const uint8_t grp, const uint8_t pin);

/*! @brief Returns the pin value
 *  @param [in] grp : Group number
 *  @param [in] pin : Pin number
 */
bool portPinValue(const uint8_t grp, const uint8_t pin);

/*! @brief   Configure the ports.
 *           Ports for peripherals are configured in their setup functions
 */
void portSetup(void);
