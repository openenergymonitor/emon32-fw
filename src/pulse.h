#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum PulseEdge_ {
  PULSE_EDGE_RISING  = 'r',
  PULSE_EDGE_FALLING = 'f',
  PULSE_EDGE_BOTH    = 'b'
} PulseEdge_t;

typedef struct PulseCfg_ {
  PulseEdge_t edge;    /* Edge or edges to detect */
  uint8_t     grp;     /* GPIO group */
  uint8_t     pin;     /* GPIO pin */
  uint32_t    periods; /* Blank period */
  bool        active;  /* Channel active  */
  bool        puEn;    /* Pull up enabled */
} PulseCfg_t;

/*! @brief Returns a pointer to the pulse counter configuration
 *  @param [in] index : index of the pulse counter to access.
 *  @return pointer to configuration struct. 0 for failure
 */
PulseCfg_t *pulseGetCfg(const size_t index);

/*! Initialise a configured pulse counter
 *  @param [in] index : pulse counter index
 */
void pulseInit(const size_t index);

/*! @brief Update the pulse counter(s) */
void pulseUpdate(void);

/*! @brief Sets the pulse count value
 *  @param [in] index : pulse count index to set
 *  @param [in] pulseCount : the value to set
 */
void pulseSetCount(const size_t index, const uint32_t value);

/*! @brief Get the current pulse count value
 *  @return current pulse value
 */
uint32_t pulseGetCount(const size_t index);
