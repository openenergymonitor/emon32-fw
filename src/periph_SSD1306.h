#pragma once

#include "emon32_samd.h"

typedef enum SSD1306_Status_ { SSD1306_SUCCESS, SSD1306_FAIL } SSD1306_Status_t;

typedef struct PosXY_ {
  uint32_t x;
  uint32_t y;
} PosXY_t;

/*! @brief Indicate if the SSD1306 has been found and initialised
 *  @return true if active, false otherwise.
 */
bool ssd1306Active(void);

/*! @brief Clear the SSD1306 buffer */
void ssd1306ClearBuffer(void);

/*! @brief Turn the display off */
SSD1306_Status_t ssd1306DisplayOff(void);

/*! @brief Place the buffer contents onto the I2C bus */
SSD1306_Status_t ssd1306DisplayUpdate(void);

/*! @brief Draw a string at the current buffer position */
SSD1306_Status_t ssd1306DrawString(const char *s);

/*! @brief Initialise the SSD1306 OLED display
 *  @param [in] pSercomI2C : SERCOM instance to use
 *  @return status of the initialisation
 */
SSD1306_Status_t ssd1306Init(Sercom *pSercomI2C);

/*! @brief Set the position in the buffer
 *  @param [in] pos : X-Y coordinations
 */
void ssd1306SetPosition(const PosXY_t pos);
