#include <string.h>

#include "driver_SERCOM.h"
#include "emon32_assert.h"
#include "emon32_samd.h"
#include "periph_SSD1306.h"

/* SSD1306 definitions */
#define CHARS_COLS_LENGTH 5u
#define PAGE_ADDR_END     7u /* 8 lines, 3u for 128x32 display -> 4 lines */
#define COL_ADDR_END      127u
#define LINE_MEM_SIZE     (PAGE_ADDR_END + 1u) * (COL_ADDR_END + 1u)
#define MAX_X             COL_ADDR_END
#define MAX_Y             (PAGE_ADDR_END + 1u) * 8u

#define SSD1306_NUM_INIT_CMDS 26u
#define SSD1306_ADDR          0x3Cu

#define SSD1306_COMMAND 0x80u /* Continuation bit=1, D/C=0; 1000 0000 */
#define SSD1306_COMMAND_STREAM                                                 \
  0x00u                           /* Continuation bit=0, D/C=0; 0000 0000      \
                                   */
#define SSD1306_DATA        0xC0u /* Continuation bit=1, D/C=1; 1100 0000 */
#define SSD1306_DATA_STREAM 0x40u /* Continuation bit=0, D/C=1; 0100 0000 */

#define SSD1306_SET_MUX_RATIO                                                  \
  0xA8u /* Set MUX ratio to N+1 MUX, N=A[5:0] : from 16MUX to 64MUX */
#define SSD1306_DISPLAY_OFFSET 0xD3u /* Set Display Offset */
#define SSD1306_DISPLAY_ON     0xAFu /* Display ON in normal mode */
#define SSD1306_DISPLAY_OFF    0xAEu /* Display OFF (sleep mode) */
#define SSD1306_DIS_ENT_DISP_ON                                                \
  0xA4u /* Entire Display ON, Output ignores RAM content */
#define SSD1306_DIS_IGNORE_RAM                                                 \
  0xA5u /* Resume to RAM content display, Output follows RAM content */
#define SSD1306_DIS_NORMAL                                                     \
  0xA6u /* Normal display, 0 in RAM: OFF in display panel, 1 in RAM: ON in     \
           display panel */
#define SSD1306_DIS_INVERSE                                                    \
  0xA7u /* Inverse display, 0 in RAM: ON in display panel, 1 in RAM: OFF in    \
           display panel */
#define SSD1306_DEACT_SCROLL                                                   \
  0x2Eu /* Stop scrolling that is configured by command 26h/27h/29h/2Ah */
#define SSD1306_ACTIVE_SCROLL                                                  \
  0x2Fu /* Start scrolling that is configured by the scrolling setup           \
           commands:26h/27h/29h/2Ah */
#define SSD1306_SET_START_LINE   0x40u /* Set Display Start Line */
#define SSD1306_MEMORY_ADDR_MODE 0x20u /* Set Memory, Addressing Mode */
#define SSD1306_SET_COLUMN_ADDR  0x21u /* Set Column Address */
#define SSD1306_SET_PAGE_ADDR    0x22u /* Set Page Address */
#define SSD1306_SEG_REMAP                                                      \
  0xA0u /* Set Segment Re-map, X[0]=0b column address 0 is mapped to SEG0 */
#define SSD1306_SEG_REMAP_OP                                                   \
  0xA1u /* Set Segment Re-map, X[0]=1b: column address 127 is mapped to SEG0   \
         */
#define SSD1306_COM_SCAN_DIR                                                   \
  0xC0u /* Set COM Output, X[3]=0b: normal mode (RESET) Scan from COM0 to      \
           COM[N –1], e N is the Multiplex ratio */
#define SSD1306_COM_SCAN_DIR_OP                                                \
  0xC8u /* Set COM Output, X[3]=1b: remapped mode. Scan from COM[N-1] to COM0, \
           e N is the Multiplex ratio */
#define SSD1306_COM_PIN_CONF                                                   \
  0xDAu /* Set COM Pins Hardware Configuration,                                \
         * A[4]=0b, Sequential COM pin configuration, A[4]=1b(RESET),          \
         * Alternative COM pin configuration A[5]=0b(RESET), Disable COM       \
         * Left/Right remap, A[5]=1b, Enable COM Left/Right remap */
#define SSD1306_SET_CONTRAST                                                   \
  0x81u /* Set Contrast Control, Double byte command to select 1 to 256        \
           contrast steps, increases as the value increases */
#define SSD1306_SET_OSC_FREQ                                                   \
  0xD5u /* Set Display Clock Divide Ratio/Oscillator Frequency                 \
         * A[3:0] : Define the divide ratio (D) of the  display clocks (DCLK): \
         * Divide ratio= A[3:0] + 1, RESET is 0000b (divide ratio = 1) A[7:4]  \
         * : Set the Oscillator Frequency, FOSC. Oscillator Frequency          \
         * increases with the value of A[7:4] and vice versa. RESET is 1000b   \
         */
#define SSD1306_SET_CHAR_REG                                                   \
  0x8Du /* Charge Pump Setting, A[2] = 0b, Disable charge pump(RESET), A[2] =  \
         * 1b, Enable charge pump during display on The Charge Pump must be    \
         * enabled by the following command: 8Dh ; Charge Pump Setting 14h ;   \
         * Enable Charge Pump AFh; Display ON */
#define SSD1306_SET_PRECHARGE 0xD9u /* Set Pre-charge Period */
#define SSD1306_VCOM_DESELECT 0xDBu /* Set VCOMH Deselect Leve */
#define SSD1306_NOP           0xE3u /* No operation */
#define SSD1306_RESET                                                          \
  0xE4u /* Maybe SW RESET, @source                                             \
           https://github.com/SmingHub/Sming/issues/501 */

static SSD1306_Status_t bufUpdatePos();
static SSD1306_Status_t drawChar(const char c);
static bool             ssd1306I2CActivate(void);

static int32_t displayFound;

/* Font definition */
static const uint8_t FONTS[][CHARS_COLS_LENGTH] = {
    {0x00u, 0x00u, 0x00u, 0x00u, 0x00u}, /* 20 space */
    {0x81u, 0x81u, 0x18u, 0x81u, 0x81u}, /* 21 ! */
    {0x00u, 0x07u, 0x00u, 0x07u, 0x00u}, /* 22 " */
    {0x14u, 0x7fu, 0x14u, 0x7fu, 0x14u}, /* 23 # */
    {0x24u, 0x2au, 0x7fu, 0x2au, 0x12u}, /* 24 $ */
    {0x23u, 0x13u, 0x08u, 0x64u, 0x62u}, /* 25 % */
    {0x36u, 0x49u, 0x55u, 0x22u, 0x50u}, /* 26 & */
    {0x00u, 0x05u, 0x03u, 0x00u, 0x00u}, /* 27 ' */
    {0x00u, 0x1cu, 0x22u, 0x41u, 0x00u}, /* 28 ( */
    {0x00u, 0x41u, 0x22u, 0x1cu, 0x00u}, /* 29 ) */
    {0x14u, 0x08u, 0x3eu, 0x08u, 0x14u}, /* 2a * */
    {0x08u, 0x08u, 0x3eu, 0x08u, 0x08u}, /* 2b + */
    {0x00u, 0x50u, 0x30u, 0x00u, 0x00u}, /* 2c , */
    {0x08u, 0x08u, 0x08u, 0x08u, 0x08u}, /* 2d - */
    {0x00u, 0x60u, 0x60u, 0x00u, 0x00u}, /* 2e . */
    {0x20u, 0x10u, 0x08u, 0x04u, 0x02u}, /* 2f / */
    {0x3eu, 0x51u, 0x49u, 0x45u, 0x3eu}, /* 30 0 */
    {0x00u, 0x42u, 0x7fu, 0x40u, 0x00u}, /* 31 1 */
    {0x42u, 0x61u, 0x51u, 0x49u, 0x46u}, /* 32 2 */
    {0x21u, 0x41u, 0x45u, 0x4bu, 0x31u}, /* 33 3 */
    {0x18u, 0x14u, 0x12u, 0x7fu, 0x10u}, /* 34 4 */
    {0x27u, 0x45u, 0x45u, 0x45u, 0x39u}, /* 35 5 */
    {0x3cu, 0x4au, 0x49u, 0x49u, 0x30u}, /* 36 6 */
    {0x01u, 0x71u, 0x09u, 0x05u, 0x03u}, /* 37 7 */
    {0x36u, 0x49u, 0x49u, 0x49u, 0x36u}, /* 38 8 */
    {0x06u, 0x49u, 0x49u, 0x29u, 0x1eu}, /* 39 9 */
    {0x00u, 0x36u, 0x36u, 0x00u, 0x00u}, /* 3a : */
    {0x00u, 0x56u, 0x36u, 0x00u, 0x00u}, /* 3b ; */
    {0x08u, 0x14u, 0x22u, 0x41u, 0x00u}, /* 3c < */
    {0x14u, 0x14u, 0x14u, 0x14u, 0x14u}, /* 3d = */
    {0x00u, 0x41u, 0x22u, 0x14u, 0x08u}, /* 3e > */
    {0x02u, 0x01u, 0x51u, 0x09u, 0x06u}, /* 3f ? */
    {0x32u, 0x49u, 0x79u, 0x41u, 0x3eu}, /* 40 @ */
    {0x7eu, 0x11u, 0x11u, 0x11u, 0x7eu}, /* 41 A */
    {0x7fu, 0x49u, 0x49u, 0x49u, 0x36u}, /* 42 B */
    {0x3eu, 0x41u, 0x41u, 0x41u, 0x22u}, /* 43 C */
    {0x7fu, 0x41u, 0x41u, 0x22u, 0x1cu}, /* 44 D */
    {0x7fu, 0x49u, 0x49u, 0x49u, 0x41u}, /* 45 E */
    {0x7fu, 0x09u, 0x09u, 0x09u, 0x01u}, /* 46 F */
    {0x3eu, 0x41u, 0x49u, 0x49u, 0x7au}, /* 47 G */
    {0x7fu, 0x08u, 0x08u, 0x08u, 0x7fu}, /* 48 H */
    {0x00u, 0x41u, 0x7fu, 0x41u, 0x00u}, /* 49 I */
    {0x20u, 0x40u, 0x41u, 0x3fu, 0x01u}, /* 4a J */
    {0x7fu, 0x08u, 0x14u, 0x22u, 0x41u}, /* 4b K */
    {0x7fu, 0x40u, 0x40u, 0x40u, 0x40u}, /* 4c L */
    {0x7fu, 0x02u, 0x0cu, 0x02u, 0x7fu}, /* 4d M */
    {0x7fu, 0x04u, 0x08u, 0x10u, 0x7fu}, /* 4e N */
    {0x3eu, 0x41u, 0x41u, 0x41u, 0x3eu}, /* 4f O */
    {0x7fu, 0x09u, 0x09u, 0x09u, 0x06u}, /* 50 P */
    {0x3eu, 0x41u, 0x51u, 0x21u, 0x5eu}, /* 51 Q */
    {0x7fu, 0x09u, 0x19u, 0x29u, 0x46u}, /* 52 R */
    {0x46u, 0x49u, 0x49u, 0x49u, 0x31u}, /* 53 S */
    {0x01u, 0x01u, 0x7fu, 0x01u, 0x01u}, /* 54 T */
    {0x3fu, 0x40u, 0x40u, 0x40u, 0x3fu}, /* 55 U */
    {0x1fu, 0x20u, 0x40u, 0x20u, 0x1fu}, /* 56 V */
    {0x3fu, 0x40u, 0x38u, 0x40u, 0x3fu}, /* 57 W */
    {0x63u, 0x14u, 0x08u, 0x14u, 0x63u}, /* 58 X */
    {0x07u, 0x08u, 0x70u, 0x08u, 0x07u}, /* 59 Y */
    {0x61u, 0x51u, 0x49u, 0x45u, 0x43u}, /* 5a Z */
    {0x00u, 0x7fu, 0x41u, 0x41u, 0x00u}, /* 5b [ */
    {0x02u, 0x04u, 0x08u, 0x10u, 0x20u}, /* 5c / */
    {0x00u, 0x41u, 0x41u, 0x7fu, 0x00u}, /* 5d ] */
    {0x04u, 0x02u, 0x01u, 0x02u, 0x04u}, /* 5e ^ */
    {0x40u, 0x40u, 0x40u, 0x40u, 0x40u}, /* 5f _ */
    {0x00u, 0x01u, 0x02u, 0x04u, 0x00u}, /* 60 ` */
    {0x20u, 0x54u, 0x54u, 0x54u, 0x78u}, /* 61 a */
    {0x7fu, 0x48u, 0x44u, 0x44u, 0x38u}, /* 62 b */
    {0x38u, 0x44u, 0x44u, 0x44u, 0x20u}, /* 63 c */
    {0x38u, 0x44u, 0x44u, 0x48u, 0x7fu}, /* 64 d */
    {0x38u, 0x54u, 0x54u, 0x54u, 0x18u}, /* 65 e */
    {0x08u, 0x7eu, 0x09u, 0x01u, 0x02u}, /* 66 f */
    {0x0cu, 0x52u, 0x52u, 0x52u, 0x3eu}, /* 67 g */
    {0x7fu, 0x08u, 0x04u, 0x04u, 0x78u}, /* 68 h */
    {0x00u, 0x44u, 0x7du, 0x40u, 0x00u}, /* 69 i */
    {0x20u, 0x40u, 0x44u, 0x3du, 0x00u}, /* 6a j */
    {0x7fu, 0x10u, 0x28u, 0x44u, 0x00u}, /* 6b k */
    {0x00u, 0x41u, 0x7fu, 0x40u, 0x00u}, /* 6c l */
    {0x7cu, 0x04u, 0x18u, 0x04u, 0x78u}, /* 6d m */
    {0x7cu, 0x08u, 0x04u, 0x04u, 0x78u}, /* 6e n */
    {0x38u, 0x44u, 0x44u, 0x44u, 0x38u}, /* 6f o */
    {0x7cu, 0x14u, 0x14u, 0x14u, 0x08u}, /* 70 p */
    {0x08u, 0x14u, 0x14u, 0x14u, 0x7cu}, /* 71 q */
    {0x7cu, 0x08u, 0x04u, 0x04u, 0x08u}, /* 72 r */
    {0x48u, 0x54u, 0x54u, 0x54u, 0x20u}, /* 73 s */
    {0x04u, 0x3fu, 0x44u, 0x40u, 0x20u}, /* 74 t */
    {0x3cu, 0x40u, 0x40u, 0x20u, 0x7cu}, /* 75 u */
    {0x1cu, 0x20u, 0x40u, 0x20u, 0x1cu}, /* 76 v */
    {0x3cu, 0x40u, 0x30u, 0x40u, 0x3cu}, /* 77 w */
    {0x44u, 0x28u, 0x10u, 0x28u, 0x44u}, /* 78 x */
    {0x0cu, 0x50u, 0x50u, 0x50u, 0x3cu}, /* 79 y */
    {0x44u, 0x64u, 0x54u, 0x4cu, 0x44u}, /* 7a z */
    {0x00u, 0x08u, 0x36u, 0x41u, 0x00u}, /* 7b { */
    {0x00u, 0x00u, 0x7fu, 0x00u, 0x00u}, /* 7c | */
    {0x00u, 0x41u, 0x36u, 0x08u, 0x00u}, /* 7d } */
    {0x10u, 0x08u, 0x08u, 0x10u, 0x08u}, /* 7e ~ */
    {0x00u, 0x00u, 0x00u, 0x00u, 0x00u}  /* 7f */
};

/*! @var pSercom : pointer to the SERCOM instance */
static Sercom *pSercom;

/*! @var lineBuffer : one line buffer */
static uint8_t  lineBuffer[LINE_MEM_SIZE];
static uint32_t posBuf = 0;

static SSD1306_Status_t bufUpdatePos(void) {
  uint32_t y     = posBuf >> 7;
  uint32_t x     = posBuf - (y << 7);
  uint32_t x_nxt = x + CHARS_COLS_LENGTH + 1u;

  if (x_nxt > COL_ADDR_END) {
    if (y > PAGE_ADDR_END) {
      return SSD1306_FAIL;
    } else if (y < (PAGE_ADDR_END - 1u)) {
      posBuf = (y + 1u) << 7;
    }
  }

  return SSD1306_SUCCESS;
}

static SSD1306_Status_t drawChar(const char c) {
  uint32_t         i      = 0;
  SSD1306_Status_t update = bufUpdatePos();
  if (SSD1306_FAIL == update) {
    return SSD1306_FAIL;
  }

  while (i < CHARS_COLS_LENGTH) {
    lineBuffer[posBuf++] = FONTS[c - 32][i++];
  }
  posBuf++;

  return SSD1306_SUCCESS;
}

static bool ssd1306I2CActivate(void) {
  I2CM_Status_t s = i2cActivate(pSercom, (SSD1306_ADDR << 1));

  if (I2CM_SUCCESS != s) {
    if (I2CM_DISABLED != s) {
      i2cAck(pSercom, I2CM_ACK, I2CM_ACK_CMD_STOP);
    }
    return false;
  }
  return true;
}

void ssd1306ClearBuffer(void) { memset(lineBuffer, 0, LINE_MEM_SIZE); }

SSD1306_Status_t ssd1306DisplayOff(void) {

  if (!ssd1306I2CActivate()) {
    return SSD1306_FAIL;
  }

  if (I2CM_SUCCESS != i2cDataWrite(pSercom, SSD1306_COMMAND)) {
    return SSD1306_FAIL;
  }
  if (I2CM_SUCCESS != i2cDataWrite(pSercom, SSD1306_DISPLAY_OFF)) {
    return SSD1306_FAIL;
  }
  i2cAck(pSercom, I2CM_ACK, I2CM_ACK_CMD_STOP);

  return SSD1306_SUCCESS;
}

SSD1306_Status_t ssd1306DisplayUpdate(void) {

  if (!ssd1306I2CActivate()) {
    return SSD1306_FAIL;
  }

  if (I2CM_SUCCESS != i2cDataWrite(pSercom, SSD1306_DATA_STREAM)) {
    return SSD1306_FAIL;
  }
  for (size_t i = 0; i < LINE_MEM_SIZE; i++) {
    if (I2CM_SUCCESS != i2cDataWrite(pSercom, lineBuffer[i])) {
      return SSD1306_FAIL;
    }
  }
  i2cAck(pSercom, I2CM_ACK, I2CM_ACK_CMD_STOP);

  return SSD1306_SUCCESS;
}

SSD1306_Status_t ssd1306DrawString(const char *s) {
  EMON32_ASSERT(s);

  SSD1306_Status_t ret;

  while (*s) {
    ret = drawChar(*s++);
    if (SSD1306_FAIL == ret) {
      return SSD1306_FAIL;
    }
  }

  return SSD1306_SUCCESS;
}

SSD1306_Status_t ssd1306Init(Sercom *pSercomI2C) {
  EMON32_ASSERT(pSercomI2C);

  const uint8_t initCmds[SSD1306_NUM_INIT_CMDS] = {
      SSD1306_DISPLAY_OFF,
      SSD1306_SET_OSC_FREQ,
      0x80u, /* 0x80 -> D */
      SSD1306_SET_MUX_RATIO,
      0x3Fu, /* 0x1F -> 128x32, 0x3F -> 128x64 */
      SSD1306_DISPLAY_OFFSET,
      0x00u,
      SSD1306_SET_START_LINE,
      SSD1306_SET_CHAR_REG,
      0x14u,
      SSD1306_MEMORY_ADDR_MODE,
      0x00u, /* 0x00 -> horizontal, 0x01 -> vertical
              * 0x02 -> page
              */
      SSD1306_SEG_REMAP_OP,
      SSD1306_COM_SCAN_DIR_OP,
      SSD1306_COM_PIN_CONF,
      0x12u, /* 0x02 -> 128x32, 0x12 -> 128x64 */
      SSD1306_SET_CONTRAST,
      0x7Fu,
      SSD1306_SET_PRECHARGE,
      0xC2u,
      SSD1306_VCOM_DESELECT,
      0x20u,
      SSD1306_DIS_ENT_DISP_ON,
      SSD1306_DIS_NORMAL,
      SSD1306_DEACT_SCROLL,
      SSD1306_DISPLAY_ON};

  pSercom = pSercomI2C;

  if (!ssd1306I2CActivate()) {
    return SSD1306_FAIL;
  }

  for (size_t i = 0; i < SSD1306_NUM_INIT_CMDS; i++) {
    if (I2CM_SUCCESS != i2cDataWrite(pSercom, SSD1306_COMMAND)) {
      return SSD1306_FAIL;
    }
    if (I2CM_SUCCESS != i2cDataWrite(pSercom, initCmds[i])) {
      return SSD1306_FAIL;
    }
  }
  i2cAck(pSercom, I2CM_ACK, I2CM_ACK_CMD_STOP);

  ssd1306ClearBuffer();

  displayFound = 1;
  return SSD1306_SUCCESS;
}

void ssd1306SetPosition(const PosXY_t pos) { posBuf = pos.x + (pos.y << 7u); }
