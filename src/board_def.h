#pragma once

/* SAMD uses Arm Cortex-M0+ or Cortex-M4F - can place fast functions into RAM
 * to avoid the penalty of loading from flash with wait states.
 */
#define RAMFUNC __attribute__((section(".ramfunc")))

/* Clock frequencies
 *  - Core is on the 48 MHz DFLL
 *  - Peripherals are on the OSC8M / 8 -> 1 MHz
 */
#define F_CORE      48000000ul
#define F_PERIPH    8000000ul
#define F_TIMER_ADC F_PERIPH / 8
#define F_TIMER2    F_PERIPH / 8

#define BOOTPROT_SAMD 0x2 /* 8KB bootloader protection, Table 22-2 */

#define NUM_V              3
#define NUM_CT             12
#define VCT_TOTAL          (NUM_V + NUM_CT)
#define CT_RES             1 /* Number of CT channels reserved for future use */
#define SAMPLE_RATE        4800
#define SAMPLES_IN_SET     2
#define SAMPLE_BUF_DEPTH   2
#define OVERSAMPLING_RATIO 2

#define ADC_VREF     1.024f
#define ADC_RES_BITS 11
#define CAL_V        8.087f
#define CAL_CT       3.0f

/* OneWire/Pulse setup */
#define NUM_OPA   2
#define PULSE_RES 1 /* Number of Pulse channels reserved */

/* Temperature sensors
 * This is the maximum number of OneWire DS18B20 sensors that can be used
 */
#define TEMP_MAX_ONEWIRE 8

/* EEPROM */
/* Top of EEPROM address, not including R/W bit */
#define EEPROM_BASE_ADDR   0x50
/* Maximum number of bytes in a single page */
#define EEPROM_PAGE_SIZE   16u
/* Worst case EEPROM write time (microseconds) */
#define EEPROM_WR_TIME     5000ul
/* Size of configuration area */
#define EEPROM_CONFIG_SIZE 256
/* Size of the EEPROM in bytes */
#define EEPROM_SIZE        1024
/* Offset of wear levelled area */
#define EEPROM_WL_OFFSET   (EEPROM_CONFIG_SIZE)

/* Serial Communication Instances */

#define SERCOM_SPI      SERCOM2
#define SERCOM_I2CM_EXT SERCOM3
#define SERCOM_I2CM     SERCOM4
#define SERCOM_UART     SERCOM5

#define SERCOM_SPI_APBCMASK      PM_APBCMASK_SERCOM2
#define SERCOM_I2CM_EXT_APBCMASK PM_APBCMASK_SERCOM3
#define SERCOM_I2CM_INT_APBCMASK PM_APBCMASK_SERCOM4
#define SERCOM_UART_APBCMASK     PM_APBCMASK_SERCOM5

#define SERCOM_SPI_GCLK_ID      SERCOM2_GCLK_ID_CORE
#define SERCOM_I2CM_EXT_GCLK_ID SERCOM3_GCLK_ID_CORE
#define SERCOM_I2CM_INT_GCLK_ID SERCOM4_GCLK_ID_CORE
#define SERCOM_UART_GCLK_ID     SERCOM5_GCLK_ID_CORE

#define SERCOM_UART_DMAC_ID_TX SERCOM5_DMAC_ID_TX

#define SERCOM_UART_INTERACTIVE_HANDLER irq_handler_sercom5()
#define SERCOM_UART_INTERACTIVE         SERCOM5

#define SERCOM_UART_NVIC_IRQn        SERCOM5_IRQn
#define SERCOM_UART_INTERACTIVE_IRQn SERCOM5_IRQn

/* Timer Instances */

/* TIMER_ADC triggers ADC conversion, TIMER_DELAY used for delay timing.
 * TIMER_DELAY must be a 32 bit timer. For SAMD21, TC4 is combined with TC5
 * (30.6.2.4) to build a 32 bit timer.
 *
 * There is also accurate millisecond/microsecond counters. The 1ms tick is
 * also used to wake up the core for events and USB handling.
 */
#define TIMER_ADC       TC3
#define TIMER_DELAY     TC4
#define TIMER_TICK      TC6
#define IRQ_TIMER_DELAY irq_handler_tc4
#define IRQ_TIMER_TICK  irq_handler_tc6

#define TIMER_ADC_GCLK_ID  TC3_GCLK_ID
#define TIMER_ADC_APBCMASK PM_APBCMASK_TC3
#define TIMER_ADC_EVT_SRC  EVSYS_ID_GEN_TC3_MCX_0

#define TIMER_DELAY_GCLK_ID  TC4_GCLK_ID
#define TIMER_DELAY_APBCMASK PM_APBCMASK_TC4
#define TIMER_DELAY_IRQn     TC4_IRQn

#define TIMER_TICK_GCLK_ID  TC6_GCLK_ID
#define TIMER_TICK_APBCMASK PM_APBCMASK_TC6
#define TIMER_TICK_IRQn     TC6_IRQn

/* Pin Configuration (nb. logical, not physical) */
#define GRP_PINA 0u
#define GRP_PINB 1u

/* Revision information */
#define GRP_REV0 GRP_PINA
#define GRP_REV1 GRP_PINA
#define GRP_REV2 GRP_PINB
#define PIN_REV0 27u
#define PIN_REV1 28u
#define PIN_REV2 17u

/* LEDs */
#define GRP_LED_STATUS GRP_PINB
#define PIN_LED_STATUS 22u
#define GRP_LED_PROG   GRP_PINB
#define PIN_LED_PROG   23u

/* OneWire/Pulse interface */
#define GRP_OPA     GRP_PINA
#define PIN_OPA1    16
#define PIN_OPA2    17
#define PIN_OPA1_PU 18
#define PIN_OPA2_PU 19

/* DISABLE_EXT_INTF */
#define GRP_DISABLE_EXT GRP_PINB
#define PIN_DISABLE_EXT 16u

/* ADC Pins */
#define GRP_ADC_VMID   GRP_PINA
#define PIN_ADC_VMID   2u
#define GRP_ADC_VREF   GRP_PINA
#define PIN_ADC_VREF   3u
#define GRP_ADC_VSENS1 GRP_PINB
#define PIN_ADC_VSENS1 8u
#define GRP_ADC_VSENS2 GRP_PINB
#define PIN_ADC_VSENS2 9u
#define GRP_ADC_VSENS3 GRP_PINA
#define PIN_ADC_VSENS3 4u
#define GRP_ADC_CT1    GRP_PINB
#define PIN_ADC_CT1    0u
#define GRP_ADC_CT2    GRP_PINB
#define PIN_ADC_CT2    1u
#define GRP_ADC_CT3    GRP_PINB
#define PIN_ADC_CT3    4u
#define GRP_ADC_CT4    GRP_PINA
#define PIN_ADC_CT4    6u
#define GRP_ADC_CT5    GRP_PINA
#define PIN_ADC_CT5    7u
#define GRP_ADC_CT6    GRP_PINA
#define PIN_ADC_CT6    8u
#define GRP_ADC_CT7    GRP_PINB
#define PIN_ADC_CT7    2u
#define GRP_ADC_CT8    GRP_PINB
#define PIN_ADC_CT8    3u
#define GRP_ADC_CT9    GRP_PINB
#define PIN_ADC_CT9    5u
#define GRP_ADC_CT10   GRP_PINB
#define PIN_ADC_CT10   6u
#define GRP_ADC_CT11   GRP_PINB
#define PIN_ADC_CT11   7u
#define GRP_ADC_CT12   GRP_PINA
#define PIN_ADC_CT12   5u
#define GRP_ADC_AIN    GRP_PINA
#define PIN_ADC_AIN    9u
#define GRP_ADC_VCAL_H GRP_PINA
#define PIN_ADC_VCAL_H 10u
#define GRP_ADC_VCAL_L GRP_PINA
#define PIN_ADC_VCAL_L 11u
#define AIN_VCAL_L     ADC_INPUTCTRL_MUXPOS_PIN19
#define AIN_VCAL_H     ADC_INPUTCTRL_MUXPOS_PIN18

/* USB */
#define GRP_USB_DM 0
#define PIN_USB_DM 24
#define GRP_USB_DP 0
#define PIN_USB_DP 25
#define PMUX_USB   PORT_PMUX_PMUXE_G

/* UART related defines */
#define PMUX_UART       PORT_PMUX_PMUXE_C /* SERCOM */
#define GRP_SERCOM_UART GRP_PINA
#define PIN_UART_TX     20u
#define PIN_UART_RX     21u
#define UART_PAD_RX     3u /* RXPO */
#define UART_PAD_TX     1u /* TXPO value, Tx on pad 2 */
#define UART_BAUD       115200u

/* RFM related defines */
#define RFM_PALEVEL_DEF 0x19 /* Safe level if no antenna installed. */
#define RFM_FREQ_DEF    3    /* 433.92 MHz in frequency enum */
#define GRP_SERCOM_SPI  GRP_PINA
#define PIN_SPI_MISO    12u
#define PIN_SPI_SCK     13u
#define PIN_SPI_MOSI    15u
#define PIN_SPI_RFM_SS  14u
#define SPI_BAUD        4000000ul
#define PMUX_SPI        PORT_PMUX_PMUXE_C
#define GRP_RFM_INTF    GRP_PINB
#define PIN_RFM_IRQ     14u
#define PMUX_RFM_IRQ    PORT_PMUX_PMUXE_A
#define PIN_RFM_RST     15u
#define RFM_RETRIES     4
#define RFM_TIMEOUT     30

/* I2C related defines */
#define GRP_SERCOM_I2C_INT GRP_PINB
#define PIN_I2C_INT_SDA    12u
#define PIN_I2C_INT_SCL    13u
#define PMUX_I2CM_INT      PORT_PMUX_PMUXE_C

#define GRP_SERCOM_I2C_EXT GRP_PINA
#define PIN_I2C_EXT_SDA    22u
#define PIN_I2C_EXT_SCL    23u
#define PMUX_I2CM_EXT      PORT_PMUX_PMUXE_C

/* DMA defines */
#define NUM_CHAN_DMA  3u
#define DMA_CHAN_UART 2u
#define DMA_CHAN_ADC1 1u
#define DMA_CHAN_ADC0 0u
