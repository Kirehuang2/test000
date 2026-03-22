/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = gpt_counter_overflow_isr, /* GPT6 COUNTER OVERFLOW (Overflow) */
            [1] = adc_b_calend0_isr, /* ADC CALEND0 (End of calibration of A/D converter unit 0) */
            [2] = adc_b_calend1_isr, /* ADC CALEND1 (End of calibration of A/D converter unit 1) */
            [3] = adc_b_adi0_isr, /* ADC ADI0 (End of A/D scanning operation(Gr.0)) */
            [4] = gpt_counter_overflow_isr, /* GPT0 COUNTER OVERFLOW (Overflow) */
            [5] = sci_b_uart_rxi_isr, /* SCI3 RXI (Receive data full) */
            [6] = sci_b_uart_txi_isr, /* SCI3 TXI (Transmit data empty) */
            [7] = sci_b_uart_tei_isr, /* SCI3 TEI (Transmit end) */
            [8] = sci_b_uart_eri_isr, /* SCI3 ERI (Receive error) */
            [9] = spi_b_rxi_isr, /* SPI1 RXI (Receive buffer full) */
            [10] = spi_b_txi_isr, /* SPI1 TXI (Transmit buffer empty) */
            [11] = spi_b_tei_isr, /* SPI1 TEI (Transmission complete event) */
            [12] = spi_b_eri_isr, /* SPI1 ERI (Error) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_GPT6_COUNTER_OVERFLOW,GROUP0), /* GPT6 COUNTER OVERFLOW (Overflow) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_ADC_CALEND0,GROUP1), /* ADC CALEND0 (End of calibration of A/D converter unit 0) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_ADC_CALEND1,GROUP2), /* ADC CALEND1 (End of calibration of A/D converter unit 1) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_ADC_ADI0,GROUP3), /* ADC ADI0 (End of A/D scanning operation(Gr.0)) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_GPT0_COUNTER_OVERFLOW,GROUP4), /* GPT0 COUNTER OVERFLOW (Overflow) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI3_RXI,GROUP5), /* SCI3 RXI (Receive data full) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI3_TXI,GROUP6), /* SCI3 TXI (Transmit data empty) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI3_TEI,GROUP7), /* SCI3 TEI (Transmit end) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SCI3_ERI,GROUP0), /* SCI3 ERI (Receive error) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SPI1_RXI,GROUP1), /* SPI1 RXI (Receive buffer full) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SPI1_TXI,GROUP2), /* SPI1 TXI (Transmit buffer empty) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SPI1_TEI,GROUP3), /* SPI1 TEI (Transmission complete event) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_SPI1_ERI,GROUP4), /* SPI1 ERI (Error) */
        };
        #endif
        #endif
