/* generated configuration header file - do not edit */
#ifndef BSP_PIN_CFG_H_
#define BSP_PIN_CFG_H_
#include "r_ioport.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

#define DI_IN1 (BSP_IO_PORT_10_PIN_05) /* DI_IN1 */
#define nFAULT (BSP_IO_PORT_11_PIN_00) /* nFAULT検出 */
#define EN_GATE_RESET (BSP_IO_PORT_13_PIN_05) /* RMW TX接続 ENgate */
#define LATCH_OFF (BSP_IO_PORT_13_PIN_07) /* LATCH_OFF_88P */
#define DI_IN2 (BSP_IO_PORT_13_PIN_10) /* DI_IN2 */
#define DI_PON (BSP_IO_PORT_14_PIN_14) /* P_ON */

extern const ioport_cfg_t g_bsp_pin_cfg; /* R7FA6T2AD3CFP.pincfg */

void BSP_PinConfigSecurityInit();

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER
#endif /* BSP_PIN_CFG_H_ */
