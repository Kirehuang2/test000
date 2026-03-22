// Closed-loop FOC control with Hall sensor (Open-loop control removed)

#include "hal_data.h"
#include "FOCCurrentControl.h"
#include "FOCSpeedControl.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "r_three_phase_api.h"
#include "assist_app_interface.h"

unsigned char Enable = 0;

float SpeedRef_RPM = 0.0f;
float SpeedFb_RPM = 0.0f;
float SpeedRefFinal_PU = 0.0f;
uint16_t Iab[2] = {0,0};
uint16_t Iab_offset[2] = {2048, 2048};  // Default offset for 12-bit ADC current sensors
uint16_t QuadCounts;
float Vabc_out[3] = {0.0f, 0.0f, 0.0f};

// Hall sensor data (obtained from API)
float f_get_angle = 0.0f;               // Hall sensor angle [rad]
float f_get_speed = 0.0f;               // Hall sensor speed [rad/s]
float f_get_phase_err = 0.0f;           // Phase error

// Closed-loop control variables
float IdqRef[2] = {0,0};
float SpeedFb_PU = 0.0F;
float SpeedFb_Hall_PU = 0.0F;  // Hall sensor speed (filtered, for speed control)
unsigned char EnCl = 0;
float IqFb = 0.0F;
// Motor diagnostic: running average and RMS of Iq
volatile float iq_avg = 0.0f;         // Running average [PU]
volatile float iq_rms = 0.0f;         // Running RMS [PU]
volatile float iq_sum = 0.0f;
volatile float iq_sq_sum = 0.0f;
volatile uint32_t iq_avg_count = 0;
float SpeedRefIn_PU = 0.0F;
float Mode = 2.0f;  // Mode >= 2 required to start speed control state machine

// Speed control ramp variables (global for debugger access)
volatile float speed_ref_ramp = 0.0f;
volatile uint8_t ramp_complete = 0;

// ============================================================
// Improved speed estimation from angle derivative
// Instead of using f_get_speed (updates only at Hall edges),
// compute speed as d(angle)/dt using a sliding window over
// the interpolated f_get_angle from FSP Hall module.
// ============================================================
#define SPEED_EST_WINDOW  100  // Window size in calls (100 = 10ms at 10kHz)
static float angle_history[SPEED_EST_WINDOW];  // Ring buffer of unwrapped angles
static uint16_t angle_hist_idx = 0;
static uint8_t angle_hist_filled = 0;
static float prev_angle_for_unwrap = 0.0f;
static float angle_accumulated = 0.0f;         // Monotonic unwrapped angle [rad electrical]

// Debug variables for improved speed estimation
volatile float debug_speed_from_angle = 0.0f;  // Speed from angle derivative [rad/s mechanical]
volatile float debug_angle_unwrapped = 0.0f;   // Accumulated unwrapped angle [rad]
volatile float debug_f_get_speed_raw = 0.0f;   // Raw FSP speed for comparison

// Speed filter coefficient (tunable via debugger)
volatile float speed_filter_alpha = 0.10f;      // 0.03=heavy, 0.10=default, 1.0=no filter

// ============================================================
// Control mode switching (changeable via debugger)
// 0 = Speed control: SpeedControl_step manages IdqRef
// 1 = Torque control: direct Iq command, bypass speed loop
// ============================================================
volatile uint8_t control_mode = 0;              // 0=Speed, 1=Torque
volatile float torque_ref_iq = 0.0f;            // Direct Iq command for torque mode [PU]
volatile float torque_ref_iq_max = 0.3f;        // Iq clamp for torque mode [PU] (was 0.194)

// Debug counters
volatile uint32_t adc_callback_count = 0;
volatile uint32_t timer_callback_count = 0;

// ADC callback rate measurement (read adc_rate_result in debugger after 2+ sec)
static uint32_t adc_count_at_1s = 0;        // snapshot at timer=1000
volatile uint32_t adc_rate_result = 0;       // callbacks per second (10000=10kHz, 20000=20kHz)

// V_dc voltage compensation
#define V_DC_NOMINAL  18.0f              // Nominal voltage the PU system is calibrated for
volatile uint8_t vdc_compensation_en = 1;  // 0=disabled, 1=enabled (debugger toggle)

// Debug: Group 1 FIFO status
volatile fsp_err_t debug_err_fifo2 = 0;   // 0=OK, 12=UNDERFLOW(FIFO empty)
volatile uint8_t debug_fifo2_count = 0;    // Number of FIFO entries read

// Debug: Direct ADDR register read (bypass FIFO)
volatile uint16_t debug_addr3 = 0;  // VIRT_CH_3 = AN012/PC00
volatile uint16_t debug_addr4 = 0;  // VIRT_CH_4 = AN013/PC01
volatile uint16_t debug_addr5 = 0;  // VIRT_CH_5 = AN014/PC02
volatile fsp_err_t debug_err_read3 = 0;

// Debug: FIFO entry dump (all 6 entries from Group 2)
volatile uint16_t debug_fifo2_data[8] = {0};   // .data field (ADC value)
volatile uint8_t  debug_fifo2_ch[8] = {0};     // .physical_channel field
volatile uint8_t  debug_fifo2_err[8] = {0};    // .err field

// Debug: Group 0 FIFO data for comparison (current sensors should read ~2048 at zero)
volatile uint16_t debug_fifo0_data[4] = {0};
volatile uint8_t  debug_fifo0_ch[4] = {0};

// Debug: Raw 32-bit FIFO register value (bypass bitfield struct)
volatile uint32_t debug_fifo2_raw[8] = {0};

// Debug variables for sign investigation
volatile float debug_speed_error = 0.0f;      // SpeedRef - SpeedFb (should be positive if below target)
volatile float debug_Ialpha = 0.0f;           // Clarke transform output
volatile float debug_Ibeta = 0.0f;            // Clarke transform output
volatile uint8_t debug_encl_smooth = 0;       // 0=Open-loop, 1=Closed-loop enabled
volatile float debug_abs_speed_error = 0.0f;  // Absolute speed error for transition check

// ============================================================
// Data Logger - Ring Buffer for debugging
// ============================================================
#define LOG_SIZE 500  // 500 samples = 5 seconds at 100Hz

typedef struct {
    float SpeedRef;
    float SpeedFb;
    float IqFb;
    float IdqRef1;      // Iq reference
    uint8_t EnCl;       // Closed-loop enable
    uint8_t ramp_phase; // ramp_complete value
} LogEntry_t;

volatile LogEntry_t log_buffer[LOG_SIZE];
volatile uint16_t log_index = 0;
volatile uint8_t log_running = 1;   // 1=Recording, 0=Stopped
volatile uint16_t log_counter = 0;  // Counter for decimation

// Debug variables for PWM output
volatile uint32_t debug_duty[3] = {0, 0, 0};
volatile float debug_vabc[3] = {0.0f, 0.0f, 0.0f};
// DRV8302 fault monitoring
volatile uint8_t drv8302_fault = 0;       // 0=Normal, 1=Fault detected
volatile uint8_t drv8302_fault_latched = 0; // Latched fault (requires power cycle to clear)

// Hall sensor angle offset (adjustable via debugger)
// Note: Optimal value may vary depending on startup position
volatile float hall_angle_offset = 0.0f;  // Start with 0, adjust via debugger

// Power switch control variables
static uint32_t power_switch_high_counter = 0;  // Counter for PE14 high duration (in ms)
static uint8_t shutdown_requested = 0;          // Shutdown latch (once set, never cleared)

// Debug variables for power switch (volatile for debugger visibility)
volatile uint8_t debug_pe14_state = 0;          // Current state of PE14 input (0=Low, 1=High)
volatile uint32_t debug_pe14_counter = 0;       // PE14 high duration counter (ms)
volatile uint8_t debug_shutdown_requested = 0;  // Shutdown state (0=normal, 1=shutting down)

// ============================================================
// LED output pin definitions (Active Low via SN74LVC07APW)
// MCU Low → LED ON, MCU High → LED OFF
// ============================================================
#define LED_POWER_GREEN  (BSP_IO_PORT_13_PIN_15)  // OUT1: Power LED Green
#define LED_POWER_RED    (BSP_IO_PORT_12_PIN_06)  // OUT2: Power LED Red
#define LED_TURBO        (BSP_IO_PORT_12_PIN_07)  // OUT3: Turbo LED
#define LED_ASSIST_H     (BSP_IO_PORT_13_PIN_01)  // OUT4: Assist Level High (Red)
#define LED_ASSIST_M     (BSP_IO_PORT_13_PIN_02)  // OUT5: Assist Level Mid (Green)
#define LED_ASSIST_L     (BSP_IO_PORT_13_PIN_03)  // OUT6: Assist Level Low (Green)

// ============================================================
// Monitoring variables (volatile for debugger visibility)
// ============================================================
// Battery voltage monitoring (AN1 - via voltage divider)
volatile float battery_voltage = 0.0f;       // Battery voltage [V]
volatile uint16_t battery_adc_raw = 0;       // Raw ADC value

// Temperature monitoring
volatile float temp_inverter = 25.0f;        // Inverter temperature [°C] (AN2)
volatile float temp_regen = 25.0f;           // Regen circuit temperature [°C] (AN3)
volatile uint16_t temp_inv_adc_raw = 0;      // Raw ADC value
volatile uint16_t temp_regen_adc_raw = 0;    // Raw ADC value

// Protection state
volatile uint8_t protection_state = 0;       // 0=Normal, 1=Warning, 2=Derating, 3=Emergency stop
volatile float output_derating = 1.0f;       // Output limit factor (0.0~1.0)

// Assist app interface (Simulink model I/O)
AssistApp_ExtU_T assist_U = {0};
AssistApp_ExtY_T assist_Y = {0};
volatile int32_t hall_enc_count = 0;          // Hall cumulative position count

// Assist level and turbo mode
volatile uint8_t assist_level = 0;           // 0=Low, 1=Mid, 2=High
volatile uint8_t turbo_mode = 0;             // 0=Off, 1=On

// Switch debounce counters
static uint8_t sw_in1_last = 1;              // Previous state of IN1 (pull-up, idle=High)
static uint8_t sw_in2_last = 1;              // Previous state of IN2
static uint16_t sw_in1_debounce = 0;         // Debounce counter for IN1
static uint16_t sw_in2_debounce = 0;         // Debounce counter for IN2
#define SW_DEBOUNCE_MS 30                    // 30ms debounce time

// NTC Thermistor parameters (NTCG104LH223HT1: 22kΩ NTC, β≈4530K)
// Circuit: 3.3V → NTC(pull-up) → ADC node → 10kΩ(pull-down) → GND
// V_adc = 3.3V * R_pulldown / (R_ntc + R_pulldown)
// R_ntc = R_pulldown * (4095 - ADC) / ADC
#define NTC_R0        22000.0f               // NTC resistance at T0 [Ω] (NTCG104LH223HT1)
#define NTC_T0        298.15f                // Reference temperature [K] (25°C)
#define NTC_BETA      4530.0f                // B25/50 parameter [K] (4485-4573K typ)
#define NTC_PULLDOWN  10000.0f               // Pull-down resistor [Ω]

// Battery voltage divider (180kΩ / 15kΩ)
#define VBAT_DIVIDER_RATIO  13.0f            // (180k + 15k) / 15k = 13
#define VBAT_ADC_VREF       3.3f             // ADC reference voltage

// Protection thresholds
#define VBAT_LOW_WARNING    14.0f            // Low battery warning [V]
#define VBAT_LOW_CUTOFF     12.0f            // Low battery cutoff [V]
#define TEMP_WARNING        70.0f            // Temperature warning [°C]
#define TEMP_DERATING       80.0f            // Temperature derating [°C]
#define TEMP_SHUTDOWN       90.0f            // Temperature shutdown [°C]

// ============================================================
// BLE UART Communication (DA14531MOD via SCI3 Hardware UART)
// ============================================================
// DA14531 DataPump: P0_6=TX, P0_5=RX, 57600 baud
// PCB (cross-wired): PE06(TXD3) → P0_5(DA14531 RX)
//                    PE05(RXD3) ← P0_6(DA14531 TX)
// PE02 → P0_0 (DA14531 RST, active low)
// ============================================================
#define BLE_RESET_PIN     (BSP_IO_PORT_14_PIN_02)  // PE02 → DA14531 P0_0/RST

#define UART_RX_BUF_SIZE  128

static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];

static volatile uint16_t uart_rx_idx = 0;
static volatile uint8_t uart_rx_complete = 0;   // 1=line received
static volatile uint8_t uart_tx_busy = 0;       // 1=TX in progress (SCI3)

// TX ring buffer: multiple messages can be queued, main loop drains them
#define TX_QUEUE_SIZE 256          // max bytes per single message (was 128, increased for large packs)
#define TX_RING_SLOTS 8            // number of message slots
static char tx_ring[TX_RING_SLOTS][TX_QUEUE_SIZE];
static volatile uint8_t tx_ring_head = 0;   // next slot to write
static volatile uint8_t tx_ring_tail = 0;   // next slot to send
static volatile uint8_t tx_ring_count = 0;  // messages queued
volatile uint16_t tx_drop_count = 0;        // DEBUG: messages dropped (ring full)

// Queue a string for transmission from main loop
static void uart_queue(const char *str) {
    if (tx_ring_count >= TX_RING_SLOTS) {
        tx_drop_count++;
        return;  // ring full, drop
    }
    uint16_t len = 0;
    while (str[len] && len < TX_QUEUE_SIZE - 1) {
        tx_ring[tx_ring_head][len] = str[len];
        len++;
    }
    tx_ring[tx_ring_head][len] = '\0';
    tx_ring_head = (tx_ring_head + 1) % TX_RING_SLOTS;
    tx_ring_count++;
}
static volatile uint16_t uart_rx_idle_ms = 0;    // ms since last RX byte (for timeout)
static volatile uint8_t ble_debug_state = 0;      // UART init progress (0=not started, 1=Open called, 4=ready)
static volatile uint16_t uart_rx_byte_count = 0;   // DEBUG: total bytes received on UART RX
static volatile uint16_t uart_err_count = 0;        // DEBUG: UART error events (framing, parity, overrun)

// DEBUG: SCI3 register snapshots (read in debugger)
static volatile uint32_t dbg_sci3_ccr0 = 0;   // CCR0: check RE bit (bit 0) = receiver enabled
static volatile uint32_t dbg_sci3_csr  = 0;   // CSR: RDRF(31), ORER(24), FER(28), PER(27)
static volatile uint32_t dbg_sci3_ccr1 = 0;   // CCR1: check SHARPS bit (bit 20) = half-duplex
static volatile uint32_t dbg_sci3_ccr2 = 0;   // CCR2: baud rate (BRR, MDDR, CKS)
static volatile uint16_t dbg_poll_count = 0;   // How many times polling ran
// DEBUG: raw byte capture (first 64 bytes received)
static volatile uint8_t dbg_rx_raw[64] = {0};
static volatile uint8_t dbg_rx_raw_idx = 0;
volatile uint32_t dbg_ccr2_value = 0;  // actual CCR2 register after UART Open
volatile uint16_t dbg_ping_count = 0;  // count how many PINGs were queued
volatile uint8_t dbg_actual_brr = 0;   // BRR from baud_setting struct
volatile uint8_t dbg_pe05_level = 99;  // PE05 (RXD3) idle level: 0=LOW, 1=HIGH
volatile uint32_t dbg_baud_cycles = 0; // trigger: set to 1 in debugger to start measurement
volatile uint32_t dbg_baud_cyccnt = 0; // result: DWT CPU cycles for start bit LOW duration

volatile uint8_t dbg_auto_send = 0;    // set to 1 in debugger to enable auto PING (2s interval)

// DEBUG: command processing counters
volatile uint16_t dbg_cmd_rx_count = 0;    // total commands received & processed
volatile uint16_t dbg_cmd_get_count = 0;   // GET commands received
volatile uint16_t dbg_cmd_timeout_count = 0; // commands completed via idle timeout (not CR/LF)

// TX spacing: wait N ms between UART messages to give DA14531 time to forward to BLE
volatile uint8_t tx_spacing_ms = 30;       // ms between messages (tunable via debugger)
static volatile uint8_t tx_spacing_timer = 0;  // countdown timer (decremented in 1ms tick)

// BLE log streaming
static volatile uint8_t ble_log_streaming = 0;      // 0=stopped, 1=running
static volatile uint16_t ble_log_interval = 100;     // ms (default 10Hz)
static uint16_t ble_log_counter = 0;

// LOG auto-stop: DA14531 SPS freezes after ~10 UART→BLE messages.
// Send N lines then auto-stop. MATLAB sends LOG START again to resume.
volatile uint8_t log_burst_max = 1;          // auto-stop after N packed msgs (DA14531 stalls after ~1)
static uint8_t log_burst_sent = 0;           // lines sent in current burst
volatile uint16_t dbg_log_lines_total = 0;   // DEBUG: total LOG lines sent
volatile uint16_t dbg_log_bursts = 0;        // DEBUG: how many bursts completed

// ============================================================
// ICM-42605 IMU (SPI1)
// ============================================================
#define ICM42605_WHO_AM_I       0x75
#define ICM42605_DEVICE_CONFIG  0x11
#define ICM42605_PWR_MGMT0     0x4E
#define ICM42605_GYRO_CONFIG0  0x4F
#define ICM42605_ACCEL_CONFIG0 0x50
#define ICM42605_REG_BANK_SEL  0x76
#define ICM42605_ACCEL_DATA_X1 0x1F
#define ICM42605_INT_STATUS    0x2D
#define ICM42605_DEVICE_ID     0x42

static volatile bool spi1_tx_done = false;
volatile uint8_t imu_who_am_i = 0;        // debug: should be 0x42
volatile uint8_t imu_init_ok = 0;         // 1=init success
volatile uint8_t imu_pwr_mgmt0 = 0;      // debug: readback of PWR_MGMT0
volatile uint8_t imu_accel_cfg = 0;       // debug: readback of ACCEL_CONFIG0
volatile uint8_t imu_int_status = 0;      // debug: INT_STATUS register
volatile uint8_t imu_spi_open_err = 0xFF; // debug: SPI Open result (0=OK)
volatile uint8_t imu_init_stage = 0;      // debug: tracks init progress
volatile uint8_t imu_spi_cb_count = 0;    // debug: SPI callback fire count
volatile uint8_t imu_spi_event = 0xFF;    // debug: last SPI event type
volatile uint8_t imu_rx_byte0 = 0;        // debug: raw rx buf[0]
volatile uint8_t imu_rx_byte1 = 0;        // debug: raw rx buf[1]
volatile uint8_t imu_spi_timeout = 0;     // debug: 1=timeout occurred
volatile uint8_t imu_spi_wr_err = 0;     // debug: last WriteRead return code
volatile uint8_t imu_spi_timeout_cnt = 0; // debug: total timeout count
volatile int16_t imu_accel_raw[3] = {0};  // X, Y, Z raw (LSB)
volatile int16_t imu_gyro_raw[3]  = {0};  // X, Y, Z raw (LSB)
volatile float imu_accel_g[3] = {0};      // X, Y, Z in g
volatile float imu_gyro_dps[3] = {0};     // X, Y, Z in deg/s
static volatile uint8_t imu_read_request = 0;  // set by 1ms ISR, cleared by main loop

// ICM-42605 sensitivity (default full-scale)
// Accel: +/-16g → 2048 LSB/g
// Gyro: +/-2000dps → 16.4 LSB/dps
#define IMU_ACCEL_SENS 2048.0f
#define IMU_GYRO_SENS  16.4f

// FSP SPI helpers (ICM-42605)
static uint8_t spi1_tx_buf[16];
static uint8_t spi1_rx_buf[16];

static void spi1_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    spi1_tx_done = false;
    fsp_err_t err = R_SPI_B_WriteRead(&g_spi1_ctrl, tx, rx, len, SPI_BIT_WIDTH_8_BITS);
    imu_spi_wr_err = (uint8_t)err;
    if (err != FSP_SUCCESS) {
        return;
    }
    volatile uint32_t timeout = 2400000;
    while (!spi1_tx_done && --timeout > 0) {}
    if (timeout == 0) {
        imu_spi_timeout = 1;
        imu_spi_timeout_cnt++;
    }
    imu_rx_byte0 = rx[0];
    imu_rx_byte1 = rx[1];
}

static uint8_t icm_read_reg(uint8_t reg)
{
    spi1_tx_buf[0] = reg | 0x80;
    spi1_tx_buf[1] = 0x00;
    spi1_xfer(spi1_tx_buf, spi1_rx_buf, 2);
    return spi1_rx_buf[1];
}

static void icm_write_reg(uint8_t reg, uint8_t val)
{
    spi1_tx_buf[0] = reg & 0x7F;
    spi1_tx_buf[1] = val;
    spi1_xfer(spi1_tx_buf, spi1_rx_buf, 2);
}

static void icm42605_init(void)
{
    imu_init_stage = 1;  // entering init

    // Soft reset
    icm_write_reg(ICM42605_DEVICE_CONFIG, 0x01);
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);

    imu_init_stage = 2;  // after reset

    // Verify WHO_AM_I
    imu_who_am_i = icm_read_reg(ICM42605_WHO_AM_I);

    imu_init_stage = 3;  // after WHO_AM_I read

    if (imu_who_am_i != ICM42605_DEVICE_ID) {
        imu_init_ok = 0;
        return;
    }

    // Bank 0
    icm_write_reg(ICM42605_REG_BANK_SEL, 0x00);

    // Accel: +/-16g, 100Hz ODR
    // ACCEL_CONFIG0[7:5]=ACCEL_FS_SEL: 000=±16g
    // ACCEL_CONFIG0[3:0]=ACCEL_ODR: 1000=100Hz
    icm_write_reg(ICM42605_ACCEL_CONFIG0, 0x08);
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);

    // Gyro: +/-2000dps, 100Hz ODR
    // GYRO_CONFIG0[7:5]=GYRO_FS_SEL: 000=±2000dps
    // GYRO_CONFIG0[3:0]=GYRO_ODR: 1000=100Hz
    icm_write_reg(ICM42605_GYRO_CONFIG0, 0x08);
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);

    // Enable accel + gyro in Low Noise mode
    // PWR_MGMT0[3:2]=GYRO_MODE: 11=LN, PWR_MGMT0[1:0]=ACCEL_MODE: 11=LN
    icm_write_reg(ICM42605_PWR_MGMT0, 0x0F);
    R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);  // 200ms for gyro+accel startup

    // Debug: readback config registers
    imu_pwr_mgmt0 = icm_read_reg(ICM42605_PWR_MGMT0);
    imu_accel_cfg = icm_read_reg(ICM42605_ACCEL_CONFIG0);
    imu_int_status = icm_read_reg(ICM42605_INT_STATUS);

    imu_init_ok = 1;
}

static void icm42605_read_all(void)
{
    if (!imu_init_ok) return;

    // Try individual register reads first (debug: bypass burst read issues)
    uint8_t h, l;
    h = icm_read_reg(0x1F); l = icm_read_reg(0x20);
    imu_accel_raw[0] = (int16_t)((h << 8) | l);
    h = icm_read_reg(0x21); l = icm_read_reg(0x22);
    imu_accel_raw[1] = (int16_t)((h << 8) | l);
    h = icm_read_reg(0x23); l = icm_read_reg(0x24);
    imu_accel_raw[2] = (int16_t)((h << 8) | l);
    h = icm_read_reg(0x25); l = icm_read_reg(0x26);
    imu_gyro_raw[0] = (int16_t)((h << 8) | l);
    h = icm_read_reg(0x27); l = icm_read_reg(0x28);
    imu_gyro_raw[1] = (int16_t)((h << 8) | l);
    h = icm_read_reg(0x29); l = icm_read_reg(0x2A);
    imu_gyro_raw[2] = (int16_t)((h << 8) | l);

    imu_accel_g[0] = (float)imu_accel_raw[0] / IMU_ACCEL_SENS;
    imu_accel_g[1] = (float)imu_accel_raw[1] / IMU_ACCEL_SENS;
    imu_accel_g[2] = (float)imu_accel_raw[2] / IMU_ACCEL_SENS;
    imu_gyro_dps[0] = (float)imu_gyro_raw[0] / IMU_GYRO_SENS;
    imu_gyro_dps[1] = (float)imu_gyro_raw[1] / IMU_GYRO_SENS;
    imu_gyro_dps[2] = (float)imu_gyro_raw[2] / IMU_GYRO_SENS;
}

// Packed data mode: accumulate N samples, send as one BLE message
#define PACK_SIZE 5   // 5 RPM + IMU ≈ 55 bytes
static int16_t pack_rpm[PACK_SIZE];          // RPM samples (integer, rounded)
static uint8_t pack_idx = 0;                 // next sample slot
static volatile uint8_t pack_mode = 1;       // 1=packed (default), 0=legacy single-line

// Variable registry for GET/SET/LIST commands
typedef enum { VAR_FLOAT, VAR_UINT8, VAR_UINT32 } var_type_t;
typedef struct {
    const char *name;
    void       *ptr;
    var_type_t  type;
    uint8_t     writable;   // 0=read-only, 1=read-write
} var_entry_t;

static const var_entry_t var_registry[] = {
    // Read-only (monitoring)
    {"SpeedFb_RPM",      (void*)&SpeedFb_RPM,        VAR_FLOAT,  0},
    {"IqFb",             (void*)&IqFb,                VAR_FLOAT,  0},
    {"battery_voltage",  (void*)&battery_voltage,     VAR_FLOAT,  0},
    {"temp_inverter",    (void*)&temp_inverter,       VAR_FLOAT,  0},
    {"temp_regen",       (void*)&temp_regen,          VAR_FLOAT,  0},
    {"protection_state", (void*)&protection_state,    VAR_UINT8,  0},
    {"SpeedFb_Hall_PU",  (void*)&SpeedFb_Hall_PU,     VAR_FLOAT,  0},
    {"IdqRef1",          (void*)&IdqRef[1],           VAR_FLOAT,  0},
    // IMU (read-only)
    {"imu_ax",           (void*)&imu_accel_g[0],      VAR_FLOAT,  0},
    {"imu_ay",           (void*)&imu_accel_g[1],      VAR_FLOAT,  0},
    {"imu_az",           (void*)&imu_accel_g[2],      VAR_FLOAT,  0},
    {"imu_gx",           (void*)&imu_gyro_dps[0],     VAR_FLOAT,  0},
    {"imu_gy",           (void*)&imu_gyro_dps[1],     VAR_FLOAT,  0},
    {"imu_gz",           (void*)&imu_gyro_dps[2],     VAR_FLOAT,  0},
    // Read-write (parameters)
    {"control_mode",        (void*)&control_mode,        VAR_UINT8,  1},
    {"torque_ref_iq",       (void*)&torque_ref_iq,       VAR_FLOAT,  1},
    {"torque_ref_iq_max",   (void*)&torque_ref_iq_max,   VAR_FLOAT,  1},
    {"speed_filter_alpha",  (void*)&speed_filter_alpha,  VAR_FLOAT,  1},
    {"vdc_compensation_en", (void*)&vdc_compensation_en, VAR_UINT8,  1},
};
#define VAR_REGISTRY_SIZE (sizeof(var_registry)/sizeof(var_registry[0]))

// LIST command state machine (sends one var per 1ms tick)
static int16_t list_send_idx = -1;  // -1=idle, 0+=current index

// ============================================================
// FSP Hall Module Runtime Overrides
// These override const values in hal_data.c so we never need to
// hand-edit generated code after FSP regeneration.
// ============================================================
static motor_sense_hall_extended_cfg_t g_motor_angle0_extend_override;  // Mutable copy
static motor_angle_cfg_t              g_motor_angle0_cfg_override;      // Mutable copy

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER
adc_b_fifo_read_t  ADC_data;
adc_b_fifo_read_t  ADC_data1;
timer_info_t Encoder_data;
timer_status_t pStatus;
bsp_io_level_t p_pin_value;
bsp_io_port_t p_port_value;

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    extern sci_b_baud_setting_t g_uart3_baud_setting;
    /* TODO: add your own code here */

 // Initialize Hall sensor FIRST (before ADC interrupts can call AngleSpeedGet)
 RM_MOTOR_SENSE_HALL_Open(&g_motor_angle0_ctrl, &g_motor_angle0_cfg);

 // Override FSP-generated Hall config values at runtime.
 // This eliminates the need to hand-edit ra_gen/hal_data.c after every FSP regeneration.
 memcpy(&g_motor_angle0_extend_override, g_motor_angle0_cfg.p_extend,
        sizeof(motor_sense_hall_extended_cfg_t));
 g_motor_angle0_extend_override.f_pwm_carrier_freq = 10.0f;   // was 20.0 (actual ADC = 10kHz)
 g_motor_angle0_extend_override.u2_maximum_period   = 2000;    // was 48 (low speed support)
 g_motor_angle0_extend_override.f4_start_speed_rad  = 0.0f;    // was 250.0 (disable pseudo speed)

 g_motor_angle0_cfg_override.p_context = g_motor_angle0_cfg.p_context;
 g_motor_angle0_cfg_override.p_extend  = &g_motor_angle0_extend_override;

 RM_MOTOR_SENSE_HALL_ParameterUpdate(&g_motor_angle0_ctrl, &g_motor_angle0_cfg_override);

 // Initialize FOC control before interrupts start
 FOCCurrentControl_initialize();
 SpeedControl_initialize();

 volatile fsp_err_t debug_err_open = R_ADC_B_Open(&g_adc0_ctrl, &g_adc0_cfg);
 volatile fsp_err_t debug_err_scancfg = R_ADC_B_ScanCfg(&g_adc0_ctrl, &g_adc0_scan_cfg);
 volatile fsp_err_t debug_err_cal = R_ADC_B_Calibrate(&g_adc0_ctrl, NULL);
 // Wait for calibration to complete
 adc_status_t adc_status;
 do {
     R_ADC_B_StatusGet(&g_adc0_ctrl, &adc_status);
 } while (adc_status.state == ADC_STATE_CALIBRATION_IN_PROGRESS);
 volatile uint8_t debug_adc_state = adc_status.state;  // 0=IDLE, should not be CALIBRATING

 R_GPT_THREE_PHASE_Open(&g_three_phase0_ctrl, &g_three_phase0_cfg);
 R_GPT_THREE_PHASE_Start(&g_three_phase0_ctrl);

 // Reset and enable DRV8302 gate driver (EN_GATE pin) BEFORE enabling PWM
 // Apply Low pulse for reset, then set High to enable
 R_IOPORT_PinWrite(&g_ioport_ctrl, EN_GATE_RESET, BSP_IO_LEVEL_LOW);
 // Wait for reset (1 second)
 R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_SECONDS);

 R_IOPORT_PinWrite(&g_ioport_ctrl, EN_GATE_RESET, BSP_IO_LEVEL_HIGH);
 // Wait for DRV8302 stabilization (10ms)
 R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);

 // Set default ADC offset for current sensors (midpoint of 12-bit ADC = 2048)
 // This is the zero-current point for current sensors with VCC/2 bias
 Iab_offset[0] = 2048;
 Iab_offset[1] = 2048;

 // Start ADC scanning
 R_ADC_B_ScanGroupStart(&g_adc0_ctrl, ADC_GROUP_MASK_0);  // Group 0 = all Unit 0 channels (current + monitoring)
 R_ADC_B_ScanGroupStart(&g_adc0_ctrl, ADC_GROUP_MASK_2);  // Group 2 = voltage monitoring (Unit 1: AN006/AN018)

 // Enable PWM output
 R_GPT_OutputEnable(&g_timer0_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
 R_GPT_OutputEnable(&g_timer1_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
 R_GPT_OutputEnable(&g_timer2_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);

 R_GPT_Open(&g_timer4_ctrl, &g_timer4_cfg);
 R_GPT_Enable(&g_timer4_ctrl);
 R_GPT_Start(&g_timer4_ctrl);

 // Step 1: Force PE06 to GPIO output HIGH with internal pull-up.
 // Pull-up keeps P0_5 (DA14531 RX) HIGH during any pin mode transition.
 R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06,
     ((uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT
      | (uint32_t) IOPORT_CFG_PORT_OUTPUT_HIGH
      | (uint32_t) IOPORT_CFG_PULLUP_ENABLE));
 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_HIGH);

 // PE02 → P0_0: keep HIGH
 R_IOPORT_PinWrite(&g_ioport_ctrl, BLE_RESET_PIN, BSP_IO_LEVEL_HIGH);
 R_BSP_SoftwareDelay(2000, BSP_DELAY_UNITS_MILLISECONDS);  // Wait for DA14531 full boot

 // Pre-set PE06 (TXD3) as GPIO HIGH before UART Open
 // UART idle = HIGH. Without this, UART Open causes a LOW glitch
 // that DA14531 interprets as a start bit → framing error → SPS stops forwarding.
 R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06,
     ((uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT
      | (uint32_t) IOPORT_CFG_PORT_OUTPUT_HIGH));
 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_HIGH);

 // Wait for DA14531 to fully boot (especially after debugger flash/reset)
 R_BSP_SoftwareDelay(2000, BSP_DELAY_UNITS_MILLISECONDS);

 // Check PE05 (DA14531 TX / RA6T2 RX) idle voltage level
 {
     bsp_io_level_t pe05_level;
     R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, &pe05_level);
     dbg_pe05_level = (uint8_t)pe05_level;
 }

 // Open UART (PE06 is already HIGH, matching UART idle state)
 fsp_err_t uart_err = R_SCI_B_UART_Open(&g_uart3_ctrl, &g_uart3_cfg);
 if (uart_err == FSP_SUCCESS) {
     ble_debug_state = 2;

     // Now switch PE06 to UART peripheral function (TXD3)
     R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06,
         ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
          | (uint32_t) IOPORT_PERIPHERAL_SCI1_3_5_7_9
          | (uint32_t) IOPORT_CFG_PULLUP_ENABLE));

     R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);

     dbg_rx_raw_idx = 0;
     memset((void *)dbg_rx_raw, 0, 64);
     uart_rx_byte_count = 0;
     uart_tx_busy = 0;

     extern volatile uint32_t dbg_ccr2_value;
     extern volatile uint8_t dbg_actual_brr;
     dbg_ccr2_value = g_uart3_ctrl.p_reg->CCR2;
     dbg_actual_brr = g_uart3_baud_setting.baudrate_bits_b.brr;

     ble_debug_state = 4;
 } else {
     ble_debug_state = 99;
 }

    // Initialize ICM-42605 IMU via FSP SPI_B
    {
        fsp_err_t spi_err = R_SPI_B_Open(&g_spi1_ctrl, &g_spi1_cfg);
        imu_spi_open_err = (uint8_t)spi_err;
        if (spi_err == FSP_SUCCESS) {
            R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
            icm42605_init();
        }
    }

    while(1)
    {
        // Baud rate measurement using DWT cycle counter (precise)
        // Set dbg_baud_cycles=1 in debugger, then send 0x00 from nRF Connect.
        // 0x00 = start bit(L) + 8 data bits(L) + stop bit(H) = 9 consecutive LOW bits.
        // Result: dbg_baud_cyccnt = CPU cycles for 9 LOW bits.
        // Baud rate = 240000000 / (dbg_baud_cyccnt / 9)
        if (dbg_baud_cycles == 1) {
            // Temporarily switch PE05 to GPIO input
            R_SCI_B_UART_Close(&g_uart3_ctrl);
            R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05,
                ((uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT
                 | (uint32_t) IOPORT_CFG_PULLUP_ENABLE));

            // Enable DWT cycle counter
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

            // Wait for pin to settle after mode change
            R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
            dbg_baud_cycles = 2;  // waiting for falling edge
            dbg_baud_cyccnt = 0;

            // Tight polling: wait for falling edge (no debounce, no function call overhead)
            // Direct register read for speed: PE05 = Port 14, Pin 05
            volatile uint16_t *p_pidr = &R_PORT14->PIDR;  // Port Input Data Register
            const uint16_t pin_mask = (1U << 5);           // Pin 05

            // Wait for LOW (start bit of 0x00)
            while ((*p_pidr & pin_mask) != 0) { /* spin */ }

            // Immediately capture cycle count
            DWT->CYCCNT = 0;

            // Wait for HIGH (stop bit — after 9 LOW bits: 1 start + 8 data zeros)
            while ((*p_pidr & pin_mask) == 0) { /* spin */ }

            dbg_baud_cyccnt = DWT->CYCCNT;  // CPU cycles for 9 bits (start + 8×0)
            dbg_baud_cycles = 3;  // success

            // Restore UART
            R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05,
                ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN
                 | (uint32_t) IOPORT_PERIPHERAL_SCI1_3_5_7_9));
            R_SCI_B_UART_Open(&g_uart3_ctrl, &g_uart3_cfg);
            ble_debug_state = 4;
        }

        // Read IMU at 100Hz (flag set by One_ms_Int every 10ms)
        if (imu_read_request) {
            imu_read_request = 0;
            icm42605_read_all();
        }

        // Drain TX ring buffer via SCI3 hardware UART
        // tx_spacing_timer ensures DA14531 has time to forward previous message to BLE
        if (tx_ring_count > 0 && !uart_tx_busy && tx_spacing_timer == 0) {
            uint16_t len = (uint16_t)strlen(tx_ring[tx_ring_tail]);
            if (len > 0) {
                uart_tx_busy = 1;
                R_SCI_B_UART_Write(&g_uart3_ctrl, (uint8_t *)tx_ring[tx_ring_tail], len);
                tx_spacing_timer = tx_spacing_ms;  // wait before next message
            }
            tx_ring_tail = (tx_ring_tail + 1) % TX_RING_SLOTS;
            tx_ring_count--;
        }
    }
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open (&g_ioport_ctrl, &IOPORT_CFG_NAME);
    }
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif

/* Callback function */
void rm_motor_driver_cyclic(adc_callback_args_t *p_args)
{
    // PWM period for triangle wave symmetric mode (0x1770 = 6000)
    // Timer is configured with period_counts = 6000
    const float PWMPeriod = 6000.0f;
    three_phase_duty_cycle_t p_duty_cycle;
    // DRV8302 reset is now handled in hal_entry() before PWM output is enabled
    if(p_args->group_mask==ADC_GROUP_MASK_0)
    {
        adc_callback_count++;  // Debug counter
        // Check DRV8302 nFAULT pin status (PB00)
        // nFAULT is active-low: Low = fault, High = normal
        bsp_io_level_t fault_pin;
        R_IOPORT_PinRead(&g_ioport_ctrl, nFAULT, &fault_pin);
        drv8302_fault = (fault_pin == BSP_IO_LEVEL_LOW) ? 1 : 0;

        // AUTO-STOP: Immediately disable motor on DRV8302 fault
        if(drv8302_fault) {
            drv8302_fault_latched = 1;  // Latch fault (requires power cycle)
        }
        if(drv8302_fault_latched) {
            Enable = 0;                 // Disable motor control
            IdqRef[0] = 0.0f;           // Clear current references
            IdqRef[1] = 0.0f;
            Vabc_out[0] = 0.5f;         // Set PWM to 50% (zero voltage output)
            Vabc_out[1] = 0.5f;
            Vabc_out[2] = 0.5f;
        }

        // Read Group 0 FIFO (all channels: current sensing + monitoring)
        // AN012-AN016 moved to Group 0 (Unit 0) to fix Unit mismatch
        R_ADC_B_FifoRead(&g_adc0_ctrl, ADC_GROUP_MASK_0, &ADC_data);

        // Debug dump
        for(uint8_t i = 0; i < ADC_data.count && i < 8; i++) {
            if(i < 4) {
                debug_fifo0_data[i] = (uint16_t) ADC_data.fifo_data[i].data;
                debug_fifo0_ch[i]   = (uint8_t)  ADC_data.fifo_data[i].physical_channel;
            }
            debug_fifo2_data[i] = (uint16_t) ADC_data.fifo_data[i].data;
            debug_fifo2_ch[i]   = (uint8_t)  ADC_data.fifo_data[i].physical_channel;
            debug_fifo2_err[i]  = (uint8_t)  ADC_data.fifo_data[i].err;
        }
        debug_fifo2_count = ADC_data.count;

        // Extract all channels by physical_channel matching
        // FIFO order may vary, so match by channel number
        for(uint8_t i = 0; i < ADC_data.count && i < 8; i++) {
            uint8_t ch = (uint8_t) ADC_data.fifo_data[i].physical_channel;
            uint16_t val = (uint16_t) ADC_data.fifo_data[i].data;
            switch(ch) {
                case 0:  Iab[0] = val; break;             // AN000 = W-phase current
                case 4:  Iab[1] = val; break;             // AN004 = V-phase current
                case 12: temp_inv_adc_raw = val; break;   // AN012 = Inverter temperature
                case 13: temp_regen_adc_raw = val; break;  // AN013 = Regen temperature
                case 14: battery_adc_raw = val; break;     // AN014 = Battery voltage
                default: break;
            }
        }

        // Convert battery ADC to voltage
        // V_bat = (ADC / 4095) * 3.3V * divider_ratio
        battery_voltage = ((float)battery_adc_raw / 4095.0f) * VBAT_ADC_VREF * VBAT_DIVIDER_RATIO;

        // Convert NTC ADC to temperature [°C]
        // Using B-parameter equation: T = 1 / (1/T0 + (1/B)*ln(R/R0))
        // R = R_pullup * ADC / (4095 - ADC)  (for pull-up to VCC divider)
        // R_ntc = R_pulldown * (4095 - ADC) / ADC  (NTC is pull-up, 10kΩ is pull-down)
        if(temp_inv_adc_raw > 10 && temp_inv_adc_raw < 4085) {
            float r_ntc = NTC_PULLDOWN * (4095.0f - (float)temp_inv_adc_raw) / (float)temp_inv_adc_raw;
            float t_kelvin = 1.0f / (1.0f/NTC_T0 + (1.0f/NTC_BETA) * logf(r_ntc / NTC_R0));
            temp_inverter = t_kelvin - 273.15f;
        }
        if(temp_regen_adc_raw > 10 && temp_regen_adc_raw < 4085) {
            float r_ntc = NTC_PULLDOWN * (4095.0f - (float)temp_regen_adc_raw) / (float)temp_regen_adc_raw;
            float t_kelvin = 1.0f / (1.0f/NTC_T0 + (1.0f/NTC_BETA) * logf(r_ntc / NTC_R0));
            temp_regen = t_kelvin - 273.15f;
        }

        // Set speed reference for startup sequence
        // Speed range is -pmsm.N_base to +pmsm.N_base
        // SpeedRefIn_PU = ((float) (ADC_data1.fifo_data[1].data - 2048) / 2047.0f);  // POT input (commented out)

        // Gradually ramp up speed reference to help state machine transition
        // State machine requires SpeedRef >= 0.15 PU to transition from OpenLoop to ClosedLoop
        // speed_ref_ramp and ramp_complete are now global variables for debugger access
        //
        // IMPORTANT: Do NOT track SpeedFb in open-loop!
        // Tracking SpeedFb creates positive feedback: higher SpeedFb -> higher SpeedRef
        // -> higher voltage -> motor accelerates -> even higher SpeedFb
        // Instead, use fixed low SpeedRef to limit open-loop voltage.

        if(ramp_complete == 0) {
            // Phase 1: Ramp to 0.03 PU (very low initial speed for safety)
            // Start with very low target to prevent scary fast startup
            if(speed_ref_ramp < 0.1f) {
                speed_ref_ramp += 0.00001f;  // Ramp to 0.1 PU (~10 seconds)
            } else {
                ramp_complete = 1;  // Ramp complete
            }
            SpeedRefIn_PU = speed_ref_ramp;
        } else {
            // Phase 2: Hold at low target speed for safety
            // Can increase this value manually via debugger once stable
            SpeedRefIn_PU = 0.1f;
        }
        // SpeedRefIn_PU is now fixed (not tracking SpeedFb)
        SpeedRef_RPM = SpeedRefIn_PU * pmsm.N_base;

        // R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_04, &p_pin_value);
        // Enable = (unsigned char) p_pin_value;

        // Delay enable signal to allow system stabilization
        // Wait for at least 100ms after power-up before enabling motor control
        static uint32_t startup_delay_counter = 0;
        if(adc_callback_count > 100) {
            Enable = 1;  // Enable motor control after stabilization
        } else {
            Enable = 0;  // Keep disabled during startup
        }

        // Get angle/speed data from Hall sensor
        (void) RM_MOTOR_SENSE_HALL_AngleSpeedGet(&g_motor_angle0_ctrl, &f_get_angle, &f_get_speed, &f_get_phase_err);
        debug_f_get_speed_raw = f_get_speed;  // Save raw FSP speed for comparison

        // ============================================================
        // Improved speed estimation from angle derivative
        //
        // f_get_angle is linearly interpolated by FSP between Hall edges
        // (after u2_maximum_period fix to 2000, interpolation works at low speed).
        // We compute speed as slope of f_get_angle over a sliding window,
        // producing much smoother speed than edge-only f_get_speed.
        //
        // Method:
        //   1. Unwrap f_get_angle (handle 0 <-> 2*PI wraparound)
        //   2. Store in ring buffer
        //   3. speed = (angle_now - angle_N_ago) / (N * Ts)
        // ============================================================

        // Step 1: Unwrap angle (detect and correct 0/2PI wraparound)
        float angle_now = f_get_angle;
        float angle_delta = angle_now - prev_angle_for_unwrap;
        if (angle_delta > 3.14159265f) {
            angle_delta -= 6.28318530f;   // Wrapped from ~2PI to ~0
        } else if (angle_delta < -3.14159265f) {
            angle_delta += 6.28318530f;   // Wrapped from ~0 to ~2PI
        }
        prev_angle_for_unwrap = angle_now;
        angle_accumulated += angle_delta;
        debug_angle_unwrapped = angle_accumulated;
        hall_enc_count = (int32_t)(angle_accumulated * (6.0f / (2.0f * 3.14159265f)));

        // Step 2: Store in ring buffer, retrieve oldest value
        float old_accumulated = angle_history[angle_hist_idx];
        angle_history[angle_hist_idx] = angle_accumulated;
        angle_hist_idx++;
        if (angle_hist_idx >= SPEED_EST_WINDOW) {
            angle_hist_idx = 0;
            angle_hist_filled = 1;
        }

        // Step 3: Compute speed from angle change over window
        float f_mechanical_speed = 0.0f;
        if (angle_hist_filled) {
            float angle_diff = angle_accumulated - old_accumulated;
            float electrical_speed = angle_diff / ((float)SPEED_EST_WINDOW * 0.0001f);  // [rad/s elec]
            float mechanical_speed = electrical_speed / pmsm.p;                          // [rad/s mech]

            // Clamp negative/near-zero speed (forward-only bicycle application)
            if (mechanical_speed < 0.5f) {
                mechanical_speed = 0.0f;
            }
            f_mechanical_speed = mechanical_speed;
        } else {
            // Ring buffer not yet full: use FSP speed as fallback during startup
            if (isfinite(f_get_speed) && pmsm.p > 0.0f) {
                f_mechanical_speed = f_get_speed / pmsm.p;
                if (f_mechanical_speed < 0.5f) {
                    f_mechanical_speed = 0.0f;
                }
            }
        }
        debug_speed_from_angle = f_mechanical_speed;

        float base_speed_radps = pmsm.N_base * 2.0f * 3.14159265f / 60.0f;

        // Calculate speed feedback with tunable IIR filter
        // Angle-derivative speed is already window-averaged (10ms),
        // so a lighter IIR filter suffices (default alpha=0.10)
        if (base_speed_radps > 0.0f) {
            float speed_raw = f_mechanical_speed / base_speed_radps;
            SpeedFb_Hall_PU = SpeedFb_Hall_PU * (1.0f - speed_filter_alpha)
                            + speed_raw * speed_filter_alpha;
            SpeedFb_PU = speed_raw;  // Unfiltered for display
            SpeedFb_RPM = f_mechanical_speed * 60.0f / (2.0f * 3.14159265f);
        } else {
            SpeedFb_Hall_PU = 0.0f;
            SpeedFb_PU = 0.0f;
            SpeedFb_RPM = 0.0f;
        }

        // Closed loop current control step function (executed at ADC rate)
        // Pass Hall sensor angle (f_get_angle) directly to FOC control
        // Use IdqRef and EnCl calculated by speed controller (1ms rate)

        // Enable closed-loop when ramp is complete
        // Note: There WILL be speed error (SpeedFb > SpeedRef in open-loop)
        // The PI controller will apply braking torque to reduce speed
        // To prevent excessive regeneration, we rely on PI output limits
        unsigned char EnCl_smooth = 0;
        float abs_speed_error = (SpeedRefIn_PU > SpeedFb_PU) ?
                                (SpeedRefIn_PU - SpeedFb_PU) : (SpeedFb_PU - SpeedRefIn_PU);

        // TEST: Direct voltage output (bypass current PI control)
        // Set to 1 to test if Hall sensor angle is working correctly
        #define TEST_BYPASS_FOC 0

        #if TEST_BYPASS_FOC
        // (Test mode - not used)
        EnCl_smooth = 1;
        IdqRef[0] = 0.0f;
        IdqRef[1] = 0.0f;
        #else
        // Normal closed-loop operation
        // Let SpeedControl manage IdqRef directly (no override)
        EnCl_smooth = 1;
        // IdqRef is set by SpeedControl_step in One_ms_Int callback
        // DO NOT override IdqRef here - speed controller needs full control
        #endif

        // Update debug variables for closed-loop transition monitoring
        debug_encl_smooth = EnCl_smooth;
        debug_abs_speed_error = abs_speed_error;

        // Hand-written FOC (minimal, no protection)
        {
            float theta_e = f_get_angle + hall_angle_offset;
            float sin_theta = sinf(theta_e);
            float cos_theta = cosf(theta_e);

            // ADC to PU (same as Simulink: offset 2048, scale 1/2048)
            float Ia_pu = ((float)Iab[0] - 2048.0f) * 0.00048828125f;
            float Ib_pu = ((float)Iab[1] - 2048.0f) * 0.00048828125f;

            // Clarke
            float Ialpha = Ia_pu;
            float Ibeta  = (Ia_pu + 2.0f * Ib_pu) * 0.577350259f;

            // Park
            float Id_raw =  Ialpha * cos_theta + Ibeta * sin_theta;
            float Iq_raw = Ibeta * cos_theta - Ialpha * sin_theta;

            // LPF
            static float Id_fb = 0.0f, Iq_fb_val = 0.0f;
            Id_fb     += 0.1f * (Id_raw - Id_fb);
            Iq_fb_val += 0.1f * (Iq_raw - Iq_fb_val);
            IqFb = Iq_fb_val;

            // PI (P-only, Ki=0)
            static float Id_integral = 0.0f, Iq_integral = 0.0f;
            static uint8_t foc_was_enabled = 0;

            if (Enable && EnCl_smooth) {
                if (!foc_was_enabled) {
                    Id_integral = 0.0f;
                    Iq_integral = 0.0f;
                    foc_was_enabled = 1;
                }

                float Id_error = IdqRef[0] - Id_fb;
                float Iq_error = IdqRef[1] - Iq_fb_val;
                float Vd = PI_params.Kp_id * Id_error + Id_integral;
                float Vq = PI_params.Kp_iq * Iq_error + Iq_integral;

                // Circle limiter
                float Vmag2 = Vd * Vd + Vq * Vq;
                if (Vmag2 > 0.9025f) {  // 0.95^2
                    float inv_mag = 0.95f / sqrtf(Vmag2);
                    Vd *= inv_mag;
                    Vq *= inv_mag;
                }

                // Inverse Park
                float Valpha = Vd * cos_theta - Vq * sin_theta;
                float Vbeta  = Vq * cos_theta + Vd * sin_theta;

                // SVPWM (min-max injection)
                float Va = Valpha;
                float Vb = 0.866025f * Vbeta - 0.5f * Valpha;
                float Vc = -0.5f * Valpha - 0.866025f * Vbeta;
                float Vmax = Va; if (Vb > Vmax) Vmax = Vb; if (Vc > Vmax) Vmax = Vc;
                float Vmin = Va; if (Vb < Vmin) Vmin = Vb; if (Vc < Vmin) Vmin = Vc;
                float Voff = -0.5f * (Vmax + Vmin);
                Vabc_out[0] = 0.5f * (Va + Voff) * 1.15470052f + 0.5f;
                Vabc_out[1] = 0.5f * (Vb + Voff) * 1.15470052f + 0.5f;
                Vabc_out[2] = 0.5f * (Vc + Voff) * 1.15470052f + 0.5f;
            } else {
                Vabc_out[0] = 0.5f; Vabc_out[1] = 0.5f; Vabc_out[2] = 0.5f;
                foc_was_enabled = 0;
            }

            // Diagnostic
            if (Enable) {
                iq_sum += IqFb;
                iq_sq_sum += IqFb * IqFb;
                iq_avg_count++;
                if (iq_avg_count >= 1000) {
                    iq_avg = iq_sum / (float)iq_avg_count;
                    iq_rms = sqrtf(iq_sq_sum / (float)iq_avg_count);
                    iq_sum = 0; iq_sq_sum = 0; iq_avg_count = 0;
                }
            }

            Mode = 2.0f;
        }

        #if TEST_BYPASS_FOC
        // TEST: Direct voltage output using Hall sensor angle (bypass current PI control)
        // This tests if Hall sensor angle is correct for FOC
        // Apply fixed Vd voltage and see if motor rotates smoothly

        // f_get_angle is ALREADY electrical angle (not mechanical)
        // Do NOT multiply by pmsm.p
        float theta_e = f_get_angle;  // Already electrical angle [rad]

        // Use global variable for angle offset (change via debugger Real-time Chart)
        // 30=0.5236, 60=1.0472, 90=1.5708, 120=2.0944, 150=2.6180, 180=3.1416
        theta_e += hall_angle_offset;

        float sin_theta = sinf(theta_e);
        float cos_theta = cosf(theta_e);

        // Fixed voltage: Vd = 0, Vq = 0.15 (Vq creates torque, Vd=0 for no field weakening)
        float Vd = 0.0f;
        float Vq = 0.15f;  // Positive Vq = forward torque

        // Inverse Park Transform: Vd,Vq -> Valpha,Vbeta
        float Valpha = Vd * cos_theta - Vq * sin_theta;
        float Vbeta  = Vd * sin_theta + Vq * cos_theta;

        // Inverse Clarke Transform: Valpha,Vbeta -> Va,Vb,Vc (SVPWM)
        float Va = Valpha;
        float Vb = -0.5f * Valpha + 0.866025f * Vbeta;
        float Vc = -0.5f * Valpha - 0.866025f * Vbeta;

        // Add offset and convert to duty cycle [0, 1]
        Vabc_out[0] = 0.5f + 0.5f * Va;
        Vabc_out[1] = 0.5f + 0.5f * Vb;
        Vabc_out[2] = 0.5f + 0.5f * Vc;
        #endif

        // Update global Iab_offset for debug visibility (FOCCurrentControl may have calibrated it)
        Iab_offset[0] = Iab_offset_local[0];
        Iab_offset[1] = Iab_offset_local[1];

        // Restore Hall sensor SpeedFb_PU for display (FOCCurrentControl overwrites it)
        if(base_speed_radps > 0.0f) {
            SpeedFb_PU = f_mechanical_speed / base_speed_radps;
        }

        // Debug: Calculate speed error and current components for sign investigation
        debug_speed_error = SpeedRefIn_PU - SpeedFb_PU;  // Positive = below target speed
        // Calculate Ialpha and Ibeta manually for debugging
        float Ia_pu = ((float)Iab[0] - 2048.0f) * 0.00048828125f;
        float Ib_pu = ((float)Iab[1] - 2048.0f) * 0.00048828125f;
        debug_Ialpha = Ia_pu;
        debug_Ibeta = ((Ia_pu + Ib_pu) + Ib_pu) * 0.577350259f;

        // ============================================================
        // Data Logger: Record data at 100Hz (every 100 PWM cycles)
        // ============================================================
        if(log_running) {
            log_counter++;
            if(log_counter >= 100) {  // 10kHz / 100 = 100Hz
                log_counter = 0;

                // Record current state
                log_buffer[log_index].SpeedRef = SpeedRefIn_PU;
                log_buffer[log_index].SpeedFb = SpeedFb_PU;
                log_buffer[log_index].IqFb = IqFb;
                log_buffer[log_index].IdqRef1 = IdqRef[1];
                log_buffer[log_index].EnCl = EnCl_smooth;
                log_buffer[log_index].ramp_phase = ramp_complete;

                log_index++;
                if(log_index >= LOG_SIZE) {
                    log_index = 0;      // Wrap around (ring buffer)
                    // log_running = 0; // Uncomment to stop after one fill
                }
            }
        }

        // SAFETY: Voltage output limiting (currently disabled for PI tuning)
        // Uncomment to enable voltage limiting if needed
        /*
        const float V_MIN = 0.40f;
        const float V_MAX = 0.60f;
        if(Vabc_out[0] < V_MIN) Vabc_out[0] = V_MIN;
        if(Vabc_out[0] > V_MAX) Vabc_out[0] = V_MAX;
        if(Vabc_out[1] < V_MIN) Vabc_out[1] = V_MIN;
        if(Vabc_out[1] > V_MAX) Vabc_out[1] = V_MAX;
        if(Vabc_out[2] < V_MIN) Vabc_out[2] = V_MIN;
        if(Vabc_out[2] > V_MAX) Vabc_out[2] = V_MAX;
        */

        // V_dc feedforward compensation: adjust duty for actual battery voltage
        // FOC outputs duty assuming V_DC_NOMINAL (18V). If battery is different,
        // scale the AC component so actual motor voltage stays correct.
        //   compensated = 0.5 + (duty - 0.5) * V_DC_NOMINAL / V_battery
        if (vdc_compensation_en && battery_voltage > 10.0f) {
            float vdc_scale = V_DC_NOMINAL / battery_voltage;
            Vabc_out[0] = 0.5f + (Vabc_out[0] - 0.5f) * vdc_scale;
            Vabc_out[1] = 0.5f + (Vabc_out[1] - 0.5f) * vdc_scale;
            Vabc_out[2] = 0.5f + (Vabc_out[2] - 0.5f) * vdc_scale;
        }

        // Update inverter.V_dc for reference (not used by Simulink FOC at runtime)
        if (battery_voltage > 10.0f) {
            inverter.V_dc = battery_voltage;
        }

        // Convert duty cycle (0.0 to 1.0) to timer counts
        // For triangle wave symmetric PWM, duty cycle is specified as compare match value
        // Vabc_out ranges from 0.0 (0% duty) to 1.0 (100% duty)
        p_duty_cycle.duty[0] = (uint32_t)(PWMPeriod * Vabc_out[0]);
        p_duty_cycle.duty[1] = (uint32_t)(PWMPeriod * Vabc_out[1]);
        p_duty_cycle.duty[2] = (uint32_t)(PWMPeriod * Vabc_out[2]);

        // Clamp to valid range [0, 6000]
        if(p_duty_cycle.duty[0] > 6000) p_duty_cycle.duty[0] = 6000;
        if(p_duty_cycle.duty[1] > 6000) p_duty_cycle.duty[1] = 6000;
        if(p_duty_cycle.duty[2] > 6000) p_duty_cycle.duty[2] = 6000;

        // Store debug values
        debug_vabc[0] = Vabc_out[0];
        debug_vabc[1] = Vabc_out[1];
        debug_vabc[2] = Vabc_out[2];
        debug_duty[0] = p_duty_cycle.duty[0];
        debug_duty[1] = p_duty_cycle.duty[1];
        debug_duty[2] = p_duty_cycle.duty[2];

        R_GPT_THREE_PHASE_DutyCycleSet(&g_three_phase0_ctrl, &p_duty_cycle);
    }
}

/* Callback function */
void current_cb(timer_callback_args_t *p_args)//  this will never hit
{
    /* TODO: add your own code here */
}


// Format float as integer.fraction string (newlib-nano doesn't support %f)
// e.g., fmt_float(buf, 32, -3.14159, 4) → "-3.1415"
static int fmt_float(char *buf, int bufsz, float val, int decimals) {
    int idx = 0;
    if (val < 0.0f) {
        if (idx < bufsz - 1) buf[idx++] = '-';
        val = -val;
    }
    uint32_t int_part = (uint32_t)val;
    float frac = val - (float)int_part;
    // Integer part
    idx += snprintf(buf + idx, bufsz - idx, "%lu", (unsigned long)int_part);
    if (decimals > 0 && idx < bufsz - 1) {
        buf[idx++] = '.';
        for (int d = 0; d < decimals && idx < bufsz - 1; d++) {
            frac *= 10.0f;
            int digit = (int)frac;
            if (digit > 9) digit = 9;
            buf[idx++] = '0' + digit;
            frac -= (float)digit;
        }
    }
    buf[idx] = '\0';
    return idx;
}

static void ble_process_command(char *cmd) {
    char resp[TX_QUEUE_SIZE];

    // Trim trailing whitespace/CR/LF
    int len = (int)strlen(cmd);
    while (len > 0 && (cmd[len-1] == ' ' || cmd[len-1] == '\r' || cmd[len-1] == '\n' || cmd[len-1] == '\t')) {
        cmd[--len] = '\0';
    }
    if (len == 0) return;
    dbg_cmd_rx_count++;

    if (strncmp(cmd, "GET ", 4) == 0) {
        dbg_cmd_get_count++;
        const char *name = cmd + 4;
        for (uint16_t i = 0; i < VAR_REGISTRY_SIZE; i++) {
            if (strcmp(var_registry[i].name, name) == 0) {
                switch (var_registry[i].type) {
                    case VAR_FLOAT: {
                        char vbuf[16];
                        fmt_float(vbuf, sizeof(vbuf), *(volatile float*)var_registry[i].ptr, 4);
                        snprintf(resp, sizeof(resp), "OK %s=%s\r\n", name, vbuf);
                        break;
                    }
                    case VAR_UINT8:
                        snprintf(resp, sizeof(resp), "OK %s=%u\r\n",
                                 name, (unsigned)*(volatile uint8_t*)var_registry[i].ptr);
                        break;
                    case VAR_UINT32:
                        snprintf(resp, sizeof(resp), "OK %s=%lu\r\n",
                                 name, (unsigned long)*(volatile uint32_t*)var_registry[i].ptr);
                        break;
                }
                uart_queue(resp);
                return;
            }
        }
        snprintf(resp, sizeof(resp), "ERR unknown: %s\r\n", name);
        uart_queue(resp);

    } else if (strncmp(cmd, "SET ", 4) == 0) {
        char *name = cmd + 4;
        char *space = strchr(name, ' ');
        if (!space) {
            uart_queue("ERR SET <var> <value>\r\n");
            return;
        }
        *space = '\0';
        const char *val_str = space + 1;

        for (uint16_t i = 0; i < VAR_REGISTRY_SIZE; i++) {
            if (strcmp(var_registry[i].name, name) == 0) {
                if (!var_registry[i].writable) {
                    snprintf(resp, sizeof(resp), "ERR %s read-only\r\n", name);
                    uart_queue(resp);
                    return;
                }
                switch (var_registry[i].type) {
                    case VAR_FLOAT: {
                        float v = (float)atof(val_str);
                        *(volatile float*)var_registry[i].ptr = v;
                        char vbuf[16];
                        fmt_float(vbuf, sizeof(vbuf), v, 4);
                        snprintf(resp, sizeof(resp), "OK %s=%s\r\n", name, vbuf);
                        break;
                    }
                    case VAR_UINT8: {
                        uint8_t v = (uint8_t)atoi(val_str);
                        *(volatile uint8_t*)var_registry[i].ptr = v;
                        snprintf(resp, sizeof(resp), "OK %s=%u\r\n", name, (unsigned)v);
                        break;
                    }
                    case VAR_UINT32: {
                        uint32_t v = (uint32_t)strtoul(val_str, NULL, 10);
                        *(volatile uint32_t*)var_registry[i].ptr = v;
                        snprintf(resp, sizeof(resp), "OK %s=%lu\r\n", name, (unsigned long)v);
                        break;
                    }
                }
                uart_queue(resp);
                return;
            }
        }
        snprintf(resp, sizeof(resp), "ERR unknown: %s\r\n", name);
        uart_queue(resp);

    } else if (strncmp(cmd, "LOG START", 9) == 0) {
        uint16_t hz = 10;  // default 10Hz
        if (cmd[9] == ' ') {
            hz = (uint16_t)atoi(cmd + 10);
            if (hz == 0) hz = 1;
            if (hz > 100) hz = 100;
        }
        ble_log_interval = 1000 / hz;
        ble_log_streaming = 1;
        ble_log_counter = 0;
        log_burst_sent = 0;
        pack_idx = 0;
        // Don't send OK response — save DA14531 buffer for LOG data

    } else if (strcmp(cmd, "R") == 0) {
        // Short restart: resume LOG with previous settings (1 byte = reliable over BLE)
        // Cooldown: ignore if already streaming (prevents triple-R from generating 3 packs)
        if (ble_log_streaming == 0) {
            ble_log_streaming = 1;
            ble_log_counter = 0;
            log_burst_sent = 0;
            pack_idx = 0;
        }

    } else if (cmd[0] == 'F' && cmd[1] >= '0' && cmd[1] <= '9') {
        // Short frequency set: "F100" = 100Hz, "F1" = 1Hz (no restart)
        uint16_t hz = (uint16_t)atoi(cmd + 1);
        if (hz == 0) hz = 1;
        if (hz > 100) hz = 100;
        ble_log_interval = 1000 / hz;

    } else if (strcmp(cmd, "LOG STOP") == 0 || strcmp(cmd, "STOP") == 0 || strcmp(cmd, "S") == 0) {
        ble_log_streaming = 0;
        log_burst_sent = 0;
        pack_idx = 0;

    } else if (strcmp(cmd, "LIST") == 0) {
        list_send_idx = 0;  // Start LIST enumeration in ble_uart_tick

    } else {
        // Don't send ERR response — saves DA14531 buffer for useful data
    }
}

// Called from One_ms_Int every 1ms
static void ble_uart_tick(void) {
    // --- Phase 1: TX only test ---
    // GPIO bit-bang TX on PE05 → DA14531 RX (P0_5) → BLE DSPS TX → phone
    // RX (PE06 ← DA14531 TX) will be added later with proper timer-based sampling.
    // For now, just detect pin activity for the LED debug indicator.
    if (ble_debug_state == 4) {
        dbg_poll_count++;
    }

    // TX spacing countdown (gives DA14531 time to forward to BLE)
    if (tx_spacing_timer > 0) {
        tx_spacing_timer--;
    }

    // Timeout: if bytes in buffer and no new data for 200ms, treat as complete
    // (nRF Connect / BLE apps don't send CR/LF after text)
    if (uart_rx_idx > 0 && !uart_rx_complete) {
        uart_rx_idle_ms++;
        if (uart_rx_idle_ms >= 200) {
            uart_rx_buf[uart_rx_idx] = '\0';
            uart_rx_complete = 1;
            dbg_cmd_timeout_count++;
        }
    }

    // Process received command line
    if (uart_rx_complete) {
        char cmd_copy[UART_RX_BUF_SIZE];
        memcpy(cmd_copy, (const void*)uart_rx_buf, uart_rx_idx + 1);  // +1 for null terminator
        uart_rx_idx = 0;
        uart_rx_complete = 0;
        ble_process_command(cmd_copy);
    }

    // LIST enumeration: send one VAR line per tick when TX ring has space
    if (list_send_idx >= 0 && tx_ring_count < TX_RING_SLOTS) {
        if (list_send_idx < (int16_t)VAR_REGISTRY_SIZE) {
            char resp[TX_QUEUE_SIZE];
            const var_entry_t *v = &var_registry[list_send_idx];
            const char *tstr = (v->type == VAR_FLOAT) ? "f" :
                               (v->type == VAR_UINT8) ? "u8" : "u32";
            snprintf(resp, sizeof(resp), "VAR %s %s %s\r\n",
                     v->name, tstr, v->writable ? "rw" : "ro");
            uart_queue(resp);
            list_send_idx++;
        } else {
            list_send_idx = -1;  // Done
        }
    }

    // Debug: auto-send test message every 2 seconds (no command needed)
    // Set dbg_auto_send=1 in debugger to enable
    {
        static uint16_t auto_send_counter = 0;
        if (dbg_auto_send) {
            auto_send_counter++;
            if (auto_send_counter >= 2000 && tx_ring_count < TX_RING_SLOTS) {
                auto_send_counter = 0;
                uart_queue("PING\r\n");
                dbg_ping_count++;
            }
        }
    }

    // Log streaming with auto-stop
    if (ble_log_streaming && tx_ring_count < TX_RING_SLOTS) {
        ble_log_counter++;
        if (ble_log_counter >= ble_log_interval) {
            ble_log_counter = 0;

            if (pack_mode) {
                // Packed mode: accumulate RPM samples, send batch when full
                pack_rpm[pack_idx] = (int16_t)(SpeedFb_RPM + 0.5f);
                pack_idx++;

                if (pack_idx >= PACK_SIZE) {
                    // Pack complete: format "B50,r1,r2,...,r50,iq,vbat,tinv,treg\r\n"
                    static char resp[TX_QUEUE_SIZE];  // static to save stack (256 bytes)
                    char iq_str[12];
                    fmt_float(iq_str, sizeof(iq_str), IqFb, 3);
                    int pos = snprintf(resp, sizeof(resp), "B%u", PACK_SIZE);
                    for (uint8_t i = 0; i < PACK_SIZE; i++) {
                        pos += snprintf(resp + pos, sizeof(resp) - (size_t)pos, ",%d", pack_rpm[i]);
                    }
                    // Compact: iq(int*100), vbat, IMU(milli-g, deci-dps) — no tinv/treg
                    snprintf(resp + pos, sizeof(resp) - (size_t)pos,
                             ",%d,%u,%d,%d,%d,%d,%d,%d\r\n",
                             (int)(IqFb * 100),
                             (unsigned)(battery_voltage + 0.5f),
                             (int)(imu_accel_g[0] * 1000),
                             (int)(imu_accel_g[1] * 1000),
                             (int)(imu_accel_g[2] * 1000),
                             (int)(imu_gyro_dps[0] * 10),
                             (int)(imu_gyro_dps[1] * 10),
                             (int)(imu_gyro_dps[2] * 10));
                    uart_queue(resp);
                    pack_idx = 0;
                    dbg_log_lines_total += PACK_SIZE;

                    if (log_burst_max > 0) {
                        log_burst_sent++;
                        if (log_burst_sent >= log_burst_max) {
                            ble_log_streaming = 0;
                            log_burst_sent = 0;
                            dbg_log_bursts++;
                        }
                    }
                }
            } else {
                // Legacy single-line mode
                char resp[TX_QUEUE_SIZE];
                char v1[12], v2[12];
                fmt_float(v1, sizeof(v1), SpeedFb_RPM, 1);
                fmt_float(v2, sizeof(v2), IqFb, 3);
                snprintf(resp, sizeof(resp), "L%s,%s,%u,%u,%u\r\n",
                         v1, v2,
                         (unsigned)(battery_voltage + 0.5f),
                         (unsigned)(temp_inverter + 0.5f),
                         (unsigned)(temp_regen + 0.5f));
                uart_queue(resp);
                dbg_log_lines_total++;
                if (log_burst_max > 0) {
                    log_burst_sent++;
                    if (log_burst_sent >= log_burst_max) {
                        ble_log_streaming = 0;
                        log_burst_sent = 0;
                        dbg_log_bursts++;
                    }
                }
            }
        }
    }
}

/* Callback function */
void One_ms_Int(timer_callback_args_t *p_args)
{
    timer_callback_count++;  // Debug counter

    // Measure ADC callback rate: diff between 1s and 2s = exact 1-second count
    if (timer_callback_count == 1000) {
        adc_count_at_1s = adc_callback_count;
    } else if (timer_callback_count == 2000) {
        adc_rate_result = adc_callback_count - adc_count_at_1s;
    }
    // Read adc_rate_result after 2+ sec: 10000=10kHz, 20000=20kHz

    bsp_io_level_t power_switch_pin;

    // Speed/Torque control mode selection (1kHz rate)
    if (control_mode == 0) {
        // Mode 0: Speed control (existing behavior)
        SpeedControl_step(Enable, SpeedRefIn_PU, SpeedFb_Hall_PU, IqFb, Mode, IdqRef, &SpeedRefFinal_PU, &EnCl);
    } else {
        // Mode 1: Torque control (direct Iq command, bypass speed PI loop)
        float iq_cmd = torque_ref_iq;
        if (iq_cmd > torque_ref_iq_max)  iq_cmd = torque_ref_iq_max;
        if (iq_cmd < -torque_ref_iq_max) iq_cmd = -torque_ref_iq_max;

        IdqRef[0] = 0.0f;     // Id = 0 (no field weakening)
        IdqRef[1] = iq_cmd;   // Iq = direct torque command
        SpeedRefFinal_PU = 0.0f;
        EnCl = Enable;         // Enable current control when motor is enabled
    }

    // Speed controller output (IdqRef and EnCl) is used directly
    // Do not override EnCl - let the speed controller state machine manage it
    // The state machine will transition: Init -> OpenLoop -> ClosedLoopTemp -> ClosedLoop

    // Power switch monitoring (PE14 -> DI_PON)
    // 3-second long press → PD07 (LATCH_OFF) HIGH → NMOS ON → TC7WH74FU PR~ LOW
    // → latch released → PMOS OFF → power supply cut → shutdown
    R_IOPORT_PinRead(&g_ioport_ctrl, DI_PON, &power_switch_pin);

    debug_pe14_state = (power_switch_pin == BSP_IO_LEVEL_HIGH) ? 1 : 0;

    if(!shutdown_requested) {
        if(power_switch_pin == BSP_IO_LEVEL_HIGH) {
            power_switch_high_counter++;
            debug_pe14_counter = power_switch_high_counter;

            if(power_switch_high_counter >= 3000) {
                // 3-second long press confirmed → initiate shutdown
                shutdown_requested = 1;
                debug_shutdown_requested = 1;

                // Stop motor before power cut
                Enable = 0;
                IdqRef[0] = 0.0f;
                IdqRef[1] = 0.0f;
                Vabc_out[0] = 0.5f;
                Vabc_out[1] = 0.5f;
                Vabc_out[2] = 0.5f;

                // Assert LATCH_OFF → power will be cut by hardware
                R_IOPORT_PinWrite(&g_ioport_ctrl, LATCH_OFF, BSP_IO_LEVEL_HIGH);
                // System will lose power after this point
            }
        } else {
            // Button released before 3 seconds - reset counter
            power_switch_high_counter = 0;
            debug_pe14_counter = 0;
        }
    }

    // ============================================================
    // Protection monitoring (1kHz rate)
    // ============================================================
    {
        uint8_t new_protection = 0;
        float new_derating = 1.0f;

        // Battery low voltage check
        if(battery_voltage < VBAT_LOW_CUTOFF && battery_voltage > 1.0f) {
            // Below cutoff - emergency stop (ignore if voltage reads near 0 = no sensor)
            new_protection = 3;
            new_derating = 0.0f;
        } else if(battery_voltage < VBAT_LOW_WARNING && battery_voltage > 1.0f) {
            // Below warning - reduce output
            if(new_protection < 1) new_protection = 1;
            if(new_derating > 0.5f) new_derating = 0.5f;
        }

        // Inverter temperature check
        if(temp_inverter >= TEMP_SHUTDOWN) {
            new_protection = 3;
            new_derating = 0.0f;
        } else if(temp_inverter >= TEMP_DERATING) {
            if(new_protection < 2) new_protection = 2;
            if(new_derating > 0.3f) new_derating = 0.3f;
        } else if(temp_inverter >= TEMP_WARNING) {
            if(new_protection < 1) new_protection = 1;
            if(new_derating > 0.7f) new_derating = 0.7f;
        }

        // Regen circuit temperature check
        if(temp_regen >= TEMP_SHUTDOWN) {
            new_protection = 3;
            new_derating = 0.0f;
        } else if(temp_regen >= TEMP_DERATING) {
            if(new_protection < 2) new_protection = 2;
            if(new_derating > 0.3f) new_derating = 0.3f;
        } else if(temp_regen >= TEMP_WARNING) {
            if(new_protection < 1) new_protection = 1;
            if(new_derating > 0.7f) new_derating = 0.7f;
        }

        protection_state = new_protection;
        output_derating = new_derating;

        // Emergency stop on critical protection
        if(protection_state >= 3) {
            Enable = 0;
        }
    }

    // ============================================================
    // Switch input handling with debounce (1kHz rate)
    // ============================================================
    uint8_t sw_in1_now, sw_in2_now;
    {
        bsp_io_level_t sw_pin;

        // IN1: Assist Level Switch (cycle L→M→H→L...)
        R_IOPORT_PinRead(&g_ioport_ctrl, DI_IN1, &sw_pin);
        sw_in1_now = (sw_pin == BSP_IO_LEVEL_HIGH) ? 1 : 0;
        if(sw_in1_now != sw_in1_last) {
            sw_in1_debounce++;
            if(sw_in1_debounce >= SW_DEBOUNCE_MS) {
                sw_in1_last = sw_in1_now;
                sw_in1_debounce = 0;
                // Falling edge (High→Low = button pressed with pull-up)
                if(sw_in1_now == 0) {
                    assist_level = (assist_level + 1) % 3;  // Cycle: 0→1→2→0
                }
            }
        } else {
            sw_in1_debounce = 0;
        }

        // IN2: Turbo Switch (toggle ON/OFF)
        R_IOPORT_PinRead(&g_ioport_ctrl, DI_IN2, &sw_pin);
        sw_in2_now = (sw_pin == BSP_IO_LEVEL_HIGH) ? 1 : 0;
        if(sw_in2_now != sw_in2_last) {
            sw_in2_debounce++;
            if(sw_in2_debounce >= SW_DEBOUNCE_MS) {
                sw_in2_last = sw_in2_now;
                sw_in2_debounce = 0;
                // Falling edge
                if(sw_in2_now == 0) {
                    turbo_mode = turbo_mode ? 0 : 1;  // Toggle
                }
            }
        } else {
            sw_in2_debounce = 0;
        }
    }

    // ============================================================
    // Assist app model input population (1kHz)
    // ============================================================
    assist_U.SVON        = (Enable != 0);
    assist_U.err_type    = protection_state;
    assist_U.hall_enccnt = hall_enc_count;
    assist_U.Mot_spd     = (int16_T)SpeedFb_RPM;
    assist_U.batterycnt  = (int32_T)battery_adc_raw;
    assist_U.Acc_x       = imu_accel_raw[0];
    assist_U.Acc_y       = imu_accel_raw[1];
    assist_U.Acc_z       = imu_accel_raw[2];
    assist_U.in_flags.btn_normal_assist = (sw_in1_now == 0);  /* active low */
    assist_U.in_flags.btn_force         = (sw_in2_now == 0);  /* active low */
    assist_U.in_flags.btn_power         = (power_switch_pin == BSP_IO_LEVEL_HIGH);

    /* assist_app_step();  // TODO: uncomment when Simulink model is integrated */

    // ============================================================
    // LED output control (Active Low: MCU Low → LED ON)
    // ============================================================
    {
        // Power LED: Green=normal, Red=fault/warning
        if(drv8302_fault_latched || protection_state >= 3) {
            // Fault or emergency: Red ON, Green OFF
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_POWER_GREEN, BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_POWER_RED, BSP_IO_LEVEL_LOW);
        } else if(protection_state >= 1) {
            // Warning: Both ON (blink Red at 2Hz)
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_POWER_GREEN, BSP_IO_LEVEL_LOW);
            bsp_io_level_t blink = ((timer_callback_count / 250) % 2) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH;
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_POWER_RED, blink);
        } else {
            // Normal: Green ON, Red OFF
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_POWER_GREEN, BSP_IO_LEVEL_LOW);
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_POWER_RED, BSP_IO_LEVEL_HIGH);
        }

        // Turbo LED — override for BLE debug
        if (ble_debug_state == 0) {
            // Normal: turbo switch controls LED
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_TURBO,
                turbo_mode ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        } else if (ble_debug_state == 1) {
            // Before Open: fast blink (100ms)
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_TURBO,
                ((timer_callback_count / 100) % 2) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        } else if (ble_debug_state == 4) {
            // All OK: solid ON
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_TURBO, BSP_IO_LEVEL_LOW);
        } else {
            // Error (99=Open fail, 3=BaudCalc fail): slow blink (500ms)
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_TURBO,
                ((timer_callback_count / 500) % 2) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        }

        // Assist level LEDs — override for UART RX debug
        if (uart_rx_byte_count > 0) {
            // ANY byte received on UART RX: Assist_L ON (solid)
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_L, BSP_IO_LEVEL_LOW);
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_M, BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_H, BSP_IO_LEVEL_HIGH);
        } else {
            // Normal assist level display
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_L,
                (assist_level == 0) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_M,
                (assist_level == 1) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_H,
                (assist_level == 2) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        }
    }

    // IMU read request every 10ms (100Hz)
    {
        static uint16_t imu_div = 0;
        if (++imu_div >= 10) {
            imu_div = 0;
            imu_read_request = 1;
        }
    }

    // BLE UART command processing and log streaming
    ble_uart_tick();
}

// ============================================================
// SCI3 UART Callback (DA14531 BLE module)
// Receives one byte at a time via UART_EVENT_RX_CHAR.
// Assembles bytes into uart_rx_buf until CR or LF is received.
// ============================================================
// ============================================================
// Assist app placeholder (replace with Embedded Coder output)
// ============================================================
void assist_app_step(void)
{
    /* Simulink generated code will go here */
    (void)assist_U;  /* suppress unused warning */
}

// ============================================================
// SPI1 callback (ICM-42605)
// ============================================================
void spi1_callback(spi_callback_args_t *p_args) {
    imu_spi_cb_count++;
    imu_spi_event = (uint8_t)p_args->event;
    if (p_args->event == SPI_EVENT_TRANSFER_COMPLETE) {
        spi1_tx_done = true;
    }
}

void sci3_uart_callback(uart_callback_args_t *p_args) {
    switch (p_args->event) {
        case UART_EVENT_TX_COMPLETE:
            uart_tx_busy = 0;
            break;
        case UART_EVENT_RX_CHAR:
        {
            uint8_t ch = (uint8_t)p_args->data;
            uart_rx_byte_count++;
            // Capture first 64 raw bytes for debug
            if (dbg_rx_raw_idx < 64) {
                dbg_rx_raw[dbg_rx_raw_idx++] = ch;
            }
            // Buffer line (CR or LF = end of line)
            // If previous line not yet processed, drop new bytes
            if (uart_rx_complete) break;  // busy, drop
            if (ch == '\r' || ch == '\n') {
                if (uart_rx_idx > 0) {
                    uart_rx_buf[uart_rx_idx] = '\0';
                    uart_rx_complete = 1;
                }
            } else if (uart_rx_idx < UART_RX_BUF_SIZE - 1) {
                uart_rx_buf[uart_rx_idx++] = ch;
                uart_rx_idle_ms = 0;  // reset timeout
            }
            break;
        }
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_OVERFLOW:
            uart_err_count++;
            break;
        default:
            break;
    }
}
