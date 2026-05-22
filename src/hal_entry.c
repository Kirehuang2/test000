// Closed-loop FOC control with Hall sensor (Open-loop control removed)

#include "hal_data.h"
#include "FOCCurrentControl.h"
#include "FOCSpeedControl.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "r_three_phase_api.h"

// ============================================================
// PWM frequency parameter — single source of truth for FOC timing.
// To migrate frequency: update both this value AND configuration.xml
//   `module.driver.three_phase.period`. They must match.
// All time-based constants below derive from PWM_FREQ_HZ.
// ============================================================
#define PWM_FREQ_HZ      18000              // FOC ISR rate [Hz], must match configuration.xml
#define PWM_TS_F         (1.0f / (float)PWM_FREQ_HZ)   // FOC sample time [s]
#define MS_TO_CYCLES(ms) ((uint32_t)((ms) * PWM_FREQ_HZ / 1000))  // ms → ISR cycle count

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
float IdFb = 0.0F;                          // Id feedback for monitoring
float SpeedRefIn_PU = 0.0F;
float Mode = 2.0f;  // Mode >= 2 required to start speed control state machine

// ============================================================
// Improved speed estimation from angle derivative
// Instead of using f_get_speed (updates only at Hall edges),
// compute speed as d(angle)/dt using a sliding window over
// the interpolated f_get_angle from FSP Hall module.
// ============================================================
#define SPEED_EST_WINDOW  MS_TO_CYCLES(20)  // 20ms window, ~8RPM resolution
static float angle_history[SPEED_EST_WINDOW];  // Ring buffer of angle DELTAS (not accumulated)
static uint16_t angle_hist_idx = 0;
static uint8_t angle_hist_filled = 0;
static float prev_angle_for_unwrap = 0.0f;
static float angle_accumulated = 0.0f;         // Monotonic unwrapped angle [rad electrical] (debug only)
static float angle_delta_sum = 0.0f;           // Running sum of deltas in window

// Debug variables for improved speed estimation
volatile float debug_speed_from_angle = 0.0f;  // Speed from angle derivative [rad/s mechanical]
volatile float debug_angle_unwrapped = 0.0f;   // Accumulated unwrapped angle [rad]
volatile float debug_f_get_speed_raw = 0.0f;   // Raw FSP speed for comparison

// Speed filter coefficient (tunable via debugger)
volatile float speed_filter_alpha = 0.05f;      // 0.05: BW≈80Hz (was 0.02=32Hz, too slow for 30Hz speed loop)

// ============================================================
// Control mode switching (changeable via debugger)
// 0 = Speed control: SpeedControl_step manages IdqRef
// 1 = Torque control: direct Iq command, bypass speed loop
// ============================================================
volatile uint8_t control_mode = 1;              // 0=Speed, 1=Torque
volatile float torque_ref_iq = 0.06f;           // Direct Iq command [PU]
volatile float torque_ref_iq_max = 0.2f;        // Iq clamp: 0.2 PU × 16.5A = 3.3A ≈ rated 3.29A
volatile float torque_max_speed_rpm = 500.0f;   // Speed limit for torque mode [RPM]

// Integrator limits (tunable via debugger/BLE for safe bring-up)
// Start conservative (0.15), increase to 0.50 after stability confirmed
volatile float integrator_limit = 0.50f;        // PI integrator clamp [PU] (was 0.15, 0.5 needed for high-speed Iq tracking)

// Debug counters
volatile uint32_t adc_callback_count = 0;
volatile uint32_t timer_callback_count = 0;

// ADC callback rate measurement (read adc_rate_result in debugger after 2+ sec)
static uint32_t adc_count_at_1s = 0;        // snapshot at timer=1000
volatile uint32_t adc_rate_result = 0;       // callbacks per second (10000=10kHz, 20000=20kHz)

// FOC ISR timing measurement via DWT cycle counter (CPU 120MHz, 1 cycle = 8.33ns)
// Used for 18kHz feasibility check: ISR must complete in < ~45us (45*120=5400 cycles)
volatile uint32_t isr_cycles_last = 0;       // Last observed ISR duration [CPU cycles]
volatile uint32_t isr_cycles_max = 0;        // Max observed ISR duration since reset
volatile float    isr_us_last = 0.0f;        // Last ISR duration [us]
volatile float    isr_us_max  = 0.0f;        // Max ISR duration [us]
volatile uint8_t  isr_max_reset = 0;         // Write 1 via BLE to clear max

// V_dc voltage compensation
#define V_DC_NOMINAL  18.0f              // Nominal voltage the PU system is calibrated for
volatile uint8_t vdc_compensation_en = 0;  // default OFF

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
volatile float debug_Iq_integral = 0.0f;      // For streaming diagnostic
volatile float debug_speed_integ = 0.0f;      // Speed PI integrator state
volatile float debug_Vmag2 = 0.0f;           // Voltage vector magnitude squared (0.9025=saturated)
volatile float debug_Vd_sat = 0.0f;          // Vd after circle limiter
volatile float debug_Vq_sat = 0.0f;          // Vq after circle limiter
volatile float debug_i2_max = 0.0f;           // Peak I² seen by I²t (LPF後)
volatile float debug_i2_avg = 0.0f;           // Running average I²
volatile float debug_pll_theta = 0.0f;        // PLL output angle [rad]
volatile float debug_hall_angle = 0.0f;       // Raw Hall angle [rad]
volatile float debug_pll_error_max = 0.0f;    // Peak |PLL phase error| [rad]
volatile float debug_iq_raw_max = 0.0f;       // Peak |Iq_raw| (LPF前)
volatile float debug_id_raw_max = 0.0f;       // Peak |Id_raw| (LPF前)
volatile float debug_ia_pu_at_spike = 0.0f;   // Ia_pu when I²_raw spike detected
volatile float debug_ib_pu_at_spike = 0.0f;   // Ib_pu when I²_raw spike detected
volatile uint16_t debug_ia_adc_at_spike = 0;  // Raw ADC count at spike
volatile uint16_t debug_ib_adc_at_spike = 0;  // Raw ADC count at spike
volatile uint8_t debug_encl_smooth = 0;       // 0=Open-loop, 1=Closed-loop enabled
volatile float debug_abs_speed_error = 0.0f;  // Absolute speed error for transition check
volatile uint8_t debug_foc_state = 0;         // 0=disabled, 1=active, 2=idq_zero_brake, 3=enable_off
volatile uint8_t debug_stop_reason = 0;       // Latched: 0=running, 1=enable_off, 2=idq_zero, 3=protection
volatile float debug_stop_iq = 0.0f;          // Iq at moment of stop
volatile float debug_stop_idqref = 0.0f;      // IdqRef[1] at moment of stop
volatile uint16_t debug_stop_rpm = 0;         // RPM at moment of stop
volatile uint8_t debug_enable_off_src = 0;    // Who set Enable=0: 1=drv_fault, 2=shutdown, 3=makita, 4=protection, 5=ble_cmd, 6=overcurrent

// ============================================================
// Sector-binned diagnostics: accumulate Id/Iq per Hall sector
// Read via BLE: GET sec0_id .. sec5_id, sec0_iq .. sec5_iq, sec0_n .. sec5_n
// Reset via BLE: SET sec_reset 1
// ============================================================
#define DIAG_SECTORS 6
volatile float   diag_id_sum[DIAG_SECTORS] = {0};
volatile float   diag_iq_sum[DIAG_SECTORS] = {0};
volatile uint32_t diag_count[DIAG_SECTORS] = {0};
volatile float   diag_id_avg[DIAG_SECTORS] = {0};  // computed on read
volatile float   diag_iq_avg[DIAG_SECTORS] = {0};
volatile float   diag_ia_sum[DIAG_SECTORS] = {0};   // Ialpha per sector
volatile float   diag_ib_sum[DIAG_SECTORS] = {0};   // Ibeta per sector
volatile float   diag_ia_avg[DIAG_SECTORS] = {0};
volatile float   diag_ib_avg[DIAG_SECTORS] = {0};
volatile uint8_t diag_sec_reset = 0;                // SET sec_reset 1 to clear

// ============================================================
// Software overcurrent protection (CRITICAL SAFETY)
// Monitors actual Id/Iq feedback current every 10kHz cycle
// Trips immediately if current exceeds limit
// ============================================================
#define OC_TRIP_PU        0.40f                // OC trip threshold: 0.40 PU = 6.6A
#define OC_GRACE_CYCLES   MS_TO_CYCLES(500)    // 500ms grace period after Enable
#define OC_SUSTAIN_CYCLES MS_TO_CYCLES(5)      // 5ms sustain filter
volatile uint8_t overcurrent_fault = 0;       // 0=OK, 1=tripped (latched, requires E0 to clear)
volatile float overcurrent_peak_iq = 0.0f;    // Peak |Iq| at trip [PU]
volatile float overcurrent_peak_id = 0.0f;    // Peak |Id| at trip [PU]
static uint16_t oc_grace_count = 0;            // Startup grace period counter (10kHz)
static uint16_t oc_sustain_count = 0;          // Consecutive over-threshold counter (10kHz)

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

// Hall sensor angle offset (adjustable via debugger or BLE P43=<value>)
volatile float hall_angle_offset = 0.20f;   // Calibrated offset (tunable via BLE)

// ============================================================
// Hall PLL: smooth angle estimation from coarse Hall edges
// Type-II PI PLL with feed-forward from speed estimate
// Solves: high-speed current oscillation due to 60°-step Hall angle
// ============================================================
static float pll_theta = 0.0f;            // PLL output angle [rad], [0, 2*pi)
static float pll_omega = 0.0f;            // PLL estimated elec. speed [rad/s]
static float pll_integrator = 0.0f;       // PI integrator state [rad/s]
volatile float pll_Kp = 50.0f;            // PLL proportional gain (wn=25, zeta=1.0)
volatile float pll_Ki = 625.0f;           // PLL integral gain (wn^2=625)
volatile uint8_t pll_enable = 0;          // 0=bypass (raw Hall), 1=PLL active
volatile uint8_t mode_change_pending = 0; // Set by C0/C1 command, cleared by FOC/speed loops after reset
static uint16_t torque_startup_ms = 0;   // Torque mode soft ramp counter (file scope for mode-change reset)
static uint8_t torque_was_enabled = 0;   // Torque mode startup detection (file scope for mode-change reset)
static float omega_e_filt = 0.0f;       // Decoupling LPF state (file scope for mode-change reset)
volatile float decouple_lpf_alpha = 0.006f * 10000.0f / (float)PWM_FREQ_HZ; // ~10Hz LPF (frequency-scaled)
volatile uint8_t decouple_enable = 1;       // 0=disable decoupling (debug), 1=enable
volatile float debug_Vd_decouple = 0.0f;    // Vd decoupling term (for diagnosis)
volatile float debug_Vq_decouple = 0.0f;    // Vq decoupling term (for diagnosis)

// Dead time compensation
// Triangle symmetric PWM: dead time affects both rising and falling edges
// Theoretical: 2 * t_dead / T_pwm = 2 * 500ns / T_pwm
// Empirical sweeps at M2000 (effective t_dead from data, FET delays + parasitics):
//   10kHz (T_pwm=100us): optimum 0.020 → effective t_dead ≈ 1.0us
//   18kHz (T_pwm=55.6us): optimum 0.045 → effective t_dead ≈ 1.25us
// Default below tracks 18kHz operation; if PWM_FREQ_HZ changes, re-sweep.
//
// Known trade-off (issue #19): high dtc_comp minimizes |I| at speed but
// causes audible chatter at M0+E1 (zero-current sign flips in the linear
// blend zone). For silent idle, SET dtc_comp 0 via BLE or detect M0/zero
// command and gate DTC. See doc/ソフト管理表.md for adaptive-DTC ideas.
//
// Phase mapping: Vabc_out[0]=C, [1]=B, [2]=A; Iab[0]=A, Iab[1]=B, Ic=-(Ia+Ib)
volatile float dtc_comp = 0.045f;           // 18kHz empirical optimum (re-sweep if PWM changes)
volatile uint8_t dtc_enable = 1;            // 0=disable, 1=enable
volatile float dtc_sign = 1.0f;            // +1 or -1: current polarity for DTC (BLE tunable)

// Angle advance compensation for computational delay
// FOC pipeline: ADC sample → compute → duty update → next PWM edge
// Total delay ≈ 1.0 × Ts_pwm = 50μs (center-aligned PWM, double-buffered)
// At high speed, this delay causes angle mismatch between measured current
// and applied voltage, leaking back-EMF into d-axis → excess current
// Compensation: advance hall_angle by ω_e × (angle_advance_us × 1e-6)
// Tunable via BLE: SET angle_advance_us <value>  (unit: microseconds)
volatile float angle_advance_us = 200.0f;  // Delay compensation [μs] (optimized: 200 = 2.0 × Ts_foc, |I|=0.071A@M2000)

// Auto-sweep for offset calibration (set cal_sweep_en=1 via debugger or BLE)
volatile uint8_t cal_sweep_en = 0;        // 1=sweeping, 0=idle
volatile uint8_t cal_sweep_idx = 0;       // current sweep index
volatile float cal_sweep_id_min = 99.0f;  // minimum |Id| found
volatile float cal_sweep_best_offset = 0.0f; // offset with minimum Id

// Power switch control variables
static uint32_t power_switch_high_counter = 0;  // Counter for PE14 high duration (in ms)
static uint8_t shutdown_requested = 0;          // Shutdown latch (once set, never cleared)

// Debug: DA14531 flow control pin states
volatile uint8_t debug_pe03_state = 0;          // PE03 (DA14531 RTS/P0_7): 0=LOW, 1=HIGH
volatile uint8_t debug_pe04_state = 0;          // PE04 (DA14531 CTS/P0_8): 0=LOW, 1=HIGH

// Diagnostic: confirm DA14531 firmware actually drives RTS (PE03 input).
// If pe03_edges stays 0 under heavy TX load (LOG START 100), DA14531 is NOT
// asserting flow control — adding HW CTS in FSP would have no effect.
// Sampled at main-loop rate (sub-microsecond when idle).
volatile uint32_t pe03_samples = 0;
volatile uint32_t pe03_highs   = 0;
volatile uint32_t pe03_edges   = 0;
volatile uint32_t pe04_samples = 0;
volatile uint32_t pe04_highs   = 0;
volatile uint32_t pe04_edges   = 0;
volatile uint8_t  pe_diag_reset = 0;            // write 1 via BLE to clear counters

// Debug variables for power switch (volatile for debugger visibility)
volatile uint8_t debug_pe14_state = 0;          // Current state of PE14 input (0=Low, 1=High)
volatile uint32_t debug_pe14_counter = 0;       // PE14 high duration counter (ms)
volatile uint8_t debug_shutdown_requested = 0;  // Shutdown state (0=normal, 1=shutting down)

// ============================================================
// Makita Battery AS Terminal
// PE01 → diode → AS: wake-up signal output
// P000 (AN016) ← 220kΩ/22kΩ divider ← AS: voltage detection
// Divider: AS_voltage = ADC_raw / 4095 * 3.3 * (220+22)/22
// ============================================================
#define AS_PIN_OUT       (BSP_IO_PORT_14_PIN_01)  // PE01: wake-up signal output
#define AS_DIVIDER_RATIO 11.0f                     // (220k+22k)/22k = 11.0
#define AS_PERMIT_THRESHOLD 5.0f                   // AS > 5V = discharge permitted
#define AS_PROHIBIT_DEBOUNCE_MS 15                 // >5ms periodic check, <24h timeout

volatile uint8_t makita_battery_en = 0;       // 0=disabled, 1=enabled
volatile uint8_t makita_battery_state = 0;    // 0=unknown, 1=permitted, 2=prohibited
volatile float as_voltage = 0.0f;             // AS terminal voltage [V]
volatile uint16_t as_adc_raw = 0;             // AS ADC raw value (AN016/P000)
static uint16_t as_low_counter = 0;           // Counter for AS LOW duration (ms)

// ============================================================
// LED output pin definitions (Active Low: MCU Low → LED ON)
// Hardware verified 2026-03-17
// ============================================================
#define LED_POWER_GREEN  (BSP_IO_PORT_13_PIN_15)  // PD15: Power LED Green
#define LED_POWER_RED    (BSP_IO_PORT_12_PIN_06)  // PC06: Power LED Red
#define LED_TURBO        (BSP_IO_PORT_12_PIN_07)  // PC07: Turbo LED
#define LED_ASSIST_H     (BSP_IO_PORT_13_PIN_01)  // PD01: Assist Level High
#define LED_ASSIST_M     (BSP_IO_PORT_13_PIN_02)  // PD02: Assist Level Middle
#define LED_ASSIST_L     (BSP_IO_PORT_13_PIN_03)  // PD03: Assist Level Low

// LED state: bit0=PwrGrn, bit1=PwrRed, bit2=Turbo, bit3=AssistH, bit4=AssistM, bit5=AssistL
// 0=OFF, 1=ON (driver handles active-low inversion)
// led_mode: 0=auto (base driver default), 1=manual (BLE/app control via led_state)
volatile uint8_t led_mode = 0;
volatile uint8_t led_state = 0;

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

// ff_scale: diagnostic only (no longer used in FOC — replaced by dq decoupling)
// Tracks |iq_cmd_final| / |iq_cmd_raw| ratio from speed/torque limiter
volatile float ff_scale = 1.0f;              // 0.0~1.0: diagnostic only

// JOG speed reference (settable via debugger or BLE)
volatile float speed_ref_rpm = 0.0f;         // Target speed [RPM]
volatile float speed_ramp_rate = 0.4f * PWM_TS_F;   // 0.4 PU/s = ~1900 RPM/s (frequency-scaled)

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
// I²t Motor Winding Thermal Protection (maxon EC45 flat 651614)
// ============================================================
// First-order thermal model: dE/dt = I² - E/τ
// At steady state: E_eq = I² × τ
// Trip when E > I_rated² × τ (= winding at 125°C)
//
// Datasheet values:
//   I_rated = 3.29A = 0.1994 PU ≈ 0.20 PU
//   τ_winding = 43.1 s
//   T_winding_max = 125°C
// ============================================================
#define I2T_I_RATED_PU    0.20f              // Rated continuous current [PU] (3.29A / 16.5A)
#define I2T_TAU_W         43.1f              // Winding thermal time constant [s]
// Threshold = I_rated² × τ = 0.04 × 43.1 = 1.724 PU²·s
// First-order thermal model: dE/dt = I² - E/τ → steady-state E = I²×τ
// At rated current (3.29A): ratio → 1.0 (asymptotic, never exceeds)
// Above rated: ratio > 1.0 → derating → trip
// Warning/Derating/Trip thresholds set above 1.0 so rated current is always allowed
#define I2T_THRESHOLD     (I2T_I_RATED_PU * I2T_I_RATED_PU * I2T_TAU_W)  // 0.04 × 43.1 = 1.724 PU²·s
#define I2T_WARNING_PCT   1.02f              // Warning at I > 3.33A continuous
#define I2T_DERATING_PCT  1.05f              // Derating at I > 3.37A continuous
#define I2T_TRIP_PCT      1.10f              // Trip at I > 3.45A continuous (~90s)
volatile float i2t_accumulator = 0.0f;       // Thermal energy accumulator [PU²·s]
volatile float i2t_ratio = 0.0f;             // Thermal load ratio (0.0~1.0+), 1.0=trip

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

// RX command queue: ISR assembles lines, main loop processes them
#define RX_CMD_SLOTS  4
#define RX_CMD_SIZE   64
static char rx_cmd_ring[RX_CMD_SLOTS][RX_CMD_SIZE];
static volatile uint8_t rx_cmd_head = 0;
static volatile uint8_t rx_cmd_tail = 0;
static volatile uint8_t rx_cmd_count = 0;
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

// TX debug
volatile uint8_t dbg_tx_last_err = 0xFF;   // last R_SCI_B_UART_Write return (0=OK)
volatile uint16_t dbg_tx_last_len = 0;     // last TX message length
volatile uint16_t dbg_tx_write_count = 0;  // number of R_SCI_B_UART_Write calls

// BLE log streaming
static volatile uint8_t ble_log_streaming = 0;      // 0=stopped, 1=running
static volatile uint16_t ble_log_interval = 100;     // ms (default 10Hz)
static uint16_t ble_log_counter = 0;

// LOG auto-stop: DA14531 SPS freezes after ~10 UART→BLE messages.
// Send N lines then auto-stop. MATLAB sends LOG START again to resume.
volatile uint8_t log_burst_max = 0;          // 0=continuous streaming (no burst limit)
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
    {"enable",              (void*)&Enable,              VAR_UINT8,  0},  // Read-only: use E0/E1 commands (OC fault check)
    {"control_mode",        (void*)&control_mode,        VAR_UINT8,  1},
    {"speed_ref_rpm",       (void*)&speed_ref_rpm,       VAR_FLOAT,  1},
    {"speed_ramp_rate",     (void*)&speed_ramp_rate,     VAR_FLOAT,  1},
    {"torque_ref_iq",       (void*)&torque_ref_iq,       VAR_FLOAT,  1},
    {"torque_ref_iq_max",   (void*)&torque_ref_iq_max,   VAR_FLOAT,  1},
    {"torque_max_speed",    (void*)&torque_max_speed_rpm, VAR_FLOAT, 1},
    {"speed_filter_alpha",  (void*)&speed_filter_alpha,  VAR_FLOAT,  1},
    {"vdc_compensation_en", (void*)&vdc_compensation_en, VAR_UINT8,  1},
    {"led_mode",            (void*)&led_mode,            VAR_UINT8,  1},
    {"led_state",           (void*)&led_state,           VAR_UINT8,  1},
    {"Kp_id",              (void*)&PI_params.Kp_id,      VAR_FLOAT,  1},
    {"Ki_id",              (void*)&PI_params.Ki_id,      VAR_FLOAT,  1},
    {"Kp_iq",              (void*)&PI_params.Kp_iq,      VAR_FLOAT,  1},
    {"Ki_iq",              (void*)&PI_params.Ki_iq,      VAR_FLOAT,  1},
    {"Kp_speed",           (void*)&PI_params.Kp_speed,   VAR_FLOAT,  1},
    {"Ki_speed",           (void*)&PI_params.Ki_speed,   VAR_FLOAT,  1},
    {"int_limit",           (void*)&integrator_limit,    VAR_FLOAT,  1},
    {"foc_state",           (void*)&debug_foc_state,     VAR_UINT8,  0},
    {"stop_reason",         (void*)&debug_stop_reason,   VAR_UINT8,  0},
    {"stop_iq",             (void*)&debug_stop_iq,       VAR_FLOAT,  0},
    {"stop_idqref",         (void*)&debug_stop_idqref,   VAR_FLOAT,  0},
    {"stop_rpm",            (void*)&debug_stop_rpm,      VAR_UINT8,  0},
    {"enable_off_src",      (void*)&debug_enable_off_src, VAR_UINT8, 0},
    {"oc_fault",            (void*)&overcurrent_fault,   VAR_UINT8,  0},
    {"oc_peak_iq",          (void*)&overcurrent_peak_iq, VAR_FLOAT,  0},
    {"oc_peak_id",          (void*)&overcurrent_peak_id, VAR_FLOAT,  0},
    {"makita_bat_en",       (void*)&makita_battery_en,   VAR_UINT8,  1},
    // Makita battery (read-only)
    {"as_voltage",          (void*)&as_voltage,          VAR_FLOAT,  0},
    {"makita_bat_state",    (void*)&makita_battery_state, VAR_UINT8, 0},
    {"i2t_ratio",           (void*)&i2t_ratio,           VAR_FLOAT,  0},
    {"ff_scale",            (void*)&ff_scale,            VAR_FLOAT,  0},
    {"hall_offset",         (void*)&hall_angle_offset,   VAR_FLOAT,  1},
    {"IdFb",                (void*)&IdFb,                VAR_FLOAT,  0},
    {"cal_sweep",           (void*)&cal_sweep_en,        VAR_UINT8,  1},
    {"cal_best_ofs",        (void*)&cal_sweep_best_offset, VAR_FLOAT, 0},
    {"i2_max",              (void*)&debug_i2_max,        VAR_FLOAT,  0},
    {"i2_avg",              (void*)&debug_i2_avg,        VAR_FLOAT,  0},
    {"iq_raw_max",          (void*)&debug_iq_raw_max,    VAR_FLOAT,  0},
    {"id_raw_max",          (void*)&debug_id_raw_max,    VAR_FLOAT,  0},
    {"ia_at_spike",         (void*)&debug_ia_pu_at_spike, VAR_FLOAT, 0},
    {"ib_at_spike",         (void*)&debug_ib_pu_at_spike, VAR_FLOAT, 0},
    {"pll_theta",           (void*)&debug_pll_theta,     VAR_FLOAT,  0},
    {"hall_angle",          (void*)&debug_hall_angle,     VAR_FLOAT,  0},
    {"pll_err_max",         (void*)&debug_pll_error_max, VAR_FLOAT,  0},
    {"Iq_integ",            (void*)&debug_Iq_integral,   VAR_FLOAT,  0},
    {"spd_integ",           (void*)&debug_speed_integ,   VAR_FLOAT,  0},
    {"decouple_alpha",      (void*)&decouple_lpf_alpha,  VAR_FLOAT,  1},
    {"pll_Kp",              (void*)&pll_Kp,              VAR_FLOAT,  1},
    {"pll_Ki",              (void*)&pll_Ki,              VAR_FLOAT,  1},
    {"pll_enable",          (void*)&pll_enable,          VAR_UINT8,  1},
    {"pll_omega",           (void*)&pll_omega,           VAR_FLOAT,  0},
    {"decouple_en",         (void*)&decouple_enable,     VAR_UINT8,  1},
    {"Vd_decouple",         (void*)&debug_Vd_decouple,   VAR_FLOAT,  0},
    {"Vq_decouple",         (void*)&debug_Vq_decouple,   VAR_FLOAT,  0},
    {"sec0_id",             (void*)&diag_id_avg[0],      VAR_FLOAT,  0},
    {"sec1_id",             (void*)&diag_id_avg[1],      VAR_FLOAT,  0},
    {"sec2_id",             (void*)&diag_id_avg[2],      VAR_FLOAT,  0},
    {"sec3_id",             (void*)&diag_id_avg[3],      VAR_FLOAT,  0},
    {"sec4_id",             (void*)&diag_id_avg[4],      VAR_FLOAT,  0},
    {"sec5_id",             (void*)&diag_id_avg[5],      VAR_FLOAT,  0},
    {"sec0_iq",             (void*)&diag_iq_avg[0],      VAR_FLOAT,  0},
    {"sec1_iq",             (void*)&diag_iq_avg[1],      VAR_FLOAT,  0},
    {"sec2_iq",             (void*)&diag_iq_avg[2],      VAR_FLOAT,  0},
    {"sec3_iq",             (void*)&diag_iq_avg[3],      VAR_FLOAT,  0},
    {"sec4_iq",             (void*)&diag_iq_avg[4],      VAR_FLOAT,  0},
    {"sec5_iq",             (void*)&diag_iq_avg[5],      VAR_FLOAT,  0},
    {"dtc_comp",            (void*)&dtc_comp,            VAR_FLOAT,  1},
    {"dtc_enable",          (void*)&dtc_enable,          VAR_UINT8,  1},
    {"dtc_sign",            (void*)&dtc_sign,            VAR_FLOAT,  1},
    {"angle_advance_us",    (void*)&angle_advance_us,    VAR_FLOAT,  1},
    {"sec0_ia",             (void*)&diag_ia_avg[0],      VAR_FLOAT,  0},
    {"sec1_ia",             (void*)&diag_ia_avg[1],      VAR_FLOAT,  0},
    {"sec2_ia",             (void*)&diag_ia_avg[2],      VAR_FLOAT,  0},
    {"sec3_ia",             (void*)&diag_ia_avg[3],      VAR_FLOAT,  0},
    {"sec4_ia",             (void*)&diag_ia_avg[4],      VAR_FLOAT,  0},
    {"sec5_ia",             (void*)&diag_ia_avg[5],      VAR_FLOAT,  0},
    {"sec0_ib",             (void*)&diag_ib_avg[0],      VAR_FLOAT,  0},
    {"sec1_ib",             (void*)&diag_ib_avg[1],      VAR_FLOAT,  0},
    {"sec2_ib",             (void*)&diag_ib_avg[2],      VAR_FLOAT,  0},
    {"sec3_ib",             (void*)&diag_ib_avg[3],      VAR_FLOAT,  0},
    {"sec4_ib",             (void*)&diag_ib_avg[4],      VAR_FLOAT,  0},
    {"sec5_ib",             (void*)&diag_ib_avg[5],      VAR_FLOAT,  0},
    {"sec0_n",              (void*)&diag_count[0],       VAR_UINT32, 0},
    {"sec_reset",           (void*)&diag_sec_reset,      VAR_UINT8,  1},
    // FOC ISR timing (DWT cycle counter, CPU 120MHz)
    {"isr_us_last",         (void*)&isr_us_last,         VAR_FLOAT,  0},
    {"isr_us_max",          (void*)&isr_us_max,          VAR_FLOAT,  0},
    {"isr_cyc_max",         (void*)&isr_cycles_max,      VAR_UINT32, 0},
    {"isr_max_reset",       (void*)&isr_max_reset,       VAR_UINT8,  1},
    // DA14531 flow-control pin diagnosis (PE03=DA14531 RTS, PE04=DA14531 CTS)
    {"pe03_samples",        (void*)&pe03_samples,        VAR_UINT32, 0},
    {"pe03_highs",          (void*)&pe03_highs,          VAR_UINT32, 0},
    {"pe03_edges",          (void*)&pe03_edges,          VAR_UINT32, 0},
    {"pe04_samples",        (void*)&pe04_samples,        VAR_UINT32, 0},
    {"pe04_highs",          (void*)&pe04_highs,          VAR_UINT32, 0},
    {"pe04_edges",          (void*)&pe04_edges,          VAR_UINT32, 0},
    {"pe_diag_reset",       (void*)&pe_diag_reset,       VAR_UINT8,  1},
};
#define VAR_REGISTRY_SIZE (sizeof(var_registry)/sizeof(var_registry[0]))

// LIST command state machine (sends one var per 1ms tick)
static int16_t list_send_idx = -1;  // -1=idle, 0+=current index
static int16_t dump_send_idx = -1;  // -1=idle, 0+=current index (D command)

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

 // Enable DWT cycle counter for FOC ISR timing measurement
 // (Cortex-M33 debug subsystem; 120MHz CPU clock → 1 cycle = 8.33ns)
 CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
 DWT->CYCCNT = 0;
 DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

 // Initialize Hall sensor FIRST (before ADC interrupts can call AngleSpeedGet)
 RM_MOTOR_SENSE_HALL_Open(&g_motor_angle0_ctrl, &g_motor_angle0_cfg);

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
 // Immediately disable PWM output pins (timer runs for ADC trigger, but FETs stay off)
 R_GPT_OutputDisable(&g_timer0_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
 R_GPT_OutputDisable(&g_timer1_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
 R_GPT_OutputDisable(&g_timer2_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);

 // Makita battery wake-up sequence (if enabled)
 if (makita_battery_en) {
     // Pulse PE01 HIGH for 10ms to wake battery from sleep
     R_IOPORT_PinWrite(&g_ioport_ctrl, AS_PIN_OUT, BSP_IO_LEVEL_HIGH);
     R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
     R_IOPORT_PinWrite(&g_ioport_ctrl, AS_PIN_OUT, BSP_IO_LEVEL_LOW);
     // Wait 60ms for battery response (spec: 50ms+)
     R_BSP_SoftwareDelay(60, BSP_DELAY_UNITS_MILLISECONDS);
 }

 // Reset DRV8302 gate driver — keep DISABLED until E1 command
 // EN_GATE LOW = FETs off, motor free-spinning
 R_IOPORT_PinWrite(&g_ioport_ctrl, EN_GATE_RESET, BSP_IO_LEVEL_LOW);
 R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);  // ensure reset completes

 // Set default ADC offset for current sensors (midpoint of 12-bit ADC = 2048)
 // This is the zero-current point for current sensors with VCC/2 bias
 // Default ADC offset (will be calibrated below)
 Iab_offset[0] = 2048;
 Iab_offset[1] = 2048;

 // Start ADC scanning
 R_ADC_B_ScanGroupStart(&g_adc0_ctrl, ADC_GROUP_MASK_0);
 R_ADC_B_ScanGroupStart(&g_adc0_ctrl, ADC_GROUP_MASK_2);

 // ADC offset calibration: average current sensor readings with motor stopped
 {
     uint32_t sum_a = 0, sum_b = 0;
     const int N_CAL = 500;  // 500ms (shorter than before)
     for (int i = 0; i < N_CAL; i++) {
         R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
         sum_a += Iab[0];
         sum_b += Iab[1];
     }
     Iab_offset[0] = (uint16_t)(sum_a / N_CAL);
     Iab_offset[1] = (uint16_t)(sum_b / N_CAL);
 }

 // PWM output starts DISABLED (FET off, motor free-spinning)
 // PWM is enabled on E1 command, disabled on E0 command
 // This prevents regenerative braking when motor is not controlled

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
        // Read DA14531 flow control pin states + diagnostic counters
        {
            static uint8_t pe03_prev = 0xFF;  // 0xFF = first iteration sentinel
            static uint8_t pe04_prev = 0xFF;
            bsp_io_level_t lvl;

            if (pe_diag_reset) {
                pe_diag_reset = 0;
                pe03_samples = 0; pe03_highs = 0; pe03_edges = 0;
                pe04_samples = 0; pe04_highs = 0; pe04_edges = 0;
                pe03_prev = 0xFF; pe04_prev = 0xFF;
            }

            R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_03, &lvl);
            debug_pe03_state = (lvl == BSP_IO_LEVEL_HIGH) ? 1 : 0;
            pe03_samples++;
            if (debug_pe03_state) pe03_highs++;
            if (pe03_prev != 0xFF && pe03_prev != debug_pe03_state) pe03_edges++;
            pe03_prev = debug_pe03_state;

            R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, &lvl);
            debug_pe04_state = (lvl == BSP_IO_LEVEL_HIGH) ? 1 : 0;
            pe04_samples++;
            if (debug_pe04_state) pe04_highs++;
            if (pe04_prev != 0xFF && pe04_prev != debug_pe04_state) pe04_edges++;
            pe04_prev = debug_pe04_state;
        }

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
        // tx_spacing_timer ensures DA14531 has time to forward previous message to BLE.
        // Byte-level pacing is handled by SCI3 HARDWARE_CTSRTS (PE03/PE04).
        if (tx_ring_count > 0 && !uart_tx_busy && tx_spacing_timer == 0) {
            uint16_t len = (uint16_t)strlen(tx_ring[tx_ring_tail]);
            if (len > 0) {
                uart_tx_busy = 1;
                fsp_err_t tx_err = R_SCI_B_UART_Write(&g_uart3_ctrl, (uint8_t *)tx_ring[tx_ring_tail], len);
                dbg_tx_last_err = (uint8_t)tx_err;
                dbg_tx_last_len = len;
                dbg_tx_write_count++;
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
    // PWM period derived from FSP timer config (single source of truth).
    // At 10kHz: 6000, 14kHz: 4286, 18kHz: 3333. Auto-tracks configuration.xml.
    const uint32_t PWM_PERIOD_COUNTS = g_timer0_cfg.period_counts;
    const float    PWMPeriod         = (float)PWM_PERIOD_COUNTS;
    three_phase_duty_cycle_t p_duty_cycle;
    // DRV8302 reset is now handled in hal_entry() before PWM output is enabled
    if(p_args->group_mask==ADC_GROUP_MASK_0)
    {
        adc_callback_count++;  // Debug counter
        // ISR timing: capture entry cycle count (DWT cycle counter)
        uint32_t isr_t0 = DWT->CYCCNT;
        if (isr_max_reset) { isr_cycles_max = 0; isr_us_max = 0.0f; isr_max_reset = 0; }
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
            Enable = 0; debug_enable_off_src = 1;  // DRV fault
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
                case 16: as_adc_raw = val; break;          // AN016 = AS terminal (Makita battery)
                default: break;
            }
        }

        // Convert battery ADC to voltage
        // V_bat = (ADC / 4095) * 3.3V * divider_ratio
        battery_voltage = ((float)battery_adc_raw / 4095.0f) * VBAT_ADC_VREF * VBAT_DIVIDER_RATIO;

        // Convert AS terminal ADC to voltage
        as_voltage = ((float)as_adc_raw / 4095.0f) * VBAT_ADC_VREF * AS_DIVIDER_RATIO;

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

        // ============================================================
        // JOG speed reference with rate limiter
        // speed_ref_rpm is set via debugger or BLE SET command.
        // Ramp SpeedRefIn_PU toward target at safe rate (10kHz call rate).
        // ============================================================
        {
            float target_pu = 0.0f;
            if (pmsm.N_base > 0.0f) {
                target_pu = speed_ref_rpm / pmsm.N_base;
            }
            // Clamp target to [-1, 1] PU (bidirectional)
            if (target_pu < -1.0f) target_pu = -1.0f;
            if (target_pu > 1.0f) target_pu = 1.0f;

            // Rate limiter: speed_ramp_rate PU/call at 10kHz
            // Default 0.00004 = 0.4 PU/s = ~1900 RPM/s (M0→M2000 in ~1s)
            float ramp_rate = speed_ramp_rate;
            if (SpeedRefIn_PU < target_pu) {
                SpeedRefIn_PU += ramp_rate;
                if (SpeedRefIn_PU > target_pu) SpeedRefIn_PU = target_pu;
            } else if (SpeedRefIn_PU > target_pu) {
                SpeedRefIn_PU -= ramp_rate;
                if (SpeedRefIn_PU < target_pu) SpeedRefIn_PU = target_pu;
            }
        }
        SpeedRef_RPM = SpeedRefIn_PU * pmsm.N_base;

        // Enable is controlled via debugger or BLE SET command (no auto-enable)

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
        angle_accumulated += angle_delta;  // kept for debug only
        debug_angle_unwrapped = angle_accumulated;

        // Step 2: Store delta in ring buffer, maintain running sum
        // (Avoids catastrophic cancellation from subtracting large accumulated angles)
        float old_delta = angle_history[angle_hist_idx];
        angle_history[angle_hist_idx] = angle_delta;
        angle_delta_sum += angle_delta - old_delta;
        angle_hist_idx++;
        if (angle_hist_idx >= SPEED_EST_WINDOW) {
            angle_hist_idx = 0;
            angle_hist_filled = 1;
        }

        // Step 3: Compute speed from sum of deltas over window
        float f_mechanical_speed = 0.0f;
        if (angle_hist_filled) {
            float electrical_speed = angle_delta_sum / ((float)SPEED_EST_WINDOW * PWM_TS_F);  // [rad/s elec]
            float mechanical_speed = electrical_speed / pmsm.p;                               // [rad/s mech]

            // Deadband: clamp near-zero speed to avoid noise at standstill
            if (mechanical_speed > -0.5f && mechanical_speed < 0.5f) {
                mechanical_speed = 0.0f;
            }
            f_mechanical_speed = mechanical_speed;
        } else {
            // Ring buffer not yet full: use FSP speed as fallback during startup
            if (isfinite(f_get_speed) && pmsm.p > 0.0f) {
                f_mechanical_speed = f_get_speed / pmsm.p;
                if (f_mechanical_speed > -0.5f && f_mechanical_speed < 0.5f) {
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
        // Use EnCl from speed controller (provides 1ms startup delay)
        // First 1ms after Enable: EnCl=0 → FOC outputs 50% (zero voltage)
        // After 1ms: SpeedControl sets EnCl=1 → PI starts gently
        EnCl_smooth = EnCl;
        #endif

        // Update debug variables for closed-loop transition monitoring
        debug_encl_smooth = EnCl_smooth;
        debug_abs_speed_error = abs_speed_error;

        // ============================================================
        // Hand-written FOC Current Controller (verified minimal version)
        // ============================================================
        {
            // ---- Hall PLL: smooth angle estimation ----
            float hall_raw = f_get_angle + hall_angle_offset;
            float hall_angle;
            debug_hall_angle = hall_raw;

            if (pll_enable) {
                // Phase detector: error = wrap(hall_raw - pll_theta)
                float phase_err = hall_raw - pll_theta;
                if (phase_err >  3.14159265f) phase_err -= 6.28318530f;
                if (phase_err < -3.14159265f) phase_err += 6.28318530f;

                // PI loop filter (no feed-forward: integrator tracks speed)
                pll_integrator += pll_Ki * PWM_TS_F * phase_err;
                // Anti-windup: clamp to ±2x max electrical speed (~4000RPM)
                if (pll_integrator >  8400.0f) pll_integrator =  8400.0f;
                if (pll_integrator < -8400.0f) pll_integrator = -8400.0f;

                pll_omega = pll_Kp * phase_err + pll_integrator;

                // VCO: integrate frequency to get angle
                pll_theta += pll_omega * PWM_TS_F;
                // Wrap to [0, 2*pi)
                if (pll_theta >= 6.28318530f) pll_theta -= 6.28318530f;
                if (pll_theta <  0.0f)        pll_theta += 6.28318530f;

                // Debug
                debug_pll_error_max = (phase_err > 0 ? phase_err : -phase_err) > debug_pll_error_max
                    ? (phase_err > 0 ? phase_err : -phase_err) : debug_pll_error_max;

                hall_angle = pll_theta;
            } else {
                // Bypass: raw Hall angle, seed PLL for seamless switch-on
                hall_angle = hall_raw;
                pll_theta = hall_raw;
                pll_integrator = SpeedFb_RPM * (2.0f * 3.14159265f / 60.0f) * pmsm.p;
                pll_omega = pll_integrator;
            }
            // Angle advance: compensate computational delay
            // omega_e_filt is the LPF'd electrical speed [rad/s] from decoupling
            // At M2000: advance = 1675 * 75μs = 0.126 rad (7.2°)
            // At M500:  advance = 419 * 75μs = 0.031 rad (1.8°)
            // hall_angle_offset was calibrated at M500 including its 1.8° delay,
            // so this advance corrects the ADDITIONAL delay at higher speeds.
            hall_angle += omega_e_filt * (angle_advance_us * 0.000001f);

            debug_pll_theta = hall_angle;

            float sin_theta = sinf(hall_angle);
            float cos_theta = cosf(hall_angle);

            float Ia_pu = ((float)Iab[0] - (float)Iab_offset[0]) * 0.00048828125f;
            float Ib_pu = ((float)Iab[1] - (float)Iab_offset[1]) * 0.00048828125f;

            float Ialpha = Ia_pu;
            float Ibeta  = (Ia_pu + 2.0f * Ib_pu) * 0.577350259f;

            float Id_raw =  Ialpha * cos_theta + Ibeta * sin_theta;
            float Iq_raw = Ibeta * cos_theta - Ialpha * sin_theta;

            // Sector-binned diagnostics: accumulate Id/Iq raw per Hall sector
            {
                if (diag_sec_reset) {
                    for (int s = 0; s < DIAG_SECTORS; s++) {
                        diag_id_sum[s] = 0; diag_iq_sum[s] = 0; diag_count[s] = 0;
                        diag_id_avg[s] = 0; diag_iq_avg[s] = 0;
                        diag_ia_sum[s] = 0; diag_ib_sum[s] = 0;
                        diag_ia_avg[s] = 0; diag_ib_avg[s] = 0;
                    }
                    diag_sec_reset = 0;
                }
                float angle_mod = f_get_angle;
                if (angle_mod < 0.0f) angle_mod += 6.28318530f;
                if (angle_mod >= 6.28318530f) angle_mod -= 6.28318530f;
                int sec = (int)(angle_mod * 0.954929658f);  // angle / (π/3) = angle * 3/π
                if (sec < 0) sec = 0;
                if (sec >= DIAG_SECTORS) sec = DIAG_SECTORS - 1;
                diag_id_sum[sec] += Id_raw;
                diag_iq_sum[sec] += Iq_raw;
                diag_ia_sum[sec] += Ialpha;
                diag_ib_sum[sec] += Ibeta;
                diag_count[sec]++;
            }

            // Debug: record RAW peak values and ADC data at spike
            {
                float iq_abs = Iq_raw > 0 ? Iq_raw : -Iq_raw;
                float id_abs = Id_raw > 0 ? Id_raw : -Id_raw;
                if (iq_abs > debug_iq_raw_max) debug_iq_raw_max = iq_abs;
                if (id_abs > debug_id_raw_max) debug_id_raw_max = id_abs;
                float i2_raw = Id_raw * Id_raw + Iq_raw * Iq_raw;
                if (i2_raw > 0.10f) {  // Spike threshold: > 0.32 PU = 5.2A
                    debug_ia_pu_at_spike = Ia_pu;
                    debug_ib_pu_at_spike = Ib_pu;
                    debug_ia_adc_at_spike = Iab[0];
                    debug_ib_adc_at_spike = Iab[1];
                }
            }

            static float Id_fb = 0.0f, Iq_fb_val = 0.0f;
            Id_fb     += 0.1f * (Id_raw - Id_fb);     // α=0.1 → fc≈159Hz
            Iq_fb_val += 0.1f * (Iq_raw - Iq_fb_val); // α=0.3 tested: oscillation 2x worse
            IqFb = Iq_fb_val;
            IdFb = Id_fb;
            debug_Ialpha = Ialpha;
            debug_Ibeta  = Ibeta;

            static float Id_integral = 0.0f, Iq_integral = 0.0f;
            static uint8_t foc_was_enabled = 0;

            // OC protection: check every 10kHz cycle (design v2 §3.1)
            // FOC-safe: zero IdqRef only, don't touch Enable/Vabc
            // Uses |I|² magnitude + 5ms sustain filter to ignore Hall spikes
            if (Enable) {
                oc_grace_count++;
                if (oc_grace_count > OC_GRACE_CYCLES) {
                    oc_grace_count = OC_GRACE_CYCLES + 1;  // prevent overflow
                    float I2_now = IqFb * IqFb + IdFb * IdFb;
                    if (I2_now > OC_TRIP_PU * OC_TRIP_PU) {
                        oc_sustain_count++;
                        if (oc_sustain_count >= OC_SUSTAIN_CYCLES) {
                            overcurrent_fault = 1;
                            float iq_abs = IqFb > 0 ? IqFb : -IqFb;
                            float id_abs = IdFb > 0 ? IdFb : -IdFb;
                            overcurrent_peak_iq = iq_abs;
                            overcurrent_peak_id = id_abs;
                            IdqRef[0] = 0.0f;
                            IdqRef[1] = 0.0f;
                        }
                    } else {
                        oc_sustain_count = 0;  // reset on below-threshold
                    }
                }
            } else {
                oc_grace_count = 0;   // reset grace on disable
                oc_sustain_count = 0;
            }

            if (Enable && EnCl_smooth) {
                // Reset FOC state on enable transition or mode change (C0↔C1)
                if (!foc_was_enabled || mode_change_pending) {
                    Id_integral = 0.0f;
                    Iq_integral = 0.0f;
                    omega_e_filt = 0.0f;
                    debug_i2_max = 0.0f;
                    debug_i2_avg = 0.0f;
                    debug_iq_raw_max = 0.0f;
                    debug_id_raw_max = 0.0f;
                    debug_pll_error_max = 0.0f;
                    // Reset PLL state to current Hall angle, seed speed for instant lock
                    pll_theta = f_get_angle + hall_angle_offset;
                    pll_integrator = SpeedFb_RPM * (2.0f * 3.14159265f / 60.0f) * pmsm.p;
                    pll_omega = pll_integrator;
                    foc_was_enabled = 1;
                }

                // OC fault: reset PI integrals to stop stall current
                if (overcurrent_fault) {
                    Id_integral = 0.0f;
                    Iq_integral = 0.0f;
                }

                // ============================================================
                // dq-axis cross-coupling decoupling (replaces old Vq_ff_lpf)
                // Compensates rotational EMF: ωe×L×I cross terms + back-EMF
                // Without this, Iq leaks into Id at speed → large Id, heating
                // omega_e is low-pass filtered (~10Hz) to prevent speed
                // estimation noise from injecting voltage disturbance
                // ============================================================
                float omega_e_raw = SpeedFb_RPM * (2.0f * 3.14159265f / 60.0f) * pmsm.p;
                omega_e_filt += decouple_lpf_alpha * (omega_e_raw - omega_e_filt);
                float omega_e = omega_e_filt;
                float Lq_pu = pmsm.Lq * inverter.ISenseMax * 1.7320508f / inverter.V_dc;
                float Ld_pu = pmsm.Ld * inverter.ISenseMax * 1.7320508f / inverter.V_dc;
                float FluxPM_pu = pmsm.FluxPM * 1.7320508f / inverter.V_dc;

                float Vd_decouple, Vq_decouple;
                if (decouple_enable) {
                    Vd_decouple = -omega_e * Lq_pu * Iq_fb_val;
                    Vq_decouple = +omega_e * Ld_pu * Id_fb + omega_e * FluxPM_pu;
                } else {
                    Vd_decouple = 0.0f;
                    Vq_decouple = 0.0f;
                }
                debug_Vd_decouple = Vd_decouple;
                debug_Vq_decouple = Vq_decouple;

                float Id_error = IdqRef[0] - Id_fb;
                float Iq_error = IdqRef[1] - Iq_fb_val;
                float Vd = PI_params.Kp_id * Id_error + Id_integral + Vd_decouple;
                float Vq = PI_params.Kp_iq * Iq_error + Iq_integral + Vq_decouple;

                // Circle limiter with back-calculation anti-windup
                float Vd_sat = Vd, Vq_sat = Vq;
                float Vmag2 = Vd * Vd + Vq * Vq;
                if (Vmag2 > 0.9025f) {
                    float inv_mag = 0.95f / sqrtf(Vmag2);
                    Vd_sat = Vd * inv_mag;
                    Vq_sat = Vq * inv_mag;
                    // Back-calculation: reduce integrator when voltage saturates
                    float Kb = 1.0f / PI_params.Kp_id;  // ≈2.3
                    Id_integral += Kb * (Vd_sat - Vd);
                    Iq_integral += Kb * (Vq_sat - Vq);
                }

                // Integrator update with clamping anti-windup
                float int_lim = integrator_limit;
                if (PI_params.Ki_id > 0.0f) {
                    Id_integral += PI_params.Ki_id * PWM_TS_F * Id_error;
                    if (Id_integral >  int_lim) Id_integral =  int_lim;
                    if (Id_integral < -int_lim) Id_integral = -int_lim;
                }
                if (PI_params.Ki_iq > 0.0f) {
                    Iq_integral += PI_params.Ki_iq * PWM_TS_F * Iq_error;
                    if (Iq_integral >  int_lim) Iq_integral =  int_lim;
                    if (Iq_integral < -int_lim) Iq_integral = -int_lim;
                }
                debug_Iq_integral = Iq_integral;
                debug_Vmag2 = Vmag2;
                debug_Vd_sat = Vd_sat;
                debug_Vq_sat = Vq_sat;

                // ============================================================
                // Real-time current magnitude limiter
                // Protects motor when FOC angle is inaccurate (stall, mech limit)
                // If measured |I| > I_rated, scale down voltage to keep current safe
                // Works regardless of dq alignment — uses raw measured current
                // ============================================================
                {
                    float I2_meas = Ialpha * Ialpha + Ibeta * Ibeta;  // |I|² from Clarke (correct magnitude)
                    // Limit at 2× rated (6.58A) — prevents catastrophic overcurrent from angle error
                    // Rated current protection is handled by i2t (thermal) and IqRef saturation
                    // Setting too low (=I_rated) kills torque at low speed due to Hall angle error
                    float I2_limit = 4.0f * I2T_I_RATED_PU * I2T_I_RATED_PU;  // 0.16 (= 6.58A²)
                    if (I2_meas > I2_limit) {
                        // Scale voltage so current stays at limit
                        // Use sqrt for accurate scaling, runs only when overlimit
                        float scale = I2T_I_RATED_PU / sqrtf(I2_meas);
                        Vd_sat *= scale;
                        Vq_sat *= scale;
                        // Pull back integrators to prevent windup during limiting
                        Id_integral *= scale;
                        Iq_integral *= scale;
                    }
                }

                float Valpha = Vd_sat * cos_theta - Vq_sat * sin_theta;
                float Vbeta  = Vq_sat * cos_theta + Vd_sat * sin_theta;

                float Va = Valpha;
                float Vb = 0.866025f * Vbeta - 0.5f * Valpha;
                float Vc = -0.5f * Valpha - 0.866025f * Vbeta;
                float Vmax = Va; if (Vb > Vmax) Vmax = Vb; if (Vc > Vmax) Vmax = Vc;
                float Vmin = Va; if (Vb < Vmin) Vmin = Vb; if (Vc < Vmin) Vmin = Vc;
                float Voff = -0.5f * (Vmax + Vmin);
                Vabc_out[0] = 0.5f * (Va + Voff) * 1.15470052f + 0.5f;
                Vabc_out[1] = 0.5f * (Vb + Voff) * 1.15470052f + 0.5f;
                Vabc_out[2] = 0.5f * (Vc + Voff) * 1.15470052f + 0.5f;
                // Dead time compensation: correct duty based on current direction
                // Positive current → actual voltage is lower → add compensation
                // Negative current → actual voltage is higher → subtract compensation
                // Dead time compensation (6-PWM, triangle symmetric)
                // Positive current: actual voltage < commanded → increase duty
                // Phase mapping: Vabc_out[0]=C, [1]=B, [2]=A
                // Use filtered dq currents → inverse Park for stable phase current estimate
                // Linear interpolation near zero crossing (SPRAA87 method):
                //   |I| > threshold: full ±dtc_comp
                //   |I| < threshold: dtc_comp * I / threshold (smooth transition)
                if (dtc_enable) {
                    // Use measured physical phase currents directly (not inverse Park)
                    // Avoids phase mapping ambiguity between Clarke/SVPWM conventions
                    // Ia_pu, Ib_pu are raw ADC measurements (physical A, B phase currents)
                    // dtc_sign: +1 or -1, adjustable via BLE to match ADC polarity
                    float Ia_dtc = Ia_pu * dtc_sign;
                    float Ib_dtc = Ib_pu * dtc_sign;
                    float Ic_dtc = -(Ia_dtc + Ib_dtc);
                    float c = dtc_comp;
                    float thr = 0.01f;  // linear zone: ±0.01 PU (±0.165A)
                    float inv_thr = 1.0f / thr;
                    // Linear ramp: saturates at ±c, smooth through zero (SPRAA87)
                    float da = Ia_dtc * inv_thr; if (da > 1.0f) da = 1.0f; if (da < -1.0f) da = -1.0f;
                    float db = Ib_dtc * inv_thr; if (db > 1.0f) db = 1.0f; if (db < -1.0f) db = -1.0f;
                    float dc = Ic_dtc * inv_thr; if (dc > 1.0f) dc = 1.0f; if (dc < -1.0f) dc = -1.0f;
                    Vabc_out[2] += c * da;  // A相
                    Vabc_out[1] += c * db;  // B相
                    Vabc_out[0] += c * dc;  // C相
                }
            } else {
                Vabc_out[0] = 0.5f; Vabc_out[1] = 0.5f; Vabc_out[2] = 0.5f;
                foc_was_enabled = 0;
            }

            Mode = 2.0f;
        }

        // ============================================================
        // Data Logger: Record data at 100Hz (every 100 PWM cycles)
        // ============================================================
        if(log_running) {
            log_counter++;
            if(log_counter >= MS_TO_CYCLES(10)) {  // log at 100Hz (every 10ms)
                log_counter = 0;

                // Record current state
                log_buffer[log_index].SpeedRef = SpeedRefIn_PU;
                log_buffer[log_index].SpeedFb = SpeedFb_PU;
                log_buffer[log_index].IqFb = IqFb;
                log_buffer[log_index].IdqRef1 = IdqRef[1];
                log_buffer[log_index].EnCl = EnCl_smooth;

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
            if (vdc_scale > 1.5f) vdc_scale = 1.5f;
            Vabc_out[0] = 0.5f + (Vabc_out[0] - 0.5f) * vdc_scale;
            Vabc_out[1] = 0.5f + (Vabc_out[1] - 0.5f) * vdc_scale;
            Vabc_out[2] = 0.5f + (Vabc_out[2] - 0.5f) * vdc_scale;
            for (int i = 0; i < 3; i++) {
                if (Vabc_out[i] < 0.02f) Vabc_out[i] = 0.02f;
                if (Vabc_out[i] > 0.98f) Vabc_out[i] = 0.98f;
            }
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

        // Clamp to valid range [0, period_counts] (frequency-tracking)
        if(p_duty_cycle.duty[0] > PWM_PERIOD_COUNTS) p_duty_cycle.duty[0] = PWM_PERIOD_COUNTS;
        if(p_duty_cycle.duty[1] > PWM_PERIOD_COUNTS) p_duty_cycle.duty[1] = PWM_PERIOD_COUNTS;
        if(p_duty_cycle.duty[2] > PWM_PERIOD_COUNTS) p_duty_cycle.duty[2] = PWM_PERIOD_COUNTS;

        // Store debug values
        debug_vabc[0] = Vabc_out[0];
        debug_vabc[1] = Vabc_out[1];
        debug_vabc[2] = Vabc_out[2];
        debug_duty[0] = p_duty_cycle.duty[0];
        debug_duty[1] = p_duty_cycle.duty[1];
        debug_duty[2] = p_duty_cycle.duty[2];

        R_GPT_THREE_PHASE_DutyCycleSet(&g_three_phase0_ctrl, &p_duty_cycle);

        // ISR timing: capture exit cycle count and update last/max
        uint32_t isr_dt = DWT->CYCCNT - isr_t0;
        isr_cycles_last = isr_dt;
        isr_us_last = (float)isr_dt * (1.0f / 120.0f);  // 120MHz CPU
        if (isr_dt > isr_cycles_max) {
            isr_cycles_max = isr_dt;
            isr_us_max = isr_us_last;
        }
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

    } else if (strcmp(cmd, "GETALL") == 0) {
do_getall:
        // Return all writable variables in one response (avoids BLE byte-drop on multiple GETs)
        int pos = 0;
        pos += snprintf(resp + pos, sizeof(resp) - pos, "ALL ");
        for (uint16_t i = 0; i < VAR_REGISTRY_SIZE; i++) {
            if (!var_registry[i].writable) continue;
            if (pos >= (int)sizeof(resp) - 40) break;  // safety margin
            switch (var_registry[i].type) {
                case VAR_FLOAT: {
                    char vbuf[16];
                    fmt_float(vbuf, sizeof(vbuf), *(volatile float*)var_registry[i].ptr, 4);
                    pos += snprintf(resp + pos, sizeof(resp) - pos, "%s=%s,", var_registry[i].name, vbuf);
                    break;
                }
                case VAR_UINT8:
                    pos += snprintf(resp + pos, sizeof(resp) - pos, "%s=%u,",
                                   var_registry[i].name, (unsigned)*(volatile uint8_t*)var_registry[i].ptr);
                    break;
                case VAR_UINT32:
                    pos += snprintf(resp + pos, sizeof(resp) - pos, "%s=%lu,",
                                   var_registry[i].name, (unsigned long)*(volatile uint32_t*)var_registry[i].ptr);
                    break;
            }
        }
        if (pos > 4 && resp[pos-1] == ',') resp[pos-1] = '\0';  // remove trailing comma
        strncat(resp, "\r\n", sizeof(resp) - strlen(resp) - 1);
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
        // Always restart unconditionally (no guard — MATLAB sends single R per burst)
        ble_log_streaming = 1;
        ble_log_counter = 0;
        log_burst_sent = 0;
        pack_idx = 0;

    } else if (cmd[0] == 'W' && (cmd[1] >= '0' && cmd[1] <= '9')) {
        // Short max speed set: "W500" = torque_max_speed_rpm = 500 (BLE-safe)
        torque_max_speed_rpm = (float)atoi(cmd + 1);
        if (torque_max_speed_rpm > 4500.0f) torque_max_speed_rpm = 4500.0f;

    } else if (cmd[0] == 'G' && cmd[1] >= '0' && cmd[1] <= '9') {
        // GET by index: "G26" → "#26=0.5000\r\n" (short, BLE-safe)
        uint16_t idx = (uint16_t)atoi(cmd + 1);
        if (idx < VAR_REGISTRY_SIZE) {
            const var_entry_t *v = &var_registry[idx];
            char resp[32];
            if (v->type == VAR_FLOAT) {
                char vbuf[12];
                fmt_float(vbuf, sizeof(vbuf), *(volatile float*)v->ptr, 4);
                snprintf(resp, sizeof(resp), "#%u=%s\r\n", idx, vbuf);
            } else {
                snprintf(resp, sizeof(resp), "#%u=%u\r\n", idx, (unsigned)(*(volatile uint8_t*)v->ptr));
            }
            uart_queue(resp);
        }

    } else if (cmd[0] == 'P' && cmd[1] >= '0' && cmd[1] <= '9') {
        // SET by index: "P26=0.5" → "#26=0.5000\r\n" (short, BLE-safe)
        // Format: P<idx>=<value>
        char *eq = strchr(cmd, '=');
        if (eq) {
            uint16_t idx = (uint16_t)atoi(cmd + 1);
            if (idx < VAR_REGISTRY_SIZE && var_registry[idx].writable) {
                const var_entry_t *v = &var_registry[idx];
                char resp[32];
                if (v->type == VAR_FLOAT) {
                    float val = (float)atof(eq + 1);
                    // Safety limits for critical parameters (design v2 §3.5)
                    if (v->ptr == (void*)&torque_ref_iq_max) {
                        if (val > 0.30f) val = 0.30f;   // rated current limit
                        if (val < 0.0f) val = 0.0f;
                    } else if (v->ptr == (void*)&torque_ref_iq) {
                        if (val > 0.30f) val = 0.30f;   // same as iq_max
                        if (val < -0.30f) val = -0.30f;
                    } else if (v->ptr == (void*)&PI_params.Ki_id || v->ptr == (void*)&PI_params.Ki_iq) {
                        if (val > 50.0f) val = 50.0f;
                        if (val < 0.0f) val = 0.0f;
                    } else if (v->ptr == (void*)&PI_params.Kp_id || v->ptr == (void*)&PI_params.Kp_iq) {
                        if (val > 10.0f) val = 10.0f;
                        if (val < 0.0f) val = 0.0f;
                    } else if (v->ptr == (void*)&PI_params.Kp_speed || v->ptr == (void*)&PI_params.Ki_speed) {
                        if (val > 10.0f) val = 10.0f;
                        if (val < 0.0f) val = 0.0f;
                    } else if (v->ptr == (void*)&hall_angle_offset) {
                        if (val > 1.0f) val = 1.0f;    // ±1 rad ≈ ±57°
                        if (val < -1.0f) val = -1.0f;
                    } else if (v->ptr == (void*)&torque_max_speed_rpm) {
                        if (val > 4000.0f) val = 4000.0f;  // below RPM limit (4500)
                        if (val < 0.0f) val = 0.0f;
                    } else if (v->ptr == (void*)&speed_filter_alpha) {
                        if (val > 1.0f) val = 1.0f;
                        if (val < 0.001f) val = 0.001f;
                    }
                    *(volatile float*)v->ptr = val;
                    char vbuf[12];
                    fmt_float(vbuf, sizeof(vbuf), val, 4);
                    snprintf(resp, sizeof(resp), "#%u=%s\r\n", idx, vbuf);
                } else {
                    uint8_t val = (uint8_t)atoi(eq + 1);
                    *(volatile uint8_t*)v->ptr = val;
                    snprintf(resp, sizeof(resp), "#%u=%u\r\n", idx, val);
                }
                uart_queue(resp);
            }
        }

    } else if (cmd[0] == '?' && cmd[1] == 'M') {
        // Short speed query: "?M" → "=300\r\n" (6 bytes, BLE-safe verify)
        char resp[16];
        snprintf(resp, sizeof(resp), "=%d\r\n", (int)speed_ref_rpm);
        uart_queue(resp);

    } else if (cmd[0] == '?' && cmd[1] == 'T') {
        // Short torque query: "?T" → "=0.100\r\n"
        char resp[16];
        char val[12];
        fmt_float(val, sizeof(val), torque_ref_iq, 3);
        snprintf(resp, sizeof(resp), "=%s\r\n", val);
        uart_queue(resp);

    } else if (cmd[0] == 'F' && cmd[1] >= '0' && cmd[1] <= '9') {
        // Short frequency set: "F100" = 100Hz, "F1" = 1Hz (no restart)
        uint16_t hz = (uint16_t)atoi(cmd + 1);
        if (hz == 0) hz = 1;
        if (hz > 100) hz = 100;
        ble_log_interval = 1000 / hz;

    } else if (cmd[0] == 'M' && (cmd[1] == '-' || (cmd[1] >= '0' && cmd[1] <= '9'))) {
        // Short speed set: "M100" = SET speed_ref_rpm 100 (≤6 bytes, BLE-safe)
        speed_ref_rpm = (float)atoi(cmd + 1);
        if (speed_ref_rpm > 4400.0f) speed_ref_rpm = 4400.0f;
        if (speed_ref_rpm < -4400.0f) speed_ref_rpm = -4400.0f;

    } else if (cmd[0] == 'T' && (cmd[1] == '-' || (cmd[1] >= '0' && cmd[1] <= '9'))) {
        // Short torque set: "T18" = 18/1000 = 0.018 PU (integer milliPU, BLE-safe)
        torque_ref_iq = (float)atoi(cmd + 1) * 0.001f;
        // Hard safety clamp at 0.30 PU (4.95A) regardless of torque_ref_iq_max
        if (torque_ref_iq > 0.30f) torque_ref_iq = 0.30f;
        if (torque_ref_iq < -0.30f) torque_ref_iq = -0.30f;
        if (torque_ref_iq > torque_ref_iq_max) torque_ref_iq = torque_ref_iq_max;
        if (torque_ref_iq < -torque_ref_iq_max) torque_ref_iq = -torque_ref_iq_max;

    } else if (cmd[0] == 'E' && (cmd[1] == '0' || cmd[1] == '1')) {
        // Short enable: "E1" = enable, "E0" = disable (2 bytes, BLE-safe)
        if (cmd[1] == '1') {
            if (overcurrent_fault) {
                // Don't enable while overcurrent fault is latched
                // Send "E0" first to clear the fault, then "E1" to enable
            } else {
                // Reset FSP Hall module to clear stale CW-biased state
                // Without this, CCW startup fails (angle stuck in CW direction)
                RM_MOTOR_SENSE_HALL_Reset(&g_motor_angle0_ctrl);
                // Clear diagnostic latches for fresh tracking
                debug_enable_off_src = 0;
                debug_stop_reason = 0;
                // Re-calibrate ADC offset (EN_GATE still OFF = zero current)
                // Prevents drift from current sensor temperature change
                {
                    uint32_t sum_a = 0, sum_b = 0;
                    const int N = 100;  // 100ms
                    for (int i = 0; i < N; i++) {
                        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
                        sum_a += Iab[0];
                        sum_b += Iab[1];
                    }
                    Iab_offset[0] = (uint16_t)(sum_a / N);
                    Iab_offset[1] = (uint16_t)(sum_b / N);
                }
                // Reset speed estimation state (prevent stale deltas from previous run)
                for (int i = 0; i < SPEED_EST_WINDOW; i++) angle_history[i] = 0.0f;
                angle_hist_idx = 0;
                angle_hist_filled = 0;
                angle_delta_sum = 0.0f;
                prev_angle_for_unwrap = f_get_angle;  // sync to current FSP angle
                angle_accumulated = 0.0f;

                // Re-enable DRV8302 gate driver + PWM output
                R_IOPORT_PinWrite(&g_ioport_ctrl, EN_GATE_RESET, BSP_IO_LEVEL_HIGH);
                R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
                R_GPT_OutputEnable(&g_timer0_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
                R_GPT_OutputEnable(&g_timer1_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
                R_GPT_OutputEnable(&g_timer2_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
                Enable = 1;
            }
        } else {
            Enable = 0;
            overcurrent_fault = 0;  // E0 clears the overcurrent fault
            // Disable PWM output + DRV8302 gate driver to fully release motor
            R_GPT_OutputDisable(&g_timer0_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
            R_GPT_OutputDisable(&g_timer1_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
            R_GPT_OutputDisable(&g_timer2_ctrl, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
            R_IOPORT_PinWrite(&g_ioport_ctrl, EN_GATE_RESET, BSP_IO_LEVEL_LOW);
            // Reset I²t to rated steady-state level — motor is still warm after trip
            // ratio=1.0 = rated current equivalent → derating on re-enable if above rated
            if (i2t_accumulator > I2T_THRESHOLD) {
                i2t_accumulator = I2T_THRESHOLD;
            }
            debug_enable_off_src = 5;
        }

    } else if (cmd[0] == 'L' && (cmd[1] >= '0' && cmd[1] <= '9')) {
        // Short Iq limit: "L60" = 60/1000 = 0.06 PU (torque_ref_iq_max)
        torque_ref_iq_max = (float)atoi(cmd + 1) * 0.001f;
        if (torque_ref_iq_max > 0.30f) torque_ref_iq_max = 0.30f;  // hard safety cap
        if (torque_ref_iq_max < 0.01f) torque_ref_iq_max = 0.01f;

    } else if (cmd[0] == 'C' && (cmd[1] == '0' || cmd[1] == '1')) {
        // Short control mode: "C0" = speed, "C1" = torque (2 bytes, BLE-safe)
        control_mode = (uint8_t)(cmd[1] - '0');
        mode_change_pending = 1;  // trigger state reset in FOC/speed loops

    } else if (strcmp(cmd, "A") == 0) {
        // Short alias for GETALL (1 byte = reliable over BLE)
        goto do_getall;

    } else if (strcmp(cmd, "D") == 0) {
        // Compute sector averages before dump
        for (int s = 0; s < DIAG_SECTORS; s++) {
            if (diag_count[s] > 0) {
                float inv_n = 1.0f / (float)diag_count[s];
                diag_id_avg[s] = diag_id_sum[s] * inv_n;
                diag_iq_avg[s] = diag_iq_sum[s] * inv_n;
                diag_ia_avg[s] = diag_ia_sum[s] * inv_n;
                diag_ib_avg[s] = diag_ib_sum[s] * inv_n;
            } else {
                diag_id_avg[s] = 0; diag_iq_avg[s] = 0;
                diag_ia_avg[s] = 0; diag_ib_avg[s] = 0;
            }
        }
        // DUMP: send all var_registry values as V<idx>=<value> (1 per ms tick)
        // Uses reliable MCU→MATLAB notification path, ~35ms for all params
        dump_send_idx = 0;

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

    // Timeout: if bytes in buffer and no CR/LF for 200ms, treat as complete
    // (nRF Connect / BLE apps don't always send CR/LF after text)
    if (uart_rx_idx > 0) {
        uart_rx_idle_ms++;
        if (uart_rx_idle_ms >= 200) {
            if (rx_cmd_count < RX_CMD_SLOTS) {
                uint16_t copy_len = uart_rx_idx < RX_CMD_SIZE - 1 ? uart_rx_idx : RX_CMD_SIZE - 1;
                memcpy(rx_cmd_ring[rx_cmd_head], uart_rx_buf, copy_len);
                rx_cmd_ring[rx_cmd_head][copy_len] = '\0';
                rx_cmd_head = (rx_cmd_head + 1) % RX_CMD_SLOTS;
                rx_cmd_count++;
            }
            uart_rx_idx = 0;
            uart_rx_idle_ms = 0;
            dbg_cmd_timeout_count++;
        }
    }

    // Process received commands from ring buffer (up to 1 per ms tick)
    if (rx_cmd_count > 0) {
        char cmd_copy[RX_CMD_SIZE];
        memcpy(cmd_copy, rx_cmd_ring[rx_cmd_tail], RX_CMD_SIZE);
        rx_cmd_tail = (rx_cmd_tail + 1) % RX_CMD_SLOTS;
        rx_cmd_count--;
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

    // DUMP: send one V<idx>=<value> every 5ms (triggered by "D" command)
    if (dump_send_idx >= 0) {
        static uint8_t dump_tick2 = 0;
        dump_tick2++;
        if (dump_tick2 >= 5 && tx_ring_count < TX_RING_SLOTS) {
            dump_tick2 = 0;
            if (dump_send_idx < (int16_t)VAR_REGISTRY_SIZE) {
                char resp[32];
                const var_entry_t *v = &var_registry[dump_send_idx];
                if (v->type == VAR_FLOAT) {
                    char vbuf[12];
                    fmt_float(vbuf, sizeof(vbuf), *(volatile float*)v->ptr, 4);
                    snprintf(resp, sizeof(resp), "V%d=%s\r\n", dump_send_idx, vbuf);
                } else {
                    snprintf(resp, sizeof(resp), "V%d=%u\r\n", dump_send_idx,
                             (unsigned)(*(volatile uint8_t*)v->ptr));
                }
                uart_queue(resp);
                dump_send_idx++;
            } else {
                dump_send_idx = -1;
                uart_queue("VEND\r\n");
            }
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
                    pos += snprintf(resp + pos, sizeof(resp) - (size_t)pos,
                             ",%d,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                             (int)(IqFb * 1000),
                             (int)(battery_voltage * 10 + 0.5f),
                             (int)(imu_accel_g[0] * 1000),
                             (int)(imu_accel_g[1] * 1000),
                             (int)(imu_accel_g[2] * 1000),
                             (int)(imu_gyro_dps[0] * 10),
                             (int)(imu_gyro_dps[1] * 10),
                             (int)(imu_gyro_dps[2] * 10),
                             (int)(IdFb * 1000),
                             (int)Enable,
                             (int)protection_state,
                             (int)(i2t_ratio * 100),
                             (int)(debug_Iq_integral * 1000),
                             (int)(IdqRef[1] * 1000),
                             (int)(IdqRef[0] * 1000),
                             (int)(debug_speed_integ * 1000),
                             (int)(debug_Vmag2 * 1000));
                    // XOR checksum of payload (B...data), append *XX\r\n
                    {
                        uint8_t csum = 0;
                        for (int ci = 0; ci < pos; ci++) csum ^= (uint8_t)resp[ci];
                        snprintf(resp + pos, sizeof(resp) - (size_t)pos, "*%02X\r\n", csum);
                    }
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

    // Hall angle offset auto-sweep calibration (2-phase: coarse + fine)
    // Phase 1: current_offset ± 0.20 in 0.05 rad steps (9 values)
    // Phase 2: best ±0.05 in 0.005 rad steps (21 values)
    // Total: (9+21) × 1.0s = 30 seconds
    if (cal_sweep_en && Enable) {
        static uint16_t cal_timer = 0;
        static float cal_id_sum = 0;
        static uint32_t cal_id_count = 0;
        static uint8_t cal_phase = 0;  // 0=coarse, 1=fine

        // Coarse offsets: generated at runtime around current offset ± 0.20 rad
        static float cal_coarse[9];    // center + (-4..+4)*0.05
        static uint8_t cal_coarse_init = 0;
        #define CAL_COARSE_NUM   9
        // Fine offsets: generated at runtime around coarse best
        static float cal_fine[21];     // center + (-10..+10)*0.005
        #define CAL_FINE_NUM     21
        #define CAL_DWELL_MS     1000  // 1.0s per offset (500ms settle + 500ms measure)

        // Initialize coarse array on first entry (centered on current offset)
        if (!cal_coarse_init) {
            float center = hall_angle_offset;
            for (int i = 0; i < CAL_COARSE_NUM; i++) {
                cal_coarse[i] = center + (float)(i - 4) * 0.05f;
            }
            cal_coarse_init = 1;
            hall_angle_offset = cal_coarse[0];
        }

        cal_timer++;
        // Accumulate |Id| during dwell (skip first 500ms for settling)
        if (cal_timer > 500) {
            float id_abs = IdFb > 0 ? IdFb : -IdFb;
            cal_id_sum += id_abs;
            cal_id_count++;
        }

        if (cal_timer >= CAL_DWELL_MS) {
            // Evaluate this offset
            float id_avg = (cal_id_count > 0) ? cal_id_sum / (float)cal_id_count : 99.0f;
            if (id_avg < cal_sweep_id_min) {
                cal_sweep_id_min = id_avg;
                cal_sweep_best_offset = hall_angle_offset;  // current offset being tested
            }

            // Next offset
            cal_sweep_idx++;
            cal_timer = 0;
            cal_id_sum = 0;
            cal_id_count = 0;

            uint8_t num_offsets = (cal_phase == 0) ? CAL_COARSE_NUM : CAL_FINE_NUM;
            if (cal_sweep_idx >= num_offsets) {
                if (cal_phase == 0) {
                    // Coarse complete → start fine sweep around best
                    cal_phase = 1;
                    cal_sweep_idx = 0;
                    float center = cal_sweep_best_offset;
                    for (int i = 0; i < CAL_FINE_NUM; i++) {
                        cal_fine[i] = center + (float)(i - 10) * 0.005f;
                    }
                    cal_sweep_id_min = 99.0f;  // reset for fine phase
                    hall_angle_offset = cal_fine[0];
                } else {
                    // Fine complete: apply best offset
                    hall_angle_offset = cal_sweep_best_offset;
                    cal_sweep_en = 0;
                    cal_sweep_idx = 0;
                    cal_phase = 0;
                    cal_coarse_init = 0;  // reset for next calibration run
                }
            } else {
                if (cal_phase == 0)
                    hall_angle_offset = cal_coarse[cal_sweep_idx];
                else
                    hall_angle_offset = cal_fine[cal_sweep_idx];
            }
        }
    }

    // Measure ADC callback rate: diff between 1s and 2s = exact 1-second count
    if (timer_callback_count == 1000) {
        adc_count_at_1s = adc_callback_count;
    } else if (timer_callback_count == 2000) {
        adc_rate_result = adc_callback_count - adc_count_at_1s;
    }
    // Read adc_rate_result after 2+ sec: 10000=10kHz, 20000=20kHz

    bsp_io_level_t power_switch_pin;

    // Speed/Torque control mode selection (1kHz rate)
    // Reset controller state on mode change (C0↔C1) to prevent stale integrators
    if (mode_change_pending) {
        mode_change_pending = 0;
        // Speed PI: reset integrator and state machine
        FOCSpeedControl_DW.Integrator_DSTATE = 0.0f;
        FOCSpeedControl_DW.Integrator_IC_LOADING = 1U;
        FOCSpeedControl_DW.is_Run = 0U;   // NO_ACTIVE_CHILD
        FOCSpeedControl_DW.is_c1_FOCSpeedControl = 1U;  // Init
        FOCSpeedControl_DW.UnitDelay_DSTATE = 0.0f;
        // Torque mode: reset soft ramp so it re-triggers on next entry
        torque_startup_ms = 0;
        torque_was_enabled = 0;
    }
    // ff_scale tracks how much IdqRef was reduced vs raw command (design v2 §3.3)
    if (control_mode == 0) {
        // Mode 0: Speed control (existing behavior)
        float iq_raw_before = IdqRef[1];  // capture before SpeedControl modifies it
        SpeedControl_step(Enable, SpeedRefIn_PU, SpeedFb_Hall_PU, IqFb, Mode, IdqRef, &SpeedRefFinal_PU, &EnCl);
        float iq_after_sc = IdqRef[1];    // after speed controller
        debug_speed_integ = FOCSpeedControl_DW.Integrator_DSTATE;
        // Apply thermal derating to speed controller Iq output
        IdqRef[1] *= output_derating;
        // Compute ff_scale for speed mode (derating effect on FF)
        float iq_abs_raw = iq_after_sc > 0 ? iq_after_sc : -iq_after_sc;
        float iq_abs_final = IdqRef[1] > 0 ? IdqRef[1] : -IdqRef[1];
        if (iq_abs_raw > 0.001f)
            ff_scale = iq_abs_final / iq_abs_raw;
        else
            ff_scale = 0.0f;
        if (ff_scale > 1.0f) ff_scale = 1.0f;
    } else {
        // Mode 1: Torque control (direct Iq command, bypass speed PI loop)
        //
        // Startup sequence:
        //   Phase 0 (0~100ms):   Alignment — Id=0.08, Iq=0 → lock rotor to d-axis
        //   Phase 1 (100~200ms): Ramp — Id decays, Iq ramps to target
        //   Phase 2 (200ms~):    Normal torque control
        //
        // Reset soft ramp on disable or mode change
        if (!Enable) {
            torque_startup_ms = 0;
            torque_was_enabled = 0;
        } else if (!torque_was_enabled) {
            torque_startup_ms = 0;
            torque_was_enabled = 1;
        }

        float iq_cmd_raw = torque_ref_iq;
        float iq_cmd = iq_cmd_raw;
        if (iq_cmd > torque_ref_iq_max)  iq_cmd = torque_ref_iq_max;
        if (iq_cmd < -torque_ref_iq_max) iq_cmd = -torque_ref_iq_max;
        iq_cmd_raw = iq_cmd;  // raw = after clamp, before limiter/derating

        // Speed limiter: linearly reduce Iq when speed > 90% of max
        if (torque_max_speed_rpm > 0.0f) {
            float speed_rpm_filtered = SpeedFb_Hall_PU * pmsm.N_base;
            float speed_abs = speed_rpm_filtered > 0 ? speed_rpm_filtered : -speed_rpm_filtered;
            float limit_start = torque_max_speed_rpm * 0.9f;
            if (speed_abs > torque_max_speed_rpm) {
                iq_cmd = 0.0f;  // hard cutoff above max
            } else if (speed_abs > limit_start) {
                float ratio = (torque_max_speed_rpm - speed_abs) / (torque_max_speed_rpm - limit_start);
                iq_cmd *= ratio;
            }
        }

        // Apply thermal derating to torque command
        iq_cmd *= output_derating;

        // Startup: soft ramp over 100ms (no d-axis alignment, Hall provides angle)
        #define TORQUE_SOFTRAMP_MS 100
        if (Enable && torque_startup_ms < TORQUE_SOFTRAMP_MS) {
            torque_startup_ms++;
            float ramp = (float)torque_startup_ms / (float)TORQUE_SOFTRAMP_MS;
            IdqRef[0] = 0.0f;
            IdqRef[1] = iq_cmd * ramp;
            iq_cmd = iq_cmd * ramp;
        } else {
            IdqRef[0] = 0.0f;
            IdqRef[1] = iq_cmd;
        }

        SpeedRefFinal_PU = 0.0f;
        EnCl = Enable;         // Enable current control when motor is enabled

        // ff_scale = |iq_cmd_final| / |iq_cmd_raw| (design v2 §3.3)
        float iq_raw_abs = iq_cmd_raw > 0 ? iq_cmd_raw : -iq_cmd_raw;
        if (iq_raw_abs > 0.001f)
            ff_scale = (iq_cmd > 0 ? iq_cmd : -iq_cmd) / iq_raw_abs;
        else
            ff_scale = 0.0f;
        if (ff_scale > 1.0f) ff_scale = 1.0f;
    }

    // Overcurrent fault: force IdqRef=0 every 1ms (design v2 §3.1)
    if (overcurrent_fault) {
        IdqRef[0] = 0.0f;
        IdqRef[1] = 0.0f;
        ff_scale = 0.0f;
    }

    // Stall detection: RPM≈0 + current ABOVE rated for >3 seconds → fault
    // Rated current 3.29A = 0.20PU → I²=0.04. Threshold set above rated so
    // continuous stall at rated current is allowed (i2t handles thermal safety).
    {
        static uint16_t stall_count = 0;
        float rpm_abs = SpeedFb_RPM > 0 ? SpeedFb_RPM : -SpeedFb_RPM;
        float I2_now = IqFb * IqFb + IdFb * IdFb;
        if (Enable && rpm_abs < 10.0f && I2_now > 0.05f) {
            stall_count++;
            if (stall_count >= 3000) {  // 3 seconds at 1kHz
                overcurrent_fault = 1;
                debug_enable_off_src = 10;  // Stall detection
                stall_count = 0;
            }
        } else {
            stall_count = 0;
        }
    }

    // RPM limit protection: ultimate safety net (design v2 §3.4)
    {
        float rpm_abs = SpeedFb_RPM > 0 ? SpeedFb_RPM : -SpeedFb_RPM;
        if (rpm_abs > 4500.0f) {
            overcurrent_fault = 1;
            Enable = 0;
            debug_enable_off_src = 9;  // RPM limit
        }
    }

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
                Enable = 0; debug_enable_off_src = 2;  // Shutdown
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
    // Makita battery AS terminal monitoring (1kHz rate)
    // ============================================================
    if (makita_battery_en && !shutdown_requested) {
        if (as_voltage < AS_PERMIT_THRESHOLD) {
            as_low_counter++;
            // Debounce: >15ms LOW = discharge prohibited (ignore 5ms periodic check)
            if (as_low_counter >= AS_PROHIBIT_DEBOUNCE_MS) {
                makita_battery_state = 2;  // prohibited
                // Self-shutdown: stop motor and cut power
                shutdown_requested = 1;
                debug_shutdown_requested = 1;
                Enable = 0; debug_enable_off_src = 3;  // Makita AS
                IdqRef[0] = 0.0f;
                IdqRef[1] = 0.0f;
                Vabc_out[0] = 0.5f;
                Vabc_out[1] = 0.5f;
                Vabc_out[2] = 0.5f;
                R_IOPORT_PinWrite(&g_ioport_ctrl, LATCH_OFF, BSP_IO_LEVEL_HIGH);
            }
        } else {
            as_low_counter = 0;
            makita_battery_state = 1;  // permitted
        }
    }

    // ============================================================
    // Protection monitoring (1kHz rate)
    // ============================================================
    {
        uint8_t new_protection = 0;
        float new_derating = 1.0f;

        // Battery low voltage check (with debounce to ignore transient dips)
        // Also treat battery_voltage ≈ 0 as ADC fault → emergency stop
        static uint16_t vbat_low_count = 0;
        if(battery_voltage < VBAT_LOW_CUTOFF) {
            // battery < 12V includes ADC fault (0V) and genuine low battery
            if (battery_voltage > 1.0f || battery_voltage < 0.5f) {
                // > 1V: genuine low battery. < 0.5V: ADC fault (no reading)
                vbat_low_count++;
                if (vbat_low_count >= 500) {  // 500ms continuous low → emergency stop
                    new_protection = 3;
                    new_derating = 0.0f;
                }
            }
        } else {
            vbat_low_count = 0;
        }
        if(battery_voltage < VBAT_LOW_WARNING && battery_voltage > 1.0f) {
            if(new_protection < 1) new_protection = 1;
            if(new_derating > 0.5f) new_derating = 0.5f;
        }

        // I²t motor winding thermal protection (design v2 §3.2)
        // Raw I² at 1kHz + clamp 0.12 PU² (Hall spike mitigation)
        // First-order model: dE/dt = I²_clamped - E/τ
        {
            float iq = IqFb;
            float id = IdFb;
            float I2 = iq * iq + id * id;  // |I|² in PU²
            const float dt = 0.001f;        // 1ms

            // Debug: track peak and average I² for diagnosis
            if (I2 > debug_i2_max) debug_i2_max = I2;
            debug_i2_avg += 0.001f * (I2 - debug_i2_avg);

            // Clamp I² at 0.12 PU² to handle Hall transition spikes
            // Raw 0.57 PU² (spike) → clamped 0.12 → effective ~93% steady state → trip ~60s
            // Without clamp: trips in 25s (too aggressive)
            // With 100Hz decimation: never trips (too permissive, caused 3/23 incident)
            float I2_clamped = I2 > 0.12f ? 0.12f : I2;

            // Pause accumulation during calibration (but don't zero — motor is still warm)
            if (!cal_sweep_en) {
                i2t_accumulator += (I2_clamped - i2t_accumulator / I2T_TAU_W) * dt;
                if (i2t_accumulator < 0.0f) i2t_accumulator = 0.0f;
            }

            i2t_ratio = i2t_accumulator / I2T_THRESHOLD;

            if (i2t_ratio >= I2T_TRIP_PCT) {
                new_protection = 3;
                new_derating = 0.0f;
                overcurrent_fault = 1;
                overcurrent_peak_iq = iq > 0 ? iq : -iq;
                overcurrent_peak_id = id > 0 ? id : -id;
                debug_enable_off_src = 8;  // I²t thermal trip
            } else if (i2t_ratio >= I2T_DERATING_PCT) {
                if (new_protection < 2) new_protection = 2;
                float derate = 0.5f * (1.0f - (i2t_ratio - I2T_DERATING_PCT) / (I2T_TRIP_PCT - I2T_DERATING_PCT));
                if (new_derating > derate) new_derating = derate;
            } else if (i2t_ratio >= I2T_WARNING_PCT) {
                if (new_protection < 1) new_protection = 1;
                float derate = 1.0f - 0.5f * (i2t_ratio - I2T_WARNING_PCT) / (I2T_DERATING_PCT - I2T_WARNING_PCT);
                if (new_derating > derate) new_derating = derate;
            }
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
            Enable = 0; debug_enable_off_src = 4;  // Protection
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
    // LED output control (Active Low: MCU Low → LED ON)
    // led_mode=0: auto (base driver default)
    // led_mode=1: manual (led_state controlled by BLE/app)
    // ============================================================
    {
        uint8_t ls;
        if (led_mode == 0) {
            // Auto mode: base driver default pattern
            ls = 0;
            if (drv8302_fault_latched || protection_state >= 3) {
                ls |= 0x02;  // PwrRed ON
            } else if (protection_state >= 1) {
                ls |= 0x01;  // PwrGrn ON
                if ((timer_callback_count / 250) % 2) ls |= 0x02;  // PwrRed blink 2Hz
            } else {
                ls |= 0x01;  // PwrGrn ON
            }
            if (Enable) ls |= 0x04;  // Turbo LED = motor enabled indicator
            led_state = ls;
        } else {
            ls = led_state;  // Manual: use BLE/app-set value directly
        }

        // Write all 6 LEDs from led_state bits (1=ON → LOW, 0=OFF → HIGH)
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_POWER_GREEN, (ls & 0x01) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_POWER_RED,   (ls & 0x02) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_TURBO,       (ls & 0x04) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_H,    (ls & 0x08) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_M,    (ls & 0x10) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ASSIST_L,    (ls & 0x20) ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
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
            // Buffer line into rx_cmd_ring (CR or LF = end of line)
            if (ch == '\r' || ch == '\n') {
                if (uart_rx_idx > 0 && rx_cmd_count < RX_CMD_SLOTS) {
                    uint16_t copy_len = uart_rx_idx < RX_CMD_SIZE - 1 ? uart_rx_idx : RX_CMD_SIZE - 1;
                    memcpy(rx_cmd_ring[rx_cmd_head], uart_rx_buf, copy_len);
                    rx_cmd_ring[rx_cmd_head][copy_len] = '\0';
                    rx_cmd_head = (rx_cmd_head + 1) % RX_CMD_SLOTS;
                    rx_cmd_count++;
                }
                uart_rx_idx = 0;
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
