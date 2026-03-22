#ifndef ASSIST_APP_INTERFACE_H_
#define ASSIST_APP_INTERFACE_H_

#include "rtwtypes.h"

/* Input flags (buttons) */
typedef struct {
    boolean_T btn_normal_assist;  /* Assist switch (DI_IN1) */
    boolean_T btn_force;          /* Turbo switch (DI_IN2) */
    boolean_T btn_power;          /* Power switch (DI_PON) */
} InFlags_T;

/* Simulink model inputs (RA6T2 -> Model) */
typedef struct {
    boolean_T SVON;          /* Servo ON state */
    uint8_T   err_type;      /* Driver error type (protection_state) */
    int32_T   hall_enccnt;   /* Hall sensor cumulative position count */
    int16_T   Mot_spd;       /* Motor speed [RPM] */
    int32_T   batterycnt;    /* Battery voltage ADC raw count */
    int16_T   Acc_x;         /* Accelerometer X [raw LSB] */
    int16_T   Acc_y;         /* Accelerometer Y [raw LSB] */
    int16_T   Acc_z;         /* Accelerometer Z [raw LSB] */
    InFlags_T in_flags;      /* Button inputs */
} AssistApp_ExtU_T;

/* Simulink model outputs (Model -> RA6T2) — placeholder for Embedded Coder */
typedef struct {
    int16_T   torque_cmd;    /* Torque command [TBD unit] */
    uint8_T   led_pattern;   /* LED display pattern */
    boolean_T buzzer;        /* Buzzer on/off */
} AssistApp_ExtY_T;

/* Global instances */
extern AssistApp_ExtU_T assist_U;
extern AssistApp_ExtY_T assist_Y;

/* Placeholder — replace with Embedded Coder generated function */
void assist_app_step(void);

#endif /* ASSIST_APP_INTERFACE_H_ */
