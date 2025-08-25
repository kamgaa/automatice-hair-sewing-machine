#ifndef STEPMOTOR_H
#define STEPMOTOR_H

#include "stm32f4xx_hal.h"

// StepMotor 구조체 정의
typedef struct StepMotor {
    GPIO_TypeDef* dir_GPIOx;
    uint16_t dir_GPIO_Pin;
    GPIO_TypeDef* pulse_GPIOx;
    uint16_t pulse_GPIO_Pin;

    uint8_t dir;
    uint32_t phase;
    int32_t current_angle_step;
    double current_angle_deg;
    int32_t desired_angle_step;
    double desired_angle_deg;
    double deg_step_ratio;
    uint32_t step_speed;


    void (*set_desired_angle)(struct StepMotor* motor, double angle);
    void (*step_while)(struct StepMotor* motor);

} StepMotor;

// 함수 선언
void set_desired_angle_func(StepMotor* motor, double angle);
void step_while_func(StepMotor* motor);
void initStepMotor(StepMotor* motor,
                   GPIO_TypeDef* dir_GPIOx,
                   uint16_t dir_GPIO_Pin,
                   GPIO_TypeDef* pulse_GPIOx,
                   uint16_t pulse_GPIO_Pin);

#endif // STEPMOTOR_H
