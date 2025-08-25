#include <stdlib.h>
#include <math.h>
#include "stepmotor.h"

// 함수 정의
void set_desired_angle_func(StepMotor* motor, double angle) {
    motor->desired_angle_deg = angle;
    motor->desired_angle_step = (int32_t)(angle / motor->deg_step_ratio*2);
}

void step_while_func(StepMotor* motor){
       	motor->current_angle_deg = motor->current_angle_step * motor->deg_step_ratio;
	if (motor->current_angle_step != motor->desired_angle_step){
		if (motor->current_angle_step < motor->desired_angle_step) {
			HAL_GPIO_WritePin(motor->dir_GPIOx, motor->dir_GPIO_Pin, GPIO_PIN_SET); // 시계 방향
			motor->dir = 0;
		} else {
			HAL_GPIO_WritePin(motor->dir_GPIOx, motor->dir_GPIO_Pin, GPIO_PIN_RESET); // 반시계 방향
			motor->dir = 1;
		}
//		    HAL_GPIO_TogglePin(motor->pulse_GPIOx, motor->pulse_GPIO_Pin); // 펄스 발생
		if (motor->phase <= motor->step_speed){//smaller phase makes motor faster but not too small
//			HAL_GPIO_WritePin(motor->pulse_GPIOx, motor->pulse_GPIO_Pin, GPIO_PIN_SET);
			motor->phase = motor->phase + 1;
		} else{
			HAL_GPIO_TogglePin(motor->pulse_GPIOx, motor->pulse_GPIO_Pin);
//			HAL_GPIO_WritePin(motor->pulse_GPIOx, motor->pulse_GPIO_Pin, GPIO_PIN_RESET);
			if (motor->dir == 0){
				motor->current_angle_step++;
			} else {
				motor->current_angle_step--;
			}
			motor->phase = 0;
		}
	}
}

void initStepMotor(StepMotor* motor,
                   GPIO_TypeDef* dir_GPIOx,
                   uint16_t dir_GPIO_Pin,
                   GPIO_TypeDef* pulse_GPIOx,
                   uint16_t pulse_GPIO_Pin) {

    motor->dir_GPIOx = dir_GPIOx;
    motor->dir_GPIO_Pin = dir_GPIO_Pin;
    motor->pulse_GPIOx = pulse_GPIOx;
    motor->pulse_GPIO_Pin = pulse_GPIO_Pin;

    motor->dir = 0;
    motor->current_angle_step = 0;
    motor->current_angle_deg = 0.0;
    motor->desired_angle_step = 0;
    motor->desired_angle_deg = 0.0;
    motor->deg_step_ratio = 360.0/1000.0;
    motor->step_speed = 50;


    motor->set_desired_angle = set_desired_angle_func;
    motor->step_while = step_while_func;
}
