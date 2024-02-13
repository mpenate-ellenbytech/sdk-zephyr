/*
 * Copyright (c) 2024 Marcus Penate (Ellenby Technologies, Inc.)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_LSM6DSV16X_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_LSM6DSV16X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

enum lsm6dsv16x_sensor_attribute {
	SENSOR_ATTR_LSM6DSV16X_FSM_ODR = SENSOR_ATTR_PRIV_START,
	SENSOR_ATTR_LSM6DSV16X_FSM_LC,
	SENSOR_ATTR_LSM6DSV16X_FSM_PRG,
	SENSOR_ATTR_LSM6DSV16X_FSM_EN,
};

#ifdef CONFIG_LSM6DSV16X_TRIGGER
enum lsm6dsv16x_sensor_trigger_type {
	SENSOR_TRIG_LSM6DSV16X_FSM1 = SENSOR_TRIG_PRIV_START,
	SENSOR_TRIG_LSM6DSV16X_FSM2,
	SENSOR_TRIG_LSM6DSV16X_FSM3,
	SENSOR_TRIG_LSM6DSV16X_FSM4,
	SENSOR_TRIG_LSM6DSV16X_FSM5,
	SENSOR_TRIG_LSM6DSV16X_FSM6,
	SENSOR_TRIG_LSM6DSV16X_FSM7,
	SENSOR_TRIG_LSM6DSV16X_FSM8,
};
#endif /* CONFIG_LSM6DSV16X_TRIGGER */

typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
	uint8_t nr_timer: 2;
	uint8_t nr_ltimer: 2;
	uint8_t nr_mask: 2;
	uint8_t nr_thresh: 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
	uint8_t nr_thresh: 2;
	uint8_t nr_mask: 2;
	uint8_t nr_ltimer: 2;
	uint8_t nr_timer: 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv16x_fsm_config_a_t;

typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
	uint8_t jmp: 1;
	uint8_t not_used0: 1;
	uint8_t stop_done: 1;
	uint8_t dectree: 1;
	uint8_t pas: 1;
	uint8_t angle: 1;
	uint8_t ext_sinmux: 1;
	uint8_t des: 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
	uint8_t des: 1;
	uint8_t ext_sinmux: 1;
	uint8_t angle: 1;
	uint8_t pas: 1;
	uint8_t dectree: 1;
	uint8_t stop_done: 1;
	uint8_t not_used0: 1;
	uint8_t jmp: 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv16x_fsm_config_b_t;

typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
	uint8_t in_sel: 3;
	uint8_t thrs3sel: 1;
	uint8_t r_tam: 1;
	uint8_t signed_: 1;
	uint8_t marksel: 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
	uint8_t marksel: 2;
	uint8_t signed_: 1;
	uint8_t r_tam: 1;
	uint8_t thrs3sel: 1;
	uint8_t in_sel: 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv16x_fsm_settings_t;

typedef struct {
	uint8_t program;

	lsm6dsv16x_fsm_config_a_t config_a;
	lsm6dsv16x_fsm_config_b_t config_b;
	uint8_t size;
	lsm6dsv16x_fsm_settings_t settings;
	uint8_t reset_pointer;
	uint8_t program_pointer;

	const uint8_t *variable_data;
	uint8_t variable_data_len;

	const uint8_t *program_data;
	uint8_t program_data_len;
} lsm6dsv16x_fsm_t;

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_LSM6DSV16X_H_ */
