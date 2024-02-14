/* ST Microelectronics LSM6DSV16X 6-axis IMU sensor driver
 *
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf
 */

#define DT_DRV_COMPAT st_lsm6dsv16x

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "lsm6dsv16x.h"

LOG_MODULE_DECLARE(LSM6DSV16X, CONFIG_SENSOR_LOG_LEVEL);

/**
 * lsm6dsv16x_enable_xl_int - XL enable selected int pin to generate interrupt
 */
static int lsm6dsv16x_enable_xl_int(const struct device *dev, int enable)
{
	const struct lsm6dsv16x_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	if (enable) {
		int16_t buf[3];

		/* dummy read: re-trigger interrupt */
		lsm6dsv16x_acceleration_raw_get(ctx, buf);
	}

	/* set interrupt */
	if (cfg->drdy_pin == 1) {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int1_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int1_route_get error");
			return ret;
		}

		val.drdy_xl = 1;

		ret = lsm6dsv16x_pin_int1_route_set(ctx, &val);
	} else {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int2_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int2_route_get error");
			return ret;
		}

		val.drdy_xl = 1;

		ret = lsm6dsv16x_pin_int2_route_set(ctx, &val);
	}

	return ret;
}

/**
 * lsm6dsv16x_enable_g_int - Gyro enable selected int pin to generate interrupt
 */
static int lsm6dsv16x_enable_g_int(const struct device *dev, int enable)
{
	const struct lsm6dsv16x_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	if (enable) {
		int16_t buf[3];

		/* dummy read: re-trigger interrupt */
		lsm6dsv16x_angular_rate_raw_get(ctx, buf);
	}

	/* set interrupt */
	if (cfg->drdy_pin == 1) {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int1_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int1_route_get error");
			return ret;
		}

		val.drdy_g = 1;

		ret = lsm6dsv16x_pin_int1_route_set(ctx, &val);
	} else {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int2_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int2_route_get error");
			return ret;
		}

		val.drdy_g = 1;

		ret = lsm6dsv16x_pin_int2_route_set(ctx, &val);
	}

	return ret;
}

/**
 * lsm6dsv16x_enable_act_int - Activity enable selected int pin to generate interrupt
 */
static int lsm6dsv16x_enable_act_int(const struct device *dev)
{
	const struct lsm6dsv16x_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	/* set interrupt */
	if (cfg->drdy_pin != 1) {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int1_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pin_int1_route_get error");
			return ret;
		}

		val.sleep_change = 1;

		LOG_DBG("New pin 1 route 0x%04X", *(uint32_t *)&val);

		ret = lsm6dsv16x_pin_int1_route_set(ctx, &val);
	} else {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int2_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pin_int2_route_get error");
			return ret;
		}

		val.sleep_change = 1;

		LOG_DBG("New pin 2 route 0x%04X", *(uint32_t *)&val);

		ret = lsm6dsv16x_pin_int2_route_set(ctx, &val);
	}

	return ret;
}

/**
 * lsm6dsv16x_enable_fsm_int - FSM enable selected int pin to generate interrupt
 */
static int lsm6dsv16x_enable_fsm_int(const struct device *dev, uint8_t index)
{
	const struct lsm6dsv16x_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	/* set interrupt */
	if (cfg->drdy_pin != 1) {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int1_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pin_int1_route_get error");
			return ret;
		}

		val.emb_func = 1;

		LOG_DBG("New pin 1 route 0x%04X", *(uint32_t *)&val);

		ret = lsm6dsv16x_pin_int1_route_set(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pin_int1_route_set error");
			return ret;
		}

		ret = lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_EMBED_FUNC_MEM_BANK);
		if (ret) {
			LOG_ERR("Failed to open emb func registers");
			return ret;
		}

		lsm6dsv16x_fsm_int1_t int_val;
		ret = lsm6dsv16x_read_reg(ctx, LSM6DSV16X_FSM_INT1, (uint8_t *)&int_val, 1);
		if (ret) {
			LOG_ERR("Failed to read FSM int1");
			lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_MAIN_MEM_BANK);
			return ret;
		}

		switch (index) {
		case 0:
			int_val.int1_fsm1 = 1;
			break;
		case 1:
			int_val.int1_fsm2 = 1;
			break;
		case 2:
			int_val.int1_fsm3 = 1;
			break;
		case 3:
			int_val.int1_fsm4 = 1;
			break;
		case 4:
			int_val.int1_fsm5 = 1;
			break;
		case 5:
			int_val.int1_fsm6 = 1;
			break;
		case 6:
			int_val.int1_fsm7 = 1;
			break;
		case 7:
			int_val.int1_fsm8 = 1;
			break;
		default:
			return -EINVAL;
		}

		ret = lsm6dsv16x_write_reg(ctx, LSM6DSV16X_FSM_INT1, (uint8_t *)&int_val, 1);
		if (ret) {
			LOG_ERR("Failed to write FSM int1");
			lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_MAIN_MEM_BANK);
			return ret;
		}

		ret = lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_MAIN_MEM_BANK);
	} else {
		lsm6dsv16x_pin_int_route_t val;

		ret = lsm6dsv16x_pin_int2_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pin_int2_route_get error");
			return ret;
		}

		val.emb_func = 1;

		LOG_DBG("New pin 2 route 0x%04X", *(uint32_t *)&val);

		ret = lsm6dsv16x_pin_int2_route_set(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pin_int2_route_set error");
			return ret;
		}

		ret = lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_EMBED_FUNC_MEM_BANK);
		if (ret) {
			LOG_ERR("Failed to open emb func registers");
			return ret;
		}

		lsm6dsv16x_fsm_int2_t int_val;
		ret = lsm6dsv16x_read_reg(ctx, LSM6DSV16X_FSM_INT2, (uint8_t *)&int_val, 1);
		if (ret) {
			LOG_ERR("Failed to read FSM int2");
			lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_MAIN_MEM_BANK);
			return ret;
		}

		switch (index) {
		case 0:
			int_val.int2_fsm1 = 1;
			break;
		case 1:
			int_val.int2_fsm2 = 1;
			break;
		case 2:
			int_val.int2_fsm3 = 1;
			break;
		case 3:
			int_val.int2_fsm4 = 1;
			break;
		case 4:
			int_val.int2_fsm5 = 1;
			break;
		case 5:
			int_val.int2_fsm6 = 1;
			break;
		case 6:
			int_val.int2_fsm7 = 1;
			break;
		case 7:
			int_val.int2_fsm8 = 1;
			break;
		default:
			return -EINVAL;
		}

		ret = lsm6dsv16x_write_reg(ctx, LSM6DSV16X_FSM_INT2, (uint8_t *)&int_val, 1);
		if (ret) {
			LOG_ERR("Failed to write FSM int2");
			lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_MAIN_MEM_BANK);
			return ret;
		}

		ret = lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_MAIN_MEM_BANK);
	}

	return ret;
}

/**
 * lsm6dsv16x_trigger_set - link external trigger to event data ready
 */
int lsm6dsv16x_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			   sensor_trigger_handler_t handler)
{
	const struct lsm6dsv16x_config *cfg = dev->config;
	struct lsm6dsv16x_data *lsm6dsv16x = dev->data;

	if (!cfg->trig_enabled) {
		LOG_ERR("trigger_set op not supported");
		return -ENOTSUP;
	}

	if (trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
		if (trig->type == SENSOR_TRIG_DATA_READY) {
			lsm6dsv16x->handler_drdy_acc = handler;
			lsm6dsv16x->trig_drdy_acc = trig;
			if (handler) {
				return lsm6dsv16x_enable_xl_int(dev, LSM6DSV16X_EN_BIT);
			} else {
				return lsm6dsv16x_enable_xl_int(dev, LSM6DSV16X_DIS_BIT);
			}
		} else if (trig->type == SENSOR_TRIG_MOTION) {
			lsm6dsv16x->handler_motion_acc = handler;
			lsm6dsv16x->trig_motion_acc = trig;
			return lsm6dsv16x_enable_act_int(dev);
		} else if (trig->type == SENSOR_TRIG_STATIONARY) {
			lsm6dsv16x->handler_stationary_acc = handler;
			lsm6dsv16x->trig_stationary_acc = trig;
			return lsm6dsv16x_enable_act_int(dev);
		} else if (trig->type >= (enum sensor_trigger_type)SENSOR_TRIG_LSM6DSV16X_FSM1 &&
			   trig->type <= (enum sensor_trigger_type)SENSOR_TRIG_LSM6DSV16X_FSM8) {
			uint8_t fsm_index = trig->type - SENSOR_TRIG_LSM6DSV16X_FSM1;
			lsm6dsv16x->handler_fsm[fsm_index] = handler;
			lsm6dsv16x->trig_fsm[fsm_index] = trig;
			return lsm6dsv16x_enable_fsm_int(dev, fsm_index);
		}
	} else if (trig->chan == SENSOR_CHAN_GYRO_XYZ) {
		lsm6dsv16x->handler_drdy_gyr = handler;
		lsm6dsv16x->trig_drdy_gyr = trig;
		if (handler) {
			return lsm6dsv16x_enable_g_int(dev, LSM6DSV16X_EN_BIT);
		} else {
			return lsm6dsv16x_enable_g_int(dev, LSM6DSV16X_DIS_BIT);
		}
	}

	return -ENOTSUP;
}

/**
 * lsm6dsv16x_handle_interrupt - handle the drdy event
 * read data and call handler if registered any
 */
static void lsm6dsv16x_handle_interrupt(const struct device *dev)
{
	struct lsm6dsv16x_data *lsm6dsv16x = dev->data;
	const struct lsm6dsv16x_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	lsm6dsv16x_pin_int_route_t val1, val2;
	if (lsm6dsv16x_pin_int1_route_get(ctx, &val1) < 0) {
		LOG_ERR("pin_int1_route_get error");
		return;
	}
	if (lsm6dsv16x_pin_int2_route_get(ctx, &val2) < 0) {
		LOG_ERR("pin_int2_route_get error");
		return;
	}

	bool check_drdy_xl = (val1.drdy_xl != 0 || val2.drdy_xl != 0);
	bool check_drdy_g = (val1.drdy_g != 0 || val2.drdy_g != 0);
	bool check_sleep = (val1.sleep_change != 0 || val2.sleep_change != 0);
	bool check_emb_func = (val1.emb_func != 0 || val2.emb_func != 0);

	if (check_drdy_xl || check_drdy_g) {
		lsm6dsv16x_data_ready_t data_ready;
		if (lsm6dsv16x_flag_data_ready_get(ctx, &data_ready) < 0) {
			LOG_ERR("Failed reading data ready flag");
			return;
		}

		if ((data_ready.drdy_xl) && (lsm6dsv16x->handler_drdy_acc != NULL)) {
			lsm6dsv16x->handler_drdy_acc(dev, lsm6dsv16x->trig_drdy_acc);
		}

		if ((data_ready.drdy_gy) && (lsm6dsv16x->handler_drdy_gyr != NULL)) {
			lsm6dsv16x->handler_drdy_gyr(dev, lsm6dsv16x->trig_drdy_gyr);
		}
	}

	if (check_sleep) {
		lsm6dsv16x_wake_up_src_t wake_up_src;
		if (lsm6dsv16x_read_reg(ctx, LSM6DSV16X_WAKE_UP_SRC, (uint8_t *)&wake_up_src, 1) <
		    0) {
			LOG_ERR("Failed reading wake up src");
			return;
		}

		if (wake_up_src.sleep_change_ia != 0) {
			if (wake_up_src.sleep_state == 0) {
				if ((lsm6dsv16x->handler_motion_acc != NULL)) {
					lsm6dsv16x->handler_motion_acc(dev,
								       lsm6dsv16x->trig_motion_acc);
				}
			} else {
				if ((lsm6dsv16x->handler_stationary_acc != NULL)) {
					lsm6dsv16x->handler_stationary_acc(
						dev, lsm6dsv16x->trig_stationary_acc);
				}
			}
		}
	}

	if (check_emb_func) {
		lsm6dsv16x_fsm_status_mainpage_t fsm_status;
		if (lsm6dsv16x_read_reg(ctx, LSM6DSV16X_FSM_STATUS_MAINPAGE, (uint8_t *)&fsm_status,
					1) < 0) {
			LOG_ERR("Failed reading fsm status");
			return;
		}

		if (fsm_status.is_fsm1 && lsm6dsv16x->handler_fsm[0] != NULL) {
			lsm6dsv16x->handler_fsm[0](dev, lsm6dsv16x->trig_fsm[0]);
		}

		if (fsm_status.is_fsm2 && lsm6dsv16x->handler_fsm[1] != NULL) {
			lsm6dsv16x->handler_fsm[1](dev, lsm6dsv16x->trig_fsm[1]);
		}

		if (fsm_status.is_fsm3 && lsm6dsv16x->handler_fsm[2] != NULL) {
			lsm6dsv16x->handler_fsm[2](dev, lsm6dsv16x->trig_fsm[2]);
		}

		if (fsm_status.is_fsm4 && lsm6dsv16x->handler_fsm[3] != NULL) {
			lsm6dsv16x->handler_fsm[3](dev, lsm6dsv16x->trig_fsm[3]);
		}

		if (fsm_status.is_fsm5 && lsm6dsv16x->handler_fsm[4] != NULL) {
			lsm6dsv16x->handler_fsm[4](dev, lsm6dsv16x->trig_fsm[4]);
		}

		if (fsm_status.is_fsm6 && lsm6dsv16x->handler_fsm[5] != NULL) {
			lsm6dsv16x->handler_fsm[5](dev, lsm6dsv16x->trig_fsm[5]);
		}

		if (fsm_status.is_fsm7 && lsm6dsv16x->handler_fsm[6] != NULL) {
			lsm6dsv16x->handler_fsm[6](dev, lsm6dsv16x->trig_fsm[6]);
		}

		if (fsm_status.is_fsm8 && lsm6dsv16x->handler_fsm[7] != NULL) {
			lsm6dsv16x->handler_fsm[7](dev, lsm6dsv16x->trig_fsm[7]);
		}
	}

	gpio_pin_interrupt_configure_dt(lsm6dsv16x->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(lsm6dsv16x->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static void lsm6dsv16x_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				     uint32_t pins)
{
	struct lsm6dsv16x_data *lsm6dsv16x = CONTAINER_OF(cb, struct lsm6dsv16x_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(lsm6dsv16x->drdy_gpio, GPIO_INT_DISABLE);

#if defined(CONFIG_LSM6DSV16X_TRIGGER_OWN_THREAD)
	k_sem_give(&lsm6dsv16x->int_sem);
#elif defined(CONFIG_LSM6DSV16X_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&lsm6dsv16x->work);
#endif /* CONFIG_LSM6DSV16X_TRIGGER_OWN_THREAD */
}

static void lsm6dsv16x_gpio_motion_callback(const struct device *dev, struct gpio_callback *cb,
					    uint32_t pins)
{
	struct lsm6dsv16x_data *lsm6dsv16x =
		CONTAINER_OF(cb, struct lsm6dsv16x_data, gpio_motion_cb);

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(lsm6dsv16x->motion_gpio, GPIO_INT_DISABLE);

#if defined(CONFIG_LSM6DSV16X_TRIGGER_OWN_THREAD)
	k_sem_give(&lsm6dsv16x->int_sem);
#elif defined(CONFIG_LSM6DSV16X_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&lsm6dsv16x->work);
#endif /* CONFIG_LSM6DSV16X_TRIGGER_OWN_THREAD */
}

#ifdef CONFIG_LSM6DSV16X_TRIGGER_OWN_THREAD
static void lsm6dsv16x_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct lsm6dsv16x_data *lsm6dsv16x = p1;

	while (1) {
		k_sem_take(&lsm6dsv16x->int_sem, K_FOREVER);
		lsm6dsv16x_handle_interrupt(lsm6dsv16x->dev);
	}
}
#endif /* CONFIG_LSM6DSV16X_TRIGGER_OWN_THREAD */

#ifdef CONFIG_LSM6DSV16X_TRIGGER_GLOBAL_THREAD
static void lsm6dsv16x_work_cb(struct k_work *work)
{
	struct lsm6dsv16x_data *lsm6dsv16x = CONTAINER_OF(work, struct lsm6dsv16x_data, work);

	lsm6dsv16x_handle_interrupt(lsm6dsv16x->dev);
}
#endif /* CONFIG_LSM6DSV16X_TRIGGER_GLOBAL_THREAD */

int lsm6dsv16x_init_interrupt(const struct device *dev)
{
	struct lsm6dsv16x_data *lsm6dsv16x = dev->data;
	const struct lsm6dsv16x_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	lsm6dsv16x->drdy_gpio = (cfg->drdy_pin == 1) ? (struct gpio_dt_spec *)&cfg->int1_gpio
						     : (struct gpio_dt_spec *)&cfg->int2_gpio;
	lsm6dsv16x->motion_gpio = (cfg->drdy_pin == 2) ? (struct gpio_dt_spec *)&cfg->int1_gpio
						       : (struct gpio_dt_spec *)&cfg->int2_gpio;

	/* setup data ready gpio interrupt (INT1 or INT2) */
	if (!gpio_is_ready_dt(lsm6dsv16x->drdy_gpio)) {
		LOG_ERR("Cannot get pointer to drdy_gpio device");
		return -EINVAL;
	}

	if (!gpio_is_ready_dt(lsm6dsv16x->motion_gpio)) {
		LOG_ERR("Cannot get pointer to motion_gpio device");
		return -EINVAL;
	}

#if defined(CONFIG_LSM6DSV16X_TRIGGER_OWN_THREAD)
	k_sem_init(&lsm6dsv16x->int_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&lsm6dsv16x->thread, lsm6dsv16x->thread_stack,
			CONFIG_LSM6DSV16X_THREAD_STACK_SIZE, lsm6dsv16x_thread, lsm6dsv16x, NULL,
			NULL, K_PRIO_COOP(CONFIG_LSM6DSV16X_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&lsm6dsv16x->thread, "lsm6dsv16x");
#elif defined(CONFIG_LSM6DSV16X_TRIGGER_GLOBAL_THREAD)
	lsm6dsv16x->work.handler = lsm6dsv16x_work_cb;
#endif /* CONFIG_LSM6DSV16X_TRIGGER_OWN_THREAD */

	ret = gpio_pin_configure_dt(lsm6dsv16x->drdy_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_DBG("Could not configure drdy_gpio");
		return ret;
	}

	ret = gpio_pin_configure_dt(lsm6dsv16x->motion_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_DBG("Could not configure motion_gpio");
		return ret;
	}

	gpio_init_callback(&lsm6dsv16x->gpio_cb, lsm6dsv16x_gpio_callback,
			   BIT(lsm6dsv16x->drdy_gpio->pin));

	gpio_init_callback(&lsm6dsv16x->gpio_motion_cb, lsm6dsv16x_gpio_motion_callback,
			   BIT(lsm6dsv16x->motion_gpio->pin));

	if (gpio_add_callback(lsm6dsv16x->drdy_gpio->port, &lsm6dsv16x->gpio_cb) < 0) {
		LOG_DBG("Could not set gpio callback");
		return -EIO;
	}

	if (gpio_add_callback(lsm6dsv16x->motion_gpio->port, &lsm6dsv16x->gpio_motion_cb) < 0) {
		LOG_DBG("Could not set motion callback");
		return -EIO;
	}

	/* set data ready mode on int1/int2 */
	LOG_DBG("drdy_pulsed is %d", (int)cfg->drdy_pulsed);
	lsm6dsv16x_data_ready_mode_t drdy_mode =
		cfg->drdy_pulsed ? LSM6DSV16X_DRDY_PULSED : LSM6DSV16X_DRDY_LATCHED;

	ret = lsm6dsv16x_data_ready_mode_set(ctx, drdy_mode);
	if (ret < 0) {
		LOG_ERR("drdy_pulsed config error %d", (int)cfg->drdy_pulsed);
		return ret;
	}

	LOG_DBG("emb_pulsed is %d", (int)cfg->emb_pulsed);
	ret = lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_EMBED_FUNC_MEM_BANK);
	if (ret) {
		LOG_ERR("Failed to change mem bank");
		return ret;
	}
	lsm6dsv16x_page_rw_t page_rw;
	ret = lsm6dsv16x_read_reg(ctx, LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);
	if (ret < 0) {
		LOG_ERR("emb_pulsed config get error");
		return ret;
	}
	page_rw.emb_func_lir = cfg->emb_pulsed ? 0 : 1;
	ret = lsm6dsv16x_write_reg(ctx, LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);
	if (ret < 0) {
		LOG_ERR("emb_pulsed config set error");
		return ret;
	}
	lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_MAIN_MEM_BANK);
	if (ret) {
		LOG_ERR("Failed to change mem bank");
		return ret;
	}

	lsm6dsv16x_interrupt_mode_t interrupt_mode;
	ret = lsm6dsv16x_interrupt_enable_get(ctx, &interrupt_mode);
	if (ret) {
		LOG_ERR("Failed to get interrupt mode");
		return ret;
	}

	interrupt_mode.enable = 1;
	interrupt_mode.lir = cfg->emb_pulsed ? 0 : 1;

	ret = lsm6dsv16x_interrupt_enable_set(ctx, interrupt_mode);
	if (ret) {
		LOG_ERR("Failed to enable interrupts");
		return ret;
	}

	lsm6dsv16x_act_mode_t act_mode = LSM6DSV16X_XL_AND_GY_NOT_AFFECTED;

	ret = lsm6dsv16x_act_mode_set(ctx, act_mode);
	if (ret < 0) {
		LOG_ERR("act mode config error %d", (int)act_mode);
		return ret;
	}

	ret = lsm6dsv16x_act_from_sleep_to_act_dur_set(ctx, LSM6DSV16X_SLEEP_TO_ACT_AT_3RD_SAMPLE);
	if (ret < 0) {
		LOG_ERR("act sleep duration config error");
		return ret;
	}

	ret = lsm6dsv16x_act_sleep_xl_odr_set(ctx, LSM6DSV16X_60Hz);
	if (ret) {
		LOG_ERR("act xl odr config error");
		return ret;
	}

	lsm6dsv16x_act_thresholds_t threshold;
	ret = lsm6dsv16x_act_thresholds_get(ctx, &threshold);
	if (ret) {
		LOG_ERR("get act thresholds error");
		return ret;
	}

	// threshold.duration = 3;
	threshold.inactivity_ths = 1;
	threshold.inactivity_cfg.wu_inact_ths_w = 3;

	ret = lsm6dsv16x_act_thresholds_set(ctx, &threshold);
	if (ret) {
		LOG_ERR("set act thresholds error");
		return ret;
	}

	lsm6dsv16x_act_wkup_time_windows_t durations;
	ret = lsm6dsv16x_act_wkup_time_windows_get(ctx, &durations);
	if (ret) {
		LOG_ERR("Failed to get durations");
		return ret;
	}

	durations.quiet = 1;

	ret = lsm6dsv16x_act_wkup_time_windows_set(ctx, durations);
	if (ret) {
		LOG_ERR("Failed to set durations");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(lsm6dsv16x->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure interrupt for drdy");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(lsm6dsv16x->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure interrupt for motion");
		return ret;
	}

	return ret;
}
