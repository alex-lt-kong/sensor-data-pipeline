#include "../module.h"
#include "../utils.h"

// The UPS HAT interaction code is translated from its Python version from:
// https://files.waveshare.com/wiki/UPS-HAT-D/UPS_HAT_D.7z
// (https://www.waveshare.com/wiki/UPS_HAT_(D))
// with the assistance of some helpful LLM models 🙇🙇🙇

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define _REG_CONFIG 0x00
#define _REG_SHUNTVOLTAGE 0x01
#define _REG_BUSVOLTAGE 0x02
#define _REG_POWER 0x03
#define _REG_CURRENT 0x04
#define _REG_CALIBRATION 0x05

#define RANGE_16V 0x00
#define RANGE_32V 0x01

#define DIV_1_40MV 0x00
#define DIV_2_80MV 0x01
#define DIV_4_160MV 0x02
#define DIV_8_320MV 0x03

#define ADCRES_9BIT_1S 0x00
#define ADCRES_10BIT_1S 0x01
#define ADCRES_11BIT_1S 0x02
#define ADCRES_12BIT_1S 0x03
#define ADCRES_12BIT_2S 0x09
#define ADCRES_12BIT_4S 0x0A
#define ADCRES_12BIT_8S 0x0B
#define ADCRES_12BIT_16S 0x0C
#define ADCRES_12BIT_32S 0x0D
#define ADCRES_12BIT_64S 0x0E
#define ADCRES_12BIT_128S 0x0F

#define POWERDOW 0x00
#define SVOLT_TRIGGERED 0x01
#define BVOLT_TRIGGERED 0x02
#define SANDBVOLT_TRIGGERED 0x03
#define ADCOFF 0x04
#define SVOLT_CONTINUOUS 0x05
#define BVOLT_CONTINUOUS 0x06
#define SANDBVOLT_CONTINUOUS 0x07

#define INA219_I2C_ADDR 0x42

#define CALIBRATION_VALUE 4096u

struct INA219_Context {
  float bus_voltage;
  float shunt_voltage;
  float current;
  float power;
  // 0: uninitialized, 1: charging, -1: discharging
  float prev_current;
  float batt_percentage;
  // batt_percentage_t0 and t0 are reset on start and each time charging status
  // is changed
  float batt_percentage_t0;
  // batt_percentage_t0 and t0 are reset on start and each time charging status
  // is changed
  time_t t0;
};

int _ina219_i2c_fd;

int ina219_write(uint8_t reg_addr, uint16_t data) {
  // The implementation of write() is tricky, LLMs can't get it right
  // The current implementation takes reference from here:
  // https://github.com/flav1972/ArduinoINA219/blob/5194f33ae9edc0e99e0cb1a6ed62e41818886fa9/INA219.cpp#L272-L296
  // But unfortunately Arduino still uses layers of layers of abstraction
  uint8_t buf[3];
  buf[0] = reg_addr;
  buf[1] = (data >> 8) & 0xFF;
  buf[2] = data & 0xFF;
  return write(_ina219_i2c_fd, buf, 3);
}

int ina219_read(uint8_t reg, uint16_t *data) {
  uint8_t buf[2];
  if (write(_ina219_i2c_fd, &reg, 1) != 1) {
    return -1;
  }
  if (read(_ina219_i2c_fd, buf, 2) != 2) {
    return -1;
  }
  *data = (buf[0] << 8) | buf[1];
  return 0;
}

int ina219_init(void) {
  _ina219_i2c_fd = open("/dev/i2c-1", O_RDWR);
  if (_ina219_i2c_fd < 0) {
    SYSLOG_ERR("Failed to open I2C device");
    return 1;
  }
  if (ioctl(_ina219_i2c_fd, I2C_SLAVE, INA219_I2C_ADDR) < 0) {
    SYSLOG_ERR("Failed to set I2C slave address");
    goto err_whatever;
  }

  if (ina219_write(_REG_CALIBRATION, CALIBRATION_VALUE) < 0) {
    SYSLOG_ERR("Failed to set CALIBRATION_VALUE");
    goto err_whatever;
  }

  uint16_t config = RANGE_32V << 13 | DIV_8_320MV << 11 |
                    ADCRES_12BIT_32S << 7 | ADCRES_12BIT_32S << 3 |
                    SANDBVOLT_CONTINUOUS;
  if (ina219_write(_REG_CONFIG, config) < 0) {
    SYSLOG_ERR("Failed to set config");
    goto err_whatever;
  }
  return 0;
err_whatever:
  close(_ina219_i2c_fd);
  return -1;
}

int ina219_get_shunt_voltage_mv(float *reading) {
  uint16_t data;
  if (ina219_write(_REG_CALIBRATION, CALIBRATION_VALUE) < 0)
    return -1;
  if (ina219_read(_REG_SHUNTVOLTAGE, &data) < 0)
    return -2;
  float shunt_voltage = data;
  if (shunt_voltage > 0x8000) {
    shunt_voltage -= 0xFFFF;
  }
  *reading = shunt_voltage * 0.01;
  return 0;
}

int ina219_get_bus_voltage_v(float *reading) {
  uint16_t data;
  if (ina219_write(_REG_CALIBRATION, CALIBRATION_VALUE) < 0)
    return -1;
  if (ina219_read(_REG_BUSVOLTAGE, &data) != 0)
    return -2;
  *reading = (float)(data >> 3) * 0.004;
  return 0;
}

int ina219_get_current_ma(float *reading) {
  uint16_t data;
  if (ina219_read(_REG_CURRENT, &data) < 0)
    return -1;
  float currnet_lsb = 0.1;
  float current = (float)data;
  if (current > 0x8000) {
    current -= 0xFFFF;
  }
  *reading = current * currnet_lsb;
  return 0;
}

int ina219_get_power_w(float *reading) {
  uint16_t data;
  if (ina219_write(_REG_CALIBRATION, CALIBRATION_VALUE) < 0)
    return -1;
  if (ina219_read(_REG_POWER, &data) < 0)
    return -2;
  float power_lsb = 0.002;
  float power = (float)data;
  if (power > 0x8000) {
    power -= 0xFFFF;
  }
  *reading = power * power_lsb;
  return 0;
}

void *post_collection_init(__attribute__((unused)) const json_object *config) {
  // We dont need any post_collection context, but to fit in the framework...
  char *ctx = malloc(sizeof(char));
  if (ctx == NULL)
    return NULL;
  printf("          Timestamp,      Status, Curr. (mA), Power (W), Batt. (%%), "
         "Hourly use (%%), Remaining (Hrs)\n");
  return ctx;
}

int post_collection(void *c_ctx, __attribute__((unused)) void *pc_ctx) {
  struct INA219_Context *ctx = (struct INA219_Context *)c_ctx;
  time_t datetime_now;
  // struct tm *timeinfo;
  char dt_now_str[sizeof("1970-01-01T00:00:00")];
  char dt_eta_str[sizeof("1970-01-01T00:00:00")];
  time(&datetime_now);
  strftime(dt_now_str, sizeof(dt_now_str), "%Y-%m-%dT%H:%M:%S",
           localtime(&datetime_now));
  float batt_use_sec = (ctx->batt_percentage_t0 - ctx->batt_percentage) /
                       (datetime_now - ctx->t0 + 1);
  float hourly_batt_use = 3600.0 * batt_use_sec;

  printf("%s, %s, %10.0f, %9.3f, %7.1f%%, %13.1f%%, ", dt_now_str,
         ctx->current > 0 ? "   Charging" : "Discharging", ctx->current,
         ctx->power, ctx->batt_percentage, hourly_batt_use);
  if (batt_use_sec != 0) {
    // If the battery is charging, it is the time until fully charged
    // If the battery is discharging, it is the time until fully depleted.
    float remaining_batt_hrs;
    if (ctx->current < 0) {
      remaining_batt_hrs = ctx->batt_percentage / batt_use_sec / 3600.0;
      datetime_now += ctx->batt_percentage / batt_use_sec;
    } else {
      remaining_batt_hrs =
          (100 - ctx->batt_percentage) / batt_use_sec / 3600.0 * -1;
      datetime_now += (100 - ctx->batt_percentage) / batt_use_sec * -1;
    }

    strftime(dt_eta_str, sizeof(dt_eta_str), "%Y-%m-%dT%H:%M:%S",
             localtime(&datetime_now));
    printf("%15.1f(%s at %s)", remaining_batt_hrs,
           ctx->current > 0 ? "Fully charged" : "Depleted", dt_eta_str);
  } else {
    // Mostly due to sampling with small intervals
    printf("%15s", "<NA>");
  }
  printf("\n");
  return 0;
}

void post_collection_destroy(void *pc_ctx) { free((char *)pc_ctx); }

void *collection_init(__attribute__((unused)) const json_object *config) {
  if (ina219_init() != 0) {
    SYSLOG_ERR("ina219_init() failed");
    return NULL;
  }
  struct INA219_Context *ctx =
      (struct INA219_Context *)malloc(sizeof(struct INA219_Context));
  if (ctx == NULL) {
    return ctx;
  }
  ctx->prev_current = 0;
  ctx->t0 = time(NULL);
  return ctx;
}

int collection(void *ctx) {
  struct INA219_Context *dat = (struct INA219_Context *)ctx;
  if (ina219_get_bus_voltage_v(&dat->bus_voltage) != 0)
    return 1;
  if (ina219_get_shunt_voltage_mv(&dat->shunt_voltage) != 0)
    return 1;
  dat->shunt_voltage /= 1000.0;
  if (ina219_get_current_ma(&dat->current) != 0)
    return 1;
  if (ina219_get_power_w(&dat->power) != 0)
    return 1;
  dat->batt_percentage = (dat->bus_voltage - 6) / 2.4 * 100;
  if (dat->prev_current * dat->current <= 0) {
    // dat->prev_charging_status is 0 or
    // dat->prev_charging_status and dat->current have the same sign
    dat->t0 = time(NULL);
    dat->batt_percentage_t0 = dat->batt_percentage;
  }
  dat->prev_current = dat->current;
  dat->batt_percentage =
      dat->batt_percentage > 100 ? 100 : dat->batt_percentage;
  dat->batt_percentage = dat->batt_percentage < 0 ? 0 : dat->batt_percentage;
  // printf("%f,%f\n", dat->batt_percentage_t0, dat->batt_percentage);
  return 0;
}

void collection_destroy(void *ctx) { free((struct INA219_Context *)ctx); }
