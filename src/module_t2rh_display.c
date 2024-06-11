#include "global_vars.h"
#include "module.h"
#include "module_lib.h"
#include "utils.h"

#include <iotctrl/7segment-display.h>
#include <iotctrl/dht31.h>
#include <iotctrl/temp-sensor.h>

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/i2c-dev.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>

struct T2RHReadings {
  double temp_celsius0;
  double temp_celsius1;
  double relative_humidity;
};

struct ConnectionInfo {
  struct T2RHReadings readings;
  char *dht31_device_path;
  char *dl11_device_path;
};

struct Twp7segDispHandles {
  struct iotctrl_7seg_disp_handle *h0;
  struct iotctrl_7seg_disp_handle *h1;
};

struct PostCollectionContext post_collection_init(const json_object *config) {

  struct PostCollectionContext ctx;
  ctx.init_success = true;

  const json_object *root = config;
  json_object *root_7sd;
  json_object_object_get_ex(root, "7seg_display0", &root_7sd);
  struct iotctrl_7seg_disp_handle *h0 = load_and_init_7seg(root_7sd);
  if (h0 == NULL) {
    goto err_h0_error;
  }

  json_object_object_get_ex(root, "7seg_display1", &root_7sd);
  struct iotctrl_7seg_disp_handle *h1 = load_and_init_7seg(root_7sd);
  if (h1 == NULL) {
    goto err_h1_error;
  }

  struct Twp7segDispHandles *t = malloc(sizeof(struct Twp7segDispHandles));
  if (t == NULL) {
    goto err_malloc_error;
  }
  t->h0 = h0;
  t->h1 = h1;
  ctx.context = t;
  return ctx;

err_malloc_error:
  iotctrl_7seg_disp_destroy(h1);
err_h1_error:
  iotctrl_7seg_disp_destroy(h0);
err_h0_error:
  ctx.init_success = false;
  return ctx;
}

int post_collection(struct CollectionContext *c_ctx,
                    struct PostCollectionContext *pc_ctx) {

  struct T2RHReadings r = ((struct ConnectionInfo *)c_ctx->context)->readings;

  struct Twp7segDispHandles *handles =
      (struct Twp7segDispHandles *)pc_ctx->context;
  iotctrl_7seg_disp_update_as_four_digit_float(handles->h0, r.temp_celsius0, 0);
  iotctrl_7seg_disp_update_as_four_digit_float(handles->h0, r.relative_humidity,
                                               1);
  iotctrl_7seg_disp_update_as_four_digit_float(handles->h1, r.temp_celsius0, 0);
  iotctrl_7seg_disp_update_as_four_digit_float(handles->h1, r.temp_celsius1, 1);
  return 0;
}

void post_collection_destroy(struct PostCollectionContext *ctx) {

  struct Twp7segDispHandles *twoHandles =
      (struct Twp7segDispHandles *)ctx->context;
  iotctrl_7seg_disp_destroy(twoHandles->h0);
  iotctrl_7seg_disp_destroy(twoHandles->h1);
  free(ctx->context);
}

struct CollectionContext collection_init(const json_object *config) {
  struct CollectionContext ctx = {.init_success = true, .context = NULL};
  struct ConnectionInfo *conn = malloc(sizeof(struct ConnectionInfo));
  if (conn == NULL) {
    SYSLOG_ERR("malloc() failed");
    goto err_malloc_conn;
  }

  const json_object *root = config;
  json_object *root_dht31_device_path;
  json_object_object_get_ex(root, "dht31_device_path", &root_dht31_device_path);
  const char *device_path;

  // TODO, the use of json_object_get_string() here could result in segmentfault
  // if JSON config file format is unexpected. Search the below line to check
  // out the correct handling
  // SYSLOG_ERR("d->device_path initialization failed");
  device_path = json_object_get_string(root_dht31_device_path);
  conn->dht31_device_path = malloc(strlen(device_path) + 1);
  if (conn->dht31_device_path == NULL) {
    SYSLOG_ERR("malloc() failed");
    goto err_malloc_dht31_path;
  }
  strcpy(conn->dht31_device_path, device_path);

  json_object *root_dl11_device_path;
  json_object_object_get_ex(root, "dl11_device_path", &root_dl11_device_path);
  // TODO, the use of json_object_get_string() here could result in segmentfault
  // if JSON config file format is unexpected. Search the below line to check
  // out the correct handling
  // SYSLOG_ERR("d->device_path initialization failed");
  device_path = (char *)json_object_get_string(root_dl11_device_path);
  conn->dl11_device_path = malloc(strlen(device_path) + 1);
  if (conn->dl11_device_path == NULL) {
    SYSLOG_ERR("malloc() failed");
    goto err_malloc_dl11_path;
  }
  strcpy(conn->dl11_device_path, device_path);

  conn->readings.temp_celsius0 = 888.8;
  conn->readings.temp_celsius1 = 888.8;
  conn->readings.relative_humidity = 888.8;
  ctx.context = conn;
  syslog(
      LOG_INFO,
      "collection_init() success, dht31_device_path: %s, dl11_device_path: %s",
      conn->dht31_device_path, conn->dl11_device_path);
  return ctx;

err_malloc_dl11_path:
  free(conn->dht31_device_path);
err_malloc_dht31_path:
  free(conn);
err_malloc_conn:
  ctx.init_success = false;
  return ctx;
}

int collection(struct CollectionContext *ctx) {
  struct ConnectionInfo *conn = (struct ConnectionInfo *)ctx->context;
  float temp_celsius_t;
  float relative_humidity_t;
  int ret = 0;
  const uint8_t sensor_count = 1;
  int16_t readings[sensor_count];
  int fd = iotctrl_dht31_init(conn->dht31_device_path);

  if ((ret = iotctrl_dht31_read(fd, &temp_celsius_t, &relative_humidity_t)) !=
      0) {
    ret = 1;
    goto err_dht31_read;
  }
  conn->readings.temp_celsius0 = temp_celsius_t;
  conn->readings.relative_humidity = relative_humidity_t;

  if (iotctrl_get_temperature(conn->dl11_device_path, sensor_count, readings,
                              1) != 0) {
    ret = 2;
    goto err_dl11_read;
  }
  conn->readings.temp_celsius1 = readings[0] / 10.0;

  syslog(LOG_INFO,
         "Readings changed to temp0: %.1f°C, temp1: %.1f°C, RH: %.1f%%",
         conn->readings.temp_celsius0, conn->readings.temp_celsius1,
         conn->readings.relative_humidity);
err_dl11_read:
err_dht31_read:
  iotctrl_dht31_destroy(fd);
  return ret;
}

void collection_destroy(struct CollectionContext *ctx) {

  struct ConnectionInfo *conn = (struct ConnectionInfo *)ctx->context;
  free(conn->dht31_device_path);
  conn->dht31_device_path = NULL;
  free(conn->dl11_device_path);
  conn->dl11_device_path = NULL;
  free(conn);
  conn = NULL;
}
