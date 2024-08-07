#include "../global_vars.h"
#include "../module.h"
#include "../module_lib.h"
#include "../utils.h"

#include <iotctrl/7segment-display.h>
#include <iotctrl/dht31.h>

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

struct TempAndRHReadings {
  double temp_celsius;
  double relative_humidity;
  time_t update_time;
};

struct DHT31Handle {
  struct TempAndRHReadings readings;
  char *device_path;
};

void *post_collection_init(const json_object *config) {
  const json_object *root = config;
  json_object *root_7sd;
  json_object_object_get_ex(root, "7seg_display", &root_7sd);
  struct iotctrl_7seg_disp_handle *h = init_7seg_from_json(root_7sd);
  return h;
}

int post_collection(void *c_ctx, void *pc_ctx) {

  struct TempAndRHReadings r = ((struct DHT31Handle *)c_ctx)->readings;

  struct iotctrl_7seg_disp_handle *h =
      (struct iotctrl_7seg_disp_handle *)pc_ctx;
  iotctrl_7seg_disp_update_as_four_digit_float(h, r.temp_celsius, 0);
  iotctrl_7seg_disp_update_as_four_digit_float(h, r.relative_humidity, 1);
  return 0;
}

void post_collection_destroy(void *ctx) {
  iotctrl_7seg_disp_destroy((struct iotctrl_7seg_disp_handle *)(ctx));
}

void *collection_init(const json_object *config) {
  struct DHT31Handle *h = malloc(sizeof(struct DHT31Handle));
  if (h == NULL) {
    SYSLOG_ERR("malloc() failed");
    goto err_malloc_handle;
  }

  const json_object *root = config;
  json_object *root_sht31_device_path;
  json_object_object_get_ex(root, "dht31_device_path", &root_sht31_device_path);
  // TODO, the use of json_object_get_string() here could result in segmentfault
  // if JSON config file format is unexpected. Search the below line to check
  // out the correct handling
  // SYSLOG_ERR("d->device_path initialization failed");
  const char *device_path = json_object_get_string(root_sht31_device_path);
  if (device_path != NULL)
    h->device_path = malloc(strlen(device_path) + 1);
  else
    h->device_path = NULL;
  if (h->device_path == NULL) {
    SYSLOG_ERR("h->device_path initialization failed");
    goto err_malloc_device_path;
  }
  strcpy(h->device_path, device_path);

  h->readings.temp_celsius = 888.8;
  h->readings.relative_humidity = 888.8;
  h->readings.update_time = -1;

  return h;

err_malloc_device_path:
  free(h);
err_malloc_handle:
  return NULL;
}

int collection(void *ctx) {
  struct DHT31Handle *dht31 = (struct DHT31Handle *)ctx;
  float temp_celsius_t;
  float relative_humidity_t;
  int ret = 0;
  int fd = iotctrl_dht31_init(dht31->device_path);
  if ((ret = iotctrl_dht31_read(fd, &temp_celsius_t, &relative_humidity_t)) !=
      0) {
    goto err_dht31_read;
    ret = 1;
  }

  dht31->readings.temp_celsius = temp_celsius_t;
  dht31->readings.relative_humidity = relative_humidity_t;
  dht31->readings.update_time = time(NULL);
  syslog(LOG_INFO, "Readings changed to temp: %.1f°C, RH: %.1f%%",
         temp_celsius_t, relative_humidity_t);
  if (dht31->readings.update_time == -1)
    SYSLOG_ERR("Failed to get time(): %d(%s)", errno, strerror(errno));

err_dht31_read:
  iotctrl_dht31_destroy(fd);
  return ret;
}

void collection_destroy(void *ctx) {

  struct DHT31Handle *dht31 = (struct DHT31Handle *)ctx;
  free(dht31->device_path);
  dht31->device_path = NULL;
  free(dht31);
  dht31 = NULL;
}
