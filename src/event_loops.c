#include "global_vars.h"
#include "module.h"
#include "utils.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>

void ev_collect_data() {
  syslog(LOG_INFO, "ev_collect_data() started");

  struct CollectionContext c_ctx = collection_init(gv_config_root);
  if (!c_ctx.init_success) {
    ev_flag = 1;
    SYSLOG_ERR("collection_init() initialization failed, sdp will exit now");
    goto err_collection_init;
  }
  syslog(LOG_INFO, "collection_init() returned without errors");

  struct PostCollectionContext pc_ctx;
  if (!(pc_ctx = post_collection_init(gv_config_root)).init_success) {
    SYSLOG_ERR(
        "post_collection_init() failed, post collection task will not run");
  }
  syslog(LOG_INFO, "post_collection_init() returned without errors");

  while (!ev_flag) {
    // You need to have sleep at the beginning so that continue branch will also
    // trigger this
    interruptible_sleep_us(gv_collection_event_interval_us);
    int ret;
    if ((ret = collection(&c_ctx)) < 0) {
      ev_flag = 1;
      SYSLOG_ERR("collection() encounters a fatal error (ret: %d)", ret);
      break;
    }
    if (ret > 0)
      syslog(LOG_WARNING,
             "collection() encounters a recoverable error (ret: %d), "
             "post_collection() call will be skipped",
             ret);
    if (!pc_ctx.init_success)
      continue;

    post_collection(&c_ctx, &pc_ctx);
  }
  if (pc_ctx.init_success)
    post_collection_destroy(&pc_ctx);
  // if (c_ctx.init_success) is not needed as we use goto to handle this case
  collection_destroy(&c_ctx);
err_collection_init:
  syslog(LOG_INFO, "ev_collect_data() exited gracefully.");
}
