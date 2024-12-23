#include "../../utils.h"
#include "../libs/mqtt.h"
#include "../module.h"

#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/cURLpp.hpp>
#include <fmt/core.h>
#include <mosquitto.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <sstream>
#include <stdbool.h>
#include <stdio.h>
#include <sys/syslog.h>
#include <thread>

using json = nlohmann::json;
using namespace curlpp::options;

struct PostCollectionCtx {
  struct mosquitto *mosq;
  const char *topic;
};
struct CollectionCtx {
  json payload;
};

void *post_collection_init(const json_object *config) {

  auto ctx = new struct PostCollectionCtx();
  struct json_object *json_ele;
  const char *host = NULL;
  const char *username = NULL;
  const char *password = NULL;
  const char *ca_file_path = NULL;
  json_pointer_get((json_object *)config, "/hko/host", &json_ele);
  host = json_object_get_string(json_ele);
  json_pointer_get((json_object *)config, "/hko/username", &json_ele);
  username = json_object_get_string(json_ele);
  json_pointer_get((json_object *)config, "/hko/password", &json_ele);
  password = json_object_get_string(json_ele);
  json_pointer_get((json_object *)config, "/hko/ca_file_path", &json_ele);
  ca_file_path = json_object_get_string(json_ele);
  json_pointer_get((json_object *)config, "/hko/topic", &json_ele);
  ctx->topic = json_object_get_string(json_ele);
  if (host == NULL || ca_file_path == NULL || username == NULL ||
      password == NULL || ctx->topic == NULL) {
    SYSLOG_ERR("Invalid configs");
    delete ctx;
    ctx = NULL;
    return NULL;
  }
  ctx->mosq = init_mosquitto(host, ca_file_path, username, password);
  if (ctx->mosq == NULL) {
    SYSLOG_ERR("init_mosquitto() failed");
    delete ctx;
    ctx = NULL;
    return NULL;
  }
  return ctx;
}

int post_collection(void *c_ctx, void *pc_ctx) {
  struct CollectionCtx *_c_ctx = (struct CollectionCtx *)c_ctx;
  struct PostCollectionCtx *_pc_ctx = (struct PostCollectionCtx *)pc_ctx;

  mosquitto_publish(_pc_ctx->mosq, NULL, _pc_ctx->topic,
                    _c_ctx->payload.dump().length(),
                    _c_ctx->payload.dump().c_str(), 2, false);
  return 0;
}

void post_collection_destroy(void *ctx) {
  auto _ctx = (struct PostCollectionCtx *)ctx;
  if (ctx == NULL)
    return;
  mosquitto_destroy(_ctx->mosq);
  mosquitto_lib_cleanup();
  // free(_ctx);
  delete _ctx;
}

void *collection_init(const json_object *config) {
  (void)config;
  auto ctx = new struct CollectionCtx();
  // ctx->payload = json::parse(R"({ })");
  if (ctx == NULL)
    return NULL;
  return ctx;
}

int collection(void *ctx) {
  auto _ctx = (struct CollectionCtx *)ctx;
  std::ostringstream os;
  try {
    os << curlpp::options::Url("https://data.weather.gov.hk/weatherAPI/"
                               "opendata/weather.php?dataType=rhrread&lang=en");
    auto j = json::parse(os.str());
    json data;
    auto place = "Happy Valley";
    for (const auto &_data : j["temperature"]["data"]) {
      if (_data.value("/place"_json_pointer, "") == place) {
        data = _data;
      }
    }
    if (data.value("/place"_json_pointer, "") != place) {
      throw std::invalid_argument(
          fmt::format("Payload does not contain place == {}", place));
    }
    if (data.value("/unit"_json_pointer, "") != "C") {
      throw std::invalid_argument(fmt::format(
          "Unexpected unit: {}", data.value("/unit"_json_pointer, "")));
    }
    syslog(LOG_INFO, "Data from HK gov: recordTime: %s, air temp: %f°C",
           j["temperature"]["recordTime"].get<std::string>().c_str(),
           data["value"].get<float>());
    auto now = std::chrono::system_clock::now();
    auto itt = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::gmtime(&itt), "%Y-%m-%dT%H:%M:%SZ");
    _ctx->payload["fh_timestamp"] = ss.str();
    _ctx->payload["hko_timestamp"] = j["temperature"]["recordTime"];
    _ctx->payload["temp_celsius"] = data["value"];
    return 0;
  } catch (const std::exception &e) {
    SYSLOG_ERR("C++ exception: %s", e.what());
  }
  return 1;
}

void collection_destroy(void *ctx) {
  if (ctx == NULL)
    return;
  struct CollectionCtx *_ctx = (struct CollectionCtx *)ctx;
  delete _ctx;
}
