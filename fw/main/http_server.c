#include "http_server.h"

#include <cJSON.h>
#include <esp_chip_info.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_random.h>
#include <string.h>

#include "data.h"
#include "util.h"

#define TAG "http_server"

/* Simple handler for getting system handler */
static esp_err_t status_get_handler(httpd_req_t* req)
{
    httpd_resp_set_type(req, "application/json");
    cJSON* root = cJSON_CreateObject();
    ESP_ERROR_CHECK(data_status_to_json(root));
    const char* resp = cJSON_PrintUnformatted(root);
    httpd_resp_sendstr(req, resp);
    free((void*)resp);
    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t http_server_init(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server");
    ERROR_CHECK(httpd_start(&server, &config) == ESP_OK, "Start server failed", err);

    httpd_uri_t system_info_get_uri = {
        .uri = "/api/v1/status",
        .method = HTTP_GET,
        .handler = status_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &system_info_get_uri);

    return ESP_OK;
err:
    return ESP_FAIL;
}