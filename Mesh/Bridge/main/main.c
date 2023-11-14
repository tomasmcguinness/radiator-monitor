/*
 * main.c
 *
 * Copyright Tomás McGuinness 2023
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <pthread.h>
#include <math.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_mac.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "mesh.h"
#include "mesh_main.h"
#include "crypto.h"
#include "access.h"
#include "board.h"
#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"

#include "mqtt_client.h"

#include <esp_http_server.h>

#include "cJSON.h"

#include "esp_system.h"
#include "esp_spiffs.h"

#include "nvs_flash.h"

#include "esp_vfs.h"

#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>
#include "qrcode.h"

#include "mdns.h"

#include "esp_netif_sntp.h"
#include "esp_sntp.h"

#define TAG "BRIDGE"

static void obtain_time(void);
static void initialize_sntp(void);

static httpd_handle_t server = NULL;
static int ws_socket;

static nvs_handle_t NVS_HANDLE;

const int BALANCING_PHASE_NONE = 0;
const int BALANCING_PHASE_CALIBRATING = 1;
const int BALANCING_PHASE_ORDERING = 2;
const int BALANCING_PHASE_BALANCING = 3;

static bool is_balancing = false;
static int balancing_phase = 0;
static int ordering_count = 0;

/***************
 * COMMON
 ***************/

struct radiator
{
    int number;
    bool swap_sensors;
    float flow_temperature;
    float return_temperature;
    float mean_temperature;
    int last_seen;
    bool is_warming;
    bool is_calibrated;
    float calibrated_flow_temperature;
    float calibrated_return_temperature;
    int warming_order;
};

typedef struct radiator Radiator;

static int radiator_count = 0;
static Radiator radiators[20];

static pthread_mutex_t lock;

static Radiator get_radiator(int number)
{
    Radiator rad;
    int found = 0;
    pthread_mutex_lock(&lock);

    for (int i = 0; i < radiator_count; i++)
    {
        if (radiators[i].number == number)
        {
            found = 1;
            rad = radiators[i];
            break;
        }
    }

    if (found == 0)
    {
        Radiator newRad = {
            .number = number};

        rad = newRad;
    }

    pthread_mutex_unlock(&lock);

    return rad;
}

static void add_or_update_radiator(Radiator r)
{
    int found = 0, i = 0;

    pthread_mutex_lock(&lock);

    for (i = 0; i < radiator_count; i++)
    {
        if (radiators[i].number == r.number)
        {
            found = 1;
            break;
        }
    }

    if (found == 1)
    {
        radiators[i].flow_temperature = r.flow_temperature;
        radiators[i].return_temperature = r.return_temperature;
        radiators[i].mean_temperature = r.mean_temperature;
        radiators[i].last_seen = r.last_seen;

        radiators[i].is_calibrated = r.is_calibrated;
        radiators[i].calibrated_flow_temperature = r.calibrated_flow_temperature;
        radiators[i].calibrated_return_temperature = r.calibrated_return_temperature;

        radiators[i].is_warming = r.is_warming;
        radiators[i].warming_order = r.warming_order;
    }
    else
    {
        radiators[radiator_count++] = r;
    }

    pthread_mutex_unlock(&lock);
}

int get_room_name(int number, char *room_name)
{
    char handle[15];
    sprintf(handle, "room_name_%d", number);

    size_t room_name_length;

    int err = nvs_get_str(NVS_HANDLE, &handle, NULL, &room_name_length);

    if (err == ESP_OK)
    {
        err = nvs_get_str(NVS_HANDLE, &handle, room_name, &room_name_length);
        room_name[room_name_length] = '/0';
    }
    else
    {
        sprintf(room_name, "Room %d", number);
        err = ESP_OK;
    }

    return err;
}

void get_room_name_topic(int number, char *room_name, char *room_name_topic)
{
    strcpy(room_name_topic, room_name);

    for (int i = 0; room_name_topic[i]; i++)
    {
        room_name_topic[i] = tolower(room_name_topic[i]);

        if (room_name_topic[i] == ' ')
        {
            room_name_topic[i] = '_';
        }
    }
}

bool get_swap_sensors(int number)
{
    char handle[15];
    sprintf(handle, "swap_sensors_%d", number);

    int8_t swap_sensors = 0;

    nvs_get_i8(NVS_HANDLE, &handle, &swap_sensors);

    return swap_sensors;
}

/***************
 * BLUETOOTH MESH
 ***************/
#define CID_ESP 0x02E5

static uint8_t dev_uuid[16] = {0xdd, 0xdd};

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

#define ESP_BLE_MESH_VND_COMPANY_ID 0x0059
#define ESP_BLE_MESH_VND_MODEL_ID 0x000A

#define ESP_BLE_MESH_VND_MODEL_OP_SET ESP_BLE_MESH_MODEL_OP_3(0x0A, ESP_BLE_MESH_VND_COMPANY_ID)
#define ESP_BLE_MESH_VND_MODEL_OP_SET_STATUS ESP_BLE_MESH_MODEL_OP_3(0x0B, ESP_BLE_MESH_VND_COMPANY_ID)

static const esp_ble_mesh_client_op_pair_t vnd_op_pair[] = {
    {ESP_BLE_MESH_VND_MODEL_OP_SET, ESP_BLE_MESH_VND_MODEL_OP_SET_STATUS},
};

static esp_ble_mesh_client_t vendor_client = {
    .op_pair_size = ARRAY_SIZE(vnd_op_pair),
    .op_pair = vnd_op_pair,
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    {ESP_BLE_MESH_VND_MODEL_OP_SET, 0, 0},
    {ESP_BLE_MESH_VND_MODEL_OP_SET_STATUS, 0, 0},
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(ESP_BLE_MESH_VND_COMPANY_ID, ESP_BLE_MESH_VND_MODEL_ID, vnd_op, NULL, &vendor_client),
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
}; 
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models)
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
    .output_size = 0,
    .output_actions = 0,
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

esp_mqtt_client_handle_t _mqtt_client;

static void ws_async_send(void *arg)
{
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    ws_pkt.payload = (uint8_t *)arg;
    ws_pkt.len = strlen(arg);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.final = true;

    httpd_ws_send_frame_async(server, ws_socket, &ws_pkt);

    free(arg);
}

static int calibrated_radiator_count = 0;

static bool is_mqtt_connected = false;

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    uint8_t *msg = (uint8_t *)param->model_operation.msg;
    uint16_t length = (uint16_t)param->model_operation.length;

    if(length != 5) {
        ESP_LOGI(TAG, "Payload is not the expected size");
        return;
    }

    short number = msg[0];

    int16_t sensor_one_value;
    int16_t sensor_two_value;

    memcpy(&sensor_one_value, &msg[1], 2);
    memcpy(&sensor_two_value, &msg[3], 2);

    Radiator rad = get_radiator(number);

    float new_flow_temperature;
    float new_return_temperature;

    bool is_swapped = get_swap_sensors(number);

    ESP_LOGI(TAG, "Sensor %d: swap: %d", number, is_swapped);

    if (is_swapped)
    {
        new_flow_temperature = (float)sensor_two_value / 10;
        new_return_temperature = (float)sensor_one_value / 10;
    }
    else
    {
        new_flow_temperature = (float)sensor_one_value / 10;
        new_return_temperature = (float)sensor_two_value / 10;
    }

    bool send_status_change = false;

    if (is_balancing)
    {
        switch (balancing_phase)
        {
        case BALANCING_PHASE_CALIBRATING:
            // Once all rads have checked in,
            //
            if (!rad.is_calibrated)
            {
                ESP_LOGI(TAG, "Calibrating radiator %d with a flow temp of %f and return temp of %f", rad.number, new_flow_temperature, new_return_temperature);
                rad.calibrated_flow_temperature = new_flow_temperature;
                rad.calibrated_return_temperature = new_return_temperature;
                rad.is_calibrated = true;
                calibrated_radiator_count++;

                ESP_LOGI(TAG, "Calibrated %d of %d", calibrated_radiator_count, radiator_count);
            }

            if (calibrated_radiator_count == radiator_count)
            {
                ESP_LOGI(TAG, "Moving to ordering phase");

                balancing_phase = BALANCING_PHASE_ORDERING;
                send_status_change = true;
                calibrated_radiator_count = 0;
            }
            break;
        case BALANCING_PHASE_ORDERING:
            if (!rad.is_warming)
            {
                bool has_flow_changed = new_flow_temperature > (rad.calibrated_flow_temperature + 1);
                bool has_return_changed = new_return_temperature > (rad.calibrated_return_temperature + 1);

                if (has_flow_changed || has_return_changed)
                {
                    rad.warming_order = ordering_count++;
                    rad.is_warming = true;

                    ESP_LOGI(TAG, "Radiator %d has started warming.  It is order %d", rad.number, rad.warming_order);
                    ESP_LOGI(TAG, "Radiator %d has flow temp of %f vs %f", rad.number, new_flow_temperature, rad.calibrated_flow_temperature);
                    ESP_LOGI(TAG, "Radiator %d has return temp of %f vs %f", rad.number, new_return_temperature, rad.calibrated_return_temperature);
                }
            }

            if (ordering_count == radiator_count)
            {
                ESP_LOGI(TAG, "All %d radiators have started warming. Moving to balancing phase", radiator_count);
                balancing_phase = BALANCING_PHASE_BALANCING;
                send_status_change = true;
            }
            break;
        }

        if (send_status_change)
        {
            cJSON *root = NULL;
            root = cJSON_CreateObject();

            cJSON_AddStringToObject(root, "type", "status_update");
            cJSON_AddNumberToObject(root, "phase", balancing_phase);

            char *payload = cJSON_PrintUnformatted(root);

            httpd_queue_work(server, ws_async_send, payload);

            cJSON_Delete(root);
        }
    }

    rad.flow_temperature = new_flow_temperature;
    rad.return_temperature = new_return_temperature;
    rad.mean_temperature = (new_flow_temperature + new_return_temperature) / 2;
    rad.last_seen = (int)time(NULL);

    add_or_update_radiator(rad);

    cJSON *root = NULL;
    root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "type", "radiator_update");

    char print_num[18];

    snprintf(print_num, 18, "%d", number);
    cJSON_AddRawToObject(root, "number", print_num);

    snprintf(print_num, 18, "%.1f", rad.flow_temperature);
    cJSON_AddRawToObject(root, "flow_temperature", print_num);

    snprintf(print_num, 18, "%.1f", rad.return_temperature);
    cJSON_AddRawToObject(root, "return_temperature", print_num);

    snprintf(print_num, 18, "%.1f", rad.mean_temperature);
    cJSON_AddRawToObject(root, "mean_temperature", print_num);

    cJSON_AddNumberToObject(root, "last_seen", rad.last_seen);

    cJSON_AddBoolToObject(root, "is_calibrated", rad.is_calibrated);

    cJSON_AddBoolToObject(root, "is_warming", rad.is_warming);

    cJSON_AddNumberToObject(root, "warming_order", rad.warming_order);

    float delta_t = fabs(rad.flow_temperature - rad.return_temperature);
    snprintf(print_num, 18, "%.1f", delta_t);
    cJSON_AddRawToObject(root, "delta_t", print_num);

    char *room_name = malloc(128);

    int err = get_room_name(rad.number, room_name);

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "room_name", room_name);
    }

    char *payload = cJSON_PrintUnformatted(root);

    char *room_name_topic = malloc(strlen(room_name) + 1);

    get_room_name_topic(rad.number, room_name, room_name_topic);

    if(is_mqtt_connected) 
    {
        char topic[128];
        sprintf(topic, "radiators/%s", room_name_topic);

        ESP_LOGI(TAG, "Publishing to MQTT topic %s", topic);
        esp_mqtt_client_publish(_mqtt_client, &topic, payload, 0, 0, 0);
    }

    httpd_queue_work(server, ws_async_send, payload);

    cJSON_Delete(root);

    free(room_name_topic);
    free(room_name);

    // This is freed by the ws_async_send
    // free(payload);
}

struct bt_mesh_device_network_info
{
    uint8_t net_key[16];
    uint16_t net_idx;
    uint8_t flags;
    uint32_t iv_index;
    uint16_t unicast_addr;
    uint8_t dev_key[16];
    uint8_t app_key[16];
    uint16_t app_idx;
    uint16_t group_addr;
};

int bt_mesh_device_auto_enter_network(struct bt_mesh_device_network_info *info)
{
    BT_INFO("bt_mesh_device_auto_enter_network()");

    const struct bt_mesh_comp *comp = NULL;
    struct bt_mesh_model *model = NULL;
    struct bt_mesh_elem *elem = NULL;
    struct bt_mesh_app_keys *keys = NULL;
    struct bt_mesh_app_key *key = NULL;
    struct bt_mesh_subnet *sub = NULL;
    int i, j, k;
    int err = 0;

    BT_INFO("bt_mesh_atomic_set_bit()");

    bt_mesh_atomic_set_bit(bt_mesh.flags, BLE_MESH_NODE);

    BT_INFO("bt_mesh_provision()");

    err = bt_mesh_provision(info->net_key, info->net_idx, info->flags, info->iv_index, info->unicast_addr, info->dev_key);

    if (err)
    {
        BT_ERR("bt_mesh_provision() failed (err %d)", err);
        return err;
    }

    sub = bt_mesh_subnet_get(info->net_idx);

    if (!sub)
    {
        BT_ERR("Invalid NetKeyIndex 0x%04x", info->net_idx);
        return -1;
    }

    for (i = 0; i < ARRAY_SIZE(bt_mesh.app_keys); i++)
    {
        key = &bt_mesh.app_keys[i];
        if (key->net_idx == BLE_MESH_KEY_UNUSED)
        {
            break;
        }
    }
    if (i == ARRAY_SIZE(bt_mesh.app_keys))
    {
        BT_ERR("Failed to allocate AppKey, 0x%04x", info->app_idx);
        return -1;
    }

    keys = sub->kr_flag ? &key->keys[1] : &key->keys[0];

    if (bt_mesh_app_id(info->app_key, &keys->id))
    {
        BT_ERR("Failed to calculate AID, 0x%04x", info->app_idx);
        return -1;
    }

    key->net_idx = info->net_idx;
    key->app_idx = info->app_idx;
    memcpy(keys->val, info->app_key, 16);

    /* Binds AppKey with all non-config models, adds group address to all these models */
    comp = bt_mesh_comp_get();
    if (!comp)
    {
        BT_ERR("Invalid composition data");
        return -1;
    }

    for (i = 0; i < comp->elem_count; i++)
    {
        elem = &comp->elem[i];
        for (j = 0; j < elem->model_count; j++)
        {
            model = &elem->models[j];
            if (model->id == BLE_MESH_MODEL_ID_CFG_SRV ||
                model->id == BLE_MESH_MODEL_ID_CFG_CLI)
            {
                continue;
            }
            for (k = 0; k < ARRAY_SIZE(model->keys); k++)
            {
                if (model->keys[k] == BLE_MESH_KEY_UNUSED)
                {
                    model->keys[k] = info->app_idx;
                    break;
                }
            }
            for (k = 0; k < ARRAY_SIZE(model->groups); k++)
            {
                if (model->groups[k] == BLE_MESH_ADDR_UNASSIGNED)
                {
                    model->groups[k] = info->group_addr;
                    break;
                }
            }
        }
        for (j = 0; j < elem->vnd_model_count; j++)
        {
            model = &elem->vnd_models[j];
            for (k = 0; k < ARRAY_SIZE(model->keys); k++)
            {
                if (model->keys[k] == BLE_MESH_KEY_UNUSED)
                {
                    model->keys[k] = info->app_idx;
                    break;
                }
            }
            for (k = 0; k < ARRAY_SIZE(model->groups); k++)
            {
                if (model->groups[k] == BLE_MESH_ADDR_UNASSIGNED)
                {
                    model->groups[k] = info->group_addr;
                    break;
                }
            }
        }
    }

    return 0;
}

static esp_err_t ble_mesh_init(void)
{
    ESP_LOGI(TAG, "ble_mesh_init");

    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    ESP_LOGI(TAG, "esp_ble_mesh_register_custom_model_callback");

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    static const uint16_t net_idx;
    static const uint16_t app_idx;
    static const uint16_t flags;
    static const uint16_t iv_index;

    struct bt_mesh_device_network_info info = {
        .net_key = {
            0x01,
            0x23,
            0x45,
            0x67,
            0x89,
            0xab,
            0xcd,
            0xef,
            0x01,
            0x23,
            0x45,
            0x67,
            0x89,
            0xab,
            0xcd,
            0xef,
        },
        .net_idx = net_idx,
        .flags = flags,
        .iv_index = iv_index,
        .unicast_addr = 0x0063,
        .dev_key = {
            0x01,
            0x23,
            0x45,
            0x67,
            0x89,
            0xab,
            0xcd,
            0xef,
            0x01,
            0x23,
            0x45,
            0x67,
            0x89,
            0xab,
            0xcd,
            0xef,
        },
        .app_key = {
            0x01,
            0x23,
            0x45,
            0x67,
            0x89,
            0xab,
            0xcd,
            0xef,
            0x01,
            0x23,
            0x45,
            0x67,
            0x89,
            0xab,
            0xcd,
            0xef,
        },
        .app_idx = app_idx,
        .group_addr = 0xc000};

    err = bt_mesh_device_auto_enter_network(&info);

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return err;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);

    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        is_mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        is_mqtt_connected = false;
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        is_mqtt_connected = false;
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    size_t value_length;

    int err = nvs_get_str(NVS_HANDLE, "mqtt_url", NULL, &value_length);

    char mqtt_url[100];
    char mqtt_username[32];
    char mqtt_password[32];

    if (err == ESP_OK)
    {
        err = nvs_get_str(NVS_HANDLE, "mqtt_url", mqtt_url, &value_length);
        mqtt_url[value_length] = '/0';

        err = nvs_get_str(NVS_HANDLE, "mqtt_username", mqtt_username, &value_length);
        mqtt_username[value_length] = '/0';

        err = nvs_get_str(NVS_HANDLE, "mqtt_password", mqtt_password, &value_length);
        mqtt_password[value_length] = '/0';
    }
    else 
    {
        ESP_LOGI(TAG, "No MQTT credentials present");
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqtt_url,
        .credentials.username = mqtt_username,
        .credentials.authentication.password = mqtt_password
    };

    _mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(_mqtt_client);
}

/****************************
 * WEB SERVER
 ****************************/

/* Max length a file path can have on storage */
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

/* Max size of an individual file. Make sure this
 * value is same as that set in upload_script.html */
#define MAX_FILE_SIZE (200 * 1024) // 200 KB
#define MAX_FILE_SIZE_STR "200KB"

/* Scratch buffer size */
#define SCRATCH_BUFSIZE 8192

#define IS_FILE_EXT(filename, ext) (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

#define MAXIMUM_RETRY 10

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_EVENT BIT0
#define WIFI_FAIL_BIT BIT1

#define PROV_QR_VERSION "v1"
#define PROV_TRANSPORT_SOFTAP "softap"
#define QRCODE_BASE_URL "https://espressif.github.io/esp-jumpstart/qrcode.html"

static int s_retry_num = 0;

struct file_server_data
{
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};

static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".html"))
    {
        return httpd_resp_set_type(req, "text/html");
    }
    else if (IS_FILE_EXT(filename, ".ico"))
    {
        return httpd_resp_set_type(req, "image/x-icon");
    }

    return httpd_resp_set_type(req, "text/plain");
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static const char *get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest)
    {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash)
    {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize)
    {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serve root");

    char *filename = "/spiffs/index.html";

    FILE *fd = fopen(filename, "r");

    if (!fd)
    {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filename);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Serve root");

    // ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    httpd_resp_set_type(req, "text/html");

    httpd_resp_set_hdr(req, "Cache-Control", "max-age=604800");

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do
    {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

        if (chunksize > 0)
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK)
            {
                fclose(fd);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);

    ESP_LOGI(TAG, "File sending complete");

    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

static esp_err_t download_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path, req->uri, sizeof(filepath));

    if (!filename)
    {
        ESP_LOGE(TAG, "Filename is too long");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == -1)
    {
        // If the user has requested the /, return the root page.
        //
        if (strcmp(filename, "/") == 0) {
             return root_get_handler(req);
        }

        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd)
    {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);

    httpd_resp_set_hdr(req, "Cache-Control", "max-age=604800");

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do
    {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

        if (chunksize > 0)
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK)
            {
                fclose(fd);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);
    ESP_LOGI(TAG, "File sending complete");

    /* Respond with an empty chunk to signal HTTP response completion */
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t radiator_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Updating room name: %d", req->content_len);

    // Get the sensor number from the URL
    //
    size_t size = strlen(req->uri);

    ESP_LOGI(TAG, "URL: %d", size);
    ESP_LOGI(TAG, "URL: %s", req->uri);

    char *pch = strrchr(req->uri, '/');
    int index_of_last_slash = pch - req->uri + 1;

    ESP_LOGI(TAG, "Last occurence of '/' found at %d", index_of_last_slash);

    int length_of_number = size - index_of_last_slash;

    ESP_LOGI(TAG, "Length of number: %d", length_of_number);

    char number[length_of_number + 1];

    memcpy(number, &req->uri[index_of_last_slash], length_of_number);

    number[length_of_number] = '\0';

    // Get the name from the request
    //
    char *content = malloc(req->content_len + 1);

    httpd_req_recv(req, content, req->content_len);

    content[req->content_len] = '\0';

    ESP_LOGI(TAG, "Room Name %s", content);

    // Create and save the key
    //
    char handle[15];
    sprintf(handle, "room_name_%s", number);

    nvs_set_str(NVS_HANDLE, &handle, content);
    nvs_commit(NVS_HANDLE);

    free(content);

    httpd_resp_set_status(req, HTTPD_204);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t radiator_post_swap_handler(httpd_req_t *req)
{
    // Get the sensor number from the URL
    //
    size_t size = strlen(req->uri);

    char *pch = strrchr(req->uri, '/');
    int index_of_last_slash = pch - req->uri + 1;

    int length_of_number = size - index_of_last_slash;

    char number[length_of_number + 1];

    memcpy(number, &req->uri[index_of_last_slash], length_of_number);

    number[length_of_number] = '\0';

    ESP_LOGI(TAG, "Swapping flow and return for  %s", number);

    // Create and save the key
    //
    char handle[15];
    sprintf(handle, "swap_sensors_%s", number);

    int8_t is_swapped;

    // Read the existing value if present.
    //
    nvs_get_i8(NVS_HANDLE, &handle, &is_swapped);

    // Flip the value.
    //
    is_swapped = is_swapped == 0 ? 1 : 0;

    nvs_set_i8(NVS_HANDLE, &handle, is_swapped);
    nvs_commit(NVS_HANDLE);

    ESP_LOGI(TAG, "Sensor %s: is_swapped: %d", number, is_swapped);

    httpd_resp_set_status(req, HTTPD_204);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t radiators_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateArray();

    for (int i = 0; i < radiator_count; i++)
    {
        Radiator rad = radiators[i];
        cJSON *jRadiator = cJSON_CreateObject();

        cJSON_AddNumberToObject(jRadiator, "number", rad.number);

        char *room_name = malloc(128);
        int err = get_room_name(rad.number, room_name);

        if (err == ESP_OK)
        {
            cJSON_AddStringToObject(jRadiator, "room_name", room_name);
        }

        free(room_name);

        cJSON_AddNumberToObject(jRadiator, "flow_temperature", rad.flow_temperature);
        cJSON_AddNumberToObject(jRadiator, "return_temperature", rad.return_temperature);
        cJSON_AddNumberToObject(jRadiator, "mean_temperature", (rad.flow_temperature + rad.return_temperature) / 2);
        cJSON_AddNumberToObject(jRadiator, "last_seen", rad.last_seen);

        if (is_balancing)
        {
            cJSON_AddBoolToObject(jRadiator, "is_calibrated", rad.is_calibrated);
            cJSON_AddBoolToObject(jRadiator, "is_warming", rad.is_warming);
            cJSON_AddNumberToObject(jRadiator, "warming_order", rad.warming_order);

            float delta_t = fabs(rad.flow_temperature - rad.return_temperature);
            cJSON_AddNumberToObject(jRadiator, "delta_t", delta_t);
        }

        cJSON_AddItemToArray(root, jRadiator);
    }

    const char *json = cJSON_Print(root);
    httpd_resp_sendstr(req, json);
    free((void *)json);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t balancing_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "phase", balancing_phase);

    const char *json = cJSON_Print(root);
    httpd_resp_sendstr(req, json);
    free((void *)json);
    cJSON_Delete(root);

    return ESP_OK;
}

static esp_err_t balancing_start_post_handler(httpd_req_t *req)
{
    // Reset everything
    //
    for (int i = 0; i < radiator_count; i++)
    {
        radiators[i].is_calibrated = false;
        radiators[i].is_warming = false;
        radiators[i].warming_order = 99;
        radiators[i].calibrated_flow_temperature = 0;
        radiators[i].calibrated_return_temperature = 0;
    }

    ordering_count = 0;
    calibrated_radiator_count = 0;

    is_balancing = true;
    balancing_phase = BALANCING_PHASE_CALIBRATING;

    httpd_resp_set_status(req, HTTPD_204);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t balancing_stop_post_handler(httpd_req_t *req)
{
    is_balancing = false;
    balancing_phase = BALANCING_PHASE_NONE;

    httpd_resp_set_status(req, HTTPD_204);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

#define NGX_UNESCAPE_URI          (1)
#define NGX_UNESCAPE_REDIRECT     (2)

void ngx_unescape_uri(u_char **dst, u_char **src, size_t size, unsigned int type)
{
    u_char  *d, *s, ch, c, decoded;
    enum {
        sw_usual = 0,
        sw_quoted,
        sw_quoted_second
    } state;

    d = *dst;
    s = *src;

    state = 0;
    decoded = 0;

    while (size--) {

        ch = *s++;

        switch (state) {
        case sw_usual:
            if (ch == '?'
                    && (type & (NGX_UNESCAPE_URI | NGX_UNESCAPE_REDIRECT))) {
                *d++ = ch;
                goto done;
            }

            if (ch == '%') {
                state = sw_quoted;
                break;
            }

            *d++ = ch;
            break;

        case sw_quoted:

            if (ch >= '0' && ch <= '9') {
                decoded = (u_char) (ch - '0');
                state = sw_quoted_second;
                break;
            }

            c = (u_char) (ch | 0x20);
            if (c >= 'a' && c <= 'f') {
                decoded = (u_char) (c - 'a' + 10);
                state = sw_quoted_second;
                break;
            }

            /* the invalid quoted character */

            state = sw_usual;

            *d++ = ch;

            break;

        case sw_quoted_second:

            state = sw_usual;

            if (ch >= '0' && ch <= '9') {
                ch = (u_char) ((decoded << 4) + (ch - '0'));

                if (type & NGX_UNESCAPE_REDIRECT) {
                    if (ch > '%' && ch < 0x7f) {
                        *d++ = ch;
                        break;
                    }

                    *d++ = '%'; *d++ = *(s - 2); *d++ = *(s - 1);

                    break;
                }

                *d++ = ch;

                break;
            }

            c = (u_char) (ch | 0x20);
            if (c >= 'a' && c <= 'f') {
                ch = (u_char) ((decoded << 4) + (c - 'a') + 10);

                if (type & NGX_UNESCAPE_URI) {
                    if (ch == '?') {
                        *d++ = ch;
                        goto done;
                    }

                    *d++ = ch;
                    break;
                }

                if (type & NGX_UNESCAPE_REDIRECT) {
                    if (ch == '?') {
                        *d++ = ch;
                        goto done;
                    }

                    if (ch > '%' && ch < 0x7f) {
                        *d++ = ch;
                        break;
                    }

                    *d++ = '%'; *d++ = *(s - 2); *d++ = *(s - 1);
                    break;
                }

                *d++ = ch;

                break;
            }

            /* the invalid quoted character */

            break;
        }
    }

done:

    *dst = d;
    *src = s;
}

void uri_decode(char *dest, const char *src, size_t len)
{
    if (!src || !dest) {
        return;
    }

    unsigned char *src_ptr = (unsigned char *)src;
    unsigned char *dst_ptr = (unsigned char *)dest;
    ngx_unescape_uri(&dst_ptr, &src_ptr, len, NGX_UNESCAPE_URI);
}

static esp_err_t settings_mqtt_post_handler(httpd_req_t *req) 
{
    char *content = malloc(req->content_len + 1);

    httpd_req_recv(req, content, req->content_len);

    content[req->content_len] = '\0';

    ESP_LOGI(TAG, "PAYLOAD: %s", content);

    char param[100], dec_param[100] = {0};

    if (httpd_query_key_value(content, "url", param, sizeof(param)) == ESP_OK) {
        ESP_LOGI(TAG, "Found URL query parameter => url=%s", param);
        uri_decode(dec_param, param, strnlen(param, 100));
        nvs_set_str(NVS_HANDLE, "mqtt_url", dec_param);
    }

    if (httpd_query_key_value(content, "username", param, sizeof(param)) == ESP_OK) {
        ESP_LOGI(TAG, "Found URL query parameter => username=%s", param);
        uri_decode(dec_param, param, strnlen(param, 100));
        nvs_set_str(NVS_HANDLE, "mqtt_username", dec_param);
    }

    if (httpd_query_key_value(content, "password", param, sizeof(param)) == ESP_OK) {
        ESP_LOGI(TAG, "Found URL query parameter => password=%s", param);
        uri_decode(dec_param, param, strnlen(param, 100));
        nvs_set_str(NVS_HANDLE, "mqtt_password", dec_param);
    }

    nvs_commit(NVS_HANDLE);

    mqtt_app_start();

    httpd_resp_set_status(req, HTTPD_204);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

struct async_resp_arg
{
    httpd_handle_t hd;
    int fd;
};

static esp_err_t ws_echo_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    ws_socket = httpd_req_to_sockfd(req);

    ESP_LOGI(TAG, "Captured socket");

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }
    ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);

    ret = httpd_ws_send_frame(req, &ws_pkt);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
    }
    free(buf);
    return ret;
}

// HTTP Error (404) Handler - Redirects all requests to the root page
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Redirecting to root");
    return ESP_OK;
}

static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_echo_handler,
    .user_ctx = NULL,
    .is_websocket = true};

static httpd_handle_t start_webserver(void)
{
    const char *base_path = "/spiffs";

    esp_vfs_spiffs_conf_t conf = {
        .base_path = base_path,
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    static struct file_server_data *server_data = NULL;

    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, base_path, sizeof(server_data->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.max_uri_handlers = 11;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);

    httpd_uri_t file_download = {
        .uri = "/*", // Match all URIs of type /path/to/file
        .method = HTTP_GET,
        .handler = download_get_handler,
        .user_ctx = server_data // Pass server data as context
    };

    httpd_uri_t radiators_get_uri = {
        .uri = "/api/v1/radiators",
        .method = HTTP_GET,
        .handler = radiators_get_handler,
        .user_ctx = server_data};

    httpd_uri_t radiator_swap_post_uri = {
        .uri = "/api/v1/radiators/swap/*",
        .method = HTTP_POST,
        .handler = radiator_post_swap_handler,
        .user_ctx = server_data};

    httpd_uri_t radiator_post_uri = {
        .uri = "/api/v1/radiators/*",
        .method = HTTP_POST,
        .handler = radiator_post_handler,
        .user_ctx = server_data};

    httpd_uri_t balancing_get_uri = {
        .uri = "/api/v1/balancing",
        .method = HTTP_GET,
        .handler = balancing_get_handler,
        .user_ctx = server_data};

    httpd_uri_t balancing_start_post_uri = {
        .uri = "/api/v1/balancing/start",
        .method = HTTP_POST,
        .handler = balancing_start_post_handler,
        .user_ctx = server_data};

    httpd_uri_t balancing_stop_post_uri = {
        .uri = "/api/v1/balancing/stop",
        .method = HTTP_POST,
        .handler = balancing_stop_post_handler,
        .user_ctx = server_data};

    httpd_uri_t settings_mqtt_post_uri = {
        .uri = "/api/v1/settings/mqtt",
        .method = HTTP_POST,
        .handler = settings_mqtt_post_handler,
        .user_ctx = server_data};

    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = server_data};

    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registering URI handlers");

        httpd_register_uri_handler(server, &ws);
        httpd_register_uri_handler(server, &settings_mqtt_post_uri);
        httpd_register_uri_handler(server, &radiator_swap_post_uri);
        httpd_register_uri_handler(server, &radiator_post_uri);
        httpd_register_uri_handler(server, &radiators_get_uri);
        httpd_register_uri_handler(server, &balancing_get_uri);
        httpd_register_uri_handler(server, &balancing_start_post_uri);
        httpd_register_uri_handler(server, &balancing_stop_post_uri);
        httpd_register_uri_handler(server, &file_download);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static int retries;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;
        case WIFI_PROV_CRED_RECV:
        {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received Wi-Fi credentials"
                          "\n\tSSID     : %s\n\tPassword : %s",
                     (const char *)wifi_sta_cfg->ssid,
                     (const char *)wifi_sta_cfg->password);
            break;
        }
        case WIFI_PROV_CRED_FAIL:
        {
            wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                          "\n\tPlease reset to factory and retry provisioning",
                     (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
            retries++;
            if (retries >= 5)
            {
                ESP_LOGI(TAG, "Failed to connect with provisioned AP, reseting provisioned credentials");
                wifi_prov_mgr_reset_sm_state_on_failure();
                retries = 0;
            }
            break;
        }
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            retries = 0;
            break;
        case WIFI_PROV_END:
            /* De-initialize manager once provisioning is finished */
            wifi_prov_mgr_deinit();
            break;
        default:
            break;
        }
    }
    else if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
            esp_wifi_connect();
            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        /* Signal main application to continue execution */
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_EVENT);
    }
}

void wifi_init_sta()
{
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

/****************
 * PROVISIONING *
 ****************/

static void wifi_prov_print_qr(const char *name, const char *pop, const char *transport)
{
    if (!name || !transport)
    {
        ESP_LOGW(TAG, "Cannot generate QR code payload. Data missing.");
        return;
    }

    char payload[150] = {0};
    snprintf(payload, sizeof(payload), "{\"ver\":\"%s\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}", PROV_QR_VERSION, name, pop, transport);

    ESP_LOGI(TAG, "Scan this QR code from the provisioning application for Provisioning.");
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&cfg, payload);
    ESP_LOGI(TAG, "If QR code is not visible, copy paste the below URL in a browser.\n%s?data=%s", QRCODE_BASE_URL, payload);
}

static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X", ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

void start_mdns_service()
{
    //initialize mDNS service
    esp_err_t err = mdns_init();

    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }

    //set hostname
    mdns_hostname_set("radmon");

    //set default instance
    mdns_instance_name_set("RadMon Bridge");
}

static void obtain_time(void)
{
    ESP_LOGI(TAG, "Starting SNTP");

    //esp_netif_sntp_start();
    
    ESP_LOGI(TAG, "Initializing and starting SNTP");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) 
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }

    time(&now);
    localtime_r(&now, &timeinfo);

    esp_netif_sntp_deinit();
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing...");

    board_init();

    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE};

    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    bool provisioned = false;

    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    /* If device is not yet provisioned start provisioning service */
    if (!provisioned)
    {
        ESP_LOGI(TAG, "Starting provisioning");

        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));

        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;

        const char *pop = "abcd1234";
        const char *service_key = NULL;

        wifi_prov_security1_params_t *sec_params = pop;

        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)sec_params, service_name, service_key));

        wifi_prov_print_qr(service_name, pop, PROV_TRANSPORT_SOFTAP);
    }
    else
    {
        ESP_LOGI(TAG, "Provisioning");

        wifi_prov_mgr_deinit();

        wifi_init_sta();
    }

    // Wait for Wifi to be connected
    //
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);

    // Fetch the current time.
    //
    obtain_time();

    err = bluetooth_init();
    if (err)
    {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    err = ble_mesh_nvs_open(&NVS_HANDLE);
    if (err)
    {
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    err = ble_mesh_init();
    if (err)
    {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    ESP_LOGI(TAG, "BLE mesh init complete");

    // Start mDNS, so the .local address will work.
    start_mdns_service();

    // Start the web server
    server = start_webserver();

    ESP_LOGI(TAG, "Web server started");
}
