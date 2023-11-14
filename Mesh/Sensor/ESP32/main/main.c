/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include <string.h>
#include "mesh.h"
#include "mesh_main.h"
#include "crypto.h"
#include "access.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"

#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"

#define TAG "SENSOR"

static nvs_handle_t NVS_HANDLE;

/***************
 * BLUETOOTH MESH
 ***************/
#define CID_ESP 0x02E5

static uint8_t dev_uuid[16] = {0xdd, 0xdd};

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

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

#define ESP_BLE_MESH_VND_COMPANY_ID 0x0059
#define ESP_BLE_MESH_VND_MODEL_ID 0x000A

#define ESP_BLE_MESH_VND_MODEL_OP_READING_SET ESP_BLE_MESH_MODEL_OP_3(0x0A, ESP_BLE_MESH_VND_COMPANY_ID)
#define ESP_BLE_MESH_VND_MODEL_OP_READING_SET_STATUS ESP_BLE_MESH_MODEL_OP_3(0x0B, ESP_BLE_MESH_VND_COMPANY_ID)

static esp_ble_mesh_model_op_t vnd_op[] = {
    {ESP_BLE_MESH_VND_MODEL_OP_READING_SET, 0, 0},
    {ESP_BLE_MESH_VND_MODEL_OP_READING_SET_STATUS, 0, 0},
    ESP_BLE_MESH_MODEL_OP_END,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_pub, 5 + 4, ROLE_NODE);

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(ESP_BLE_MESH_VND_COMPANY_ID, ESP_BLE_MESH_VND_MODEL_ID, vnd_op, &vnd_pub, NULL),
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models)};

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
    switch (event) {
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
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

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
            if (model->id == BLE_MESH_MODEL_ID_CFG_CLI)
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

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{

}

static esp_err_t ble_mesh_init(void)
{
    ESP_LOGI(TAG, "ble_mesh_init");

    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
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
        .unicast_addr = 0x0065,
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

#define MSG_SEND_TTL        3
#define MSG_TIMEOUT         0
#define MSG_ROLE            ROLE_PROVISIONER

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

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

    uint32_t opcode = ESP_BLE_MESH_VND_MODEL_OP_READING_SET;

    vnd_pub.publish_addr = 0x0063;

    uint8_t data[5] ={ 0x66, 0x01, 0x01, 0x01, 0x01};

    err = esp_ble_mesh_model_publish(&vnd_models[0], ESP_BLE_MESH_VND_MODEL_OP_READING_SET, sizeof(data), data, ROLE_NODE);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send vendor message 0x%06" PRIx32, opcode);
    }
}
