#ifndef BT_MESH_RADIATOR_CLI_H__
#define BT_MESH_RADIATOR_CLI_H__

#include <zephyr/bluetooth/mesh.h>
#include <bluetooth/mesh/model_types.h>

#define BT_MESH_RADIATOR_CLI_VENDOR_COMPANY_ID    0x0059
#define BT_MESH_RADIATOR_CLI_VENDOR_MODEL_ID      0x000A

#define BT_MESH_RADIATOR_CLI_OP_READING_SET BT_MESH_MODEL_OP_3(0x0A, BT_MESH_RADIATOR_CLI_VENDOR_COMPANY_ID)
#define BT_MESH_RADIATOR_CLI_OP_READING_LENGTH 9

struct bt_mesh_radiator_cli;

#define BT_MESH_MODEL_RADIATOR_CLI(_radiators)                               \
		BT_MESH_MODEL_VND_CB(BT_MESH_RADIATOR_CLI_VENDOR_COMPANY_ID,         \
			BT_MESH_RADIATOR_CLI_VENDOR_MODEL_ID,                            \
			_bt_mesh_radiator_cli_op, &(_radiators)->pub,                    \
			BT_MESH_MODEL_USER_DATA(struct bt_mesh_radiator_cli, _radiators),\
			&_bt_mesh_radiator_cli_cb)

struct bt_mesh_radiator_cli {
	struct bt_mesh_model *model;
	struct bt_mesh_model_pub pub;
	struct net_buf_simple pub_msg;
	uint8_t buf[BT_MESH_MODEL_BUF_LEN(BT_MESH_RADIATOR_CLI_OP_READING_SET, BT_MESH_RADIATOR_CLI_OP_READING_LENGTH)];
	const struct bt_mesh_radiator_cli_handlers *handlers;
};

struct bt_mesh_radiator_cli_handlers {
    void (*const reading)(struct bt_mesh_radiator_cli *chat,
			              struct bt_mesh_msg_ctx *ctx,
			              const uint8_t *msg);
};

int bt_mesh_radiator_cli_reading_set(struct bt_mesh_radiator_cli *radiator, uint16_t flow_temperature, uint16_t return_temperature);

extern const struct bt_mesh_model_op _bt_mesh_radiator_cli_op[];
extern const struct bt_mesh_model_cb _bt_mesh_radiator_cli_cb;

#endif