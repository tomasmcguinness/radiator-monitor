#include <zephyr/bluetooth/mesh.h>
#include "radiator_cli.h"
#include "mesh/net.h"
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(radiator);

static int handle_reading(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	return 0;
}

static int bt_mesh_radiator_cli_update_handler(struct bt_mesh_model *model)
{
	return 0;
}

static int bt_mesh_radiator_cli_init(struct bt_mesh_model *model)
{
    printk("bt_mesh_radiator_cli_init()\n");

	struct bt_mesh_radiator_cli *radiator = model->user_data;

	radiator->model = model;

	net_buf_simple_init_with_data(&radiator->pub_msg, radiator->buf, sizeof(radiator->buf));
	radiator->pub.msg = &radiator->pub_msg;
	radiator->pub.update = bt_mesh_radiator_cli_update_handler;

	return 0;
}

static int bt_mesh_radiator_cli_start(struct bt_mesh_model *model)
{
	return 0;
}

const struct bt_mesh_model_op _bt_mesh_radiator_cli_op[] = {
    { BT_MESH_RADIATOR_CLI_OP_READING_SET, 9, handle_reading },
	BT_MESH_MODEL_OP_END,
};

const struct bt_mesh_model_cb _bt_mesh_radiator_cli_cb = {
    .init = bt_mesh_radiator_cli_init,
	.start = bt_mesh_radiator_cli_start,
};

int bt_mesh_radiator_cli_reading_set(struct bt_mesh_radiator_cli *radiator, uint16_t flow_temperature, uint16_t return_temperature)
{
    printk("bt_mesh_radiator_cli_reading_set()\n");

    struct net_buf_simple *buf = radiator->model->pub->msg;

	bt_mesh_model_msg_init(buf, BT_MESH_RADIATOR_CLI_OP_READING_SET);

	int config_sensor_number = CONFIG_SENSOR_NUMBER;
	uint8_t number = config_sensor_number;

	net_buf_simple_add_u8(buf, number);
	net_buf_simple_add_le16(buf, flow_temperature);
	net_buf_simple_add_le16(buf, return_temperature);

    printk("Length to publish: %d\n", radiator->model->pub->msg->len);

    // Broadcast to everyone!
    //radiator->model->pub->addr = 0xFFFF;

	// Target the Bridge node (number 99)
	radiator->model->pub->addr = 0x0063;
    radiator->model->pub->ttl = 3;
		
    int err = bt_mesh_model_publish(radiator->model);

    if(err) {
        printk("Failed to publish: %d\n", err);
    }

    return err;
}