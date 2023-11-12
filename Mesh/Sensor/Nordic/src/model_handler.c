/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/sensor_types.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"
#include "radiator_cli.h"

static bool address_is_local(struct bt_mesh_model *mod, uint16_t addr)
{
	return bt_mesh_model_elem(mod)->addr == addr;
}

static void handle_reading(struct bt_mesh_radiator_cli *radiator,
						   struct bt_mesh_msg_ctx *ctx,
						   const uint8_t *msg)
{
	/* Don't print own messages. */
	if (address_is_local(radiator->model, ctx->addr)) {
		return;
	}
}

static const struct bt_mesh_radiator_cli_handlers radiator_handlers = {
	.reading = handle_reading
};

static struct bt_mesh_radiator_cli radiator = {
	.handlers = &radiator_handlers
};

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(
        1,
        BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV),
        BT_MESH_MODEL_LIST(BT_MESH_MODEL_RADIATOR_CLI(&radiator))),
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

const struct bt_mesh_comp *model_handler_init(void)
{
	return &comp;
}

void cmd_send_reading(uint16_t flow_temperature, uint16_t return_temperature) 
{
	int err = bt_mesh_radiator_cli_reading_set(&radiator, flow_temperature, return_temperature);

	if(err) {
		printk("Bluetooth radiator set failed (err %d)\n", err);
	}
}
