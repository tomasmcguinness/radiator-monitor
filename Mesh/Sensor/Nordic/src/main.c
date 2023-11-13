#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/dk_prov.h>

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

#include <zephyr/drivers/gpio.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include "radiator_cli.h"
#include "model_handler.h"

static const uint16_t net_idx;
static const uint16_t app_idx;
static const uint32_t iv_index;
static uint8_t flags;

#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define BCOEFFICIENT 3977
#define SERIESRESISTOR 10000

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	dk_leds_init();

	static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };

	static const struct bt_mesh_prov prov = {
		.uuid = dev_uuid
	};

	err = bt_mesh_init(&prov, model_handler_init());
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	settings_load();

	static const uint8_t net_key[16] = {
		0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
		0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	};

	static const uint8_t dev_key[16] = {
		0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
		0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	};

	static const uint8_t app_key[16] = {
		0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
		0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	};

	int config_number = CONFIG_SENSOR_NUMBER;

	uint16_t addr = config_number;

	err = bt_mesh_provision(net_key, net_idx, flags, iv_index, addr, dev_key);
	
	if (err == -EALREADY) {
		printk("Using stored settings\n");
	} else if (err) {
		printk("Provisioning failed (err %d)\n", err);
		return;
	} else {
		printk("Provisioning completed\n");

		// Add the AppKey
		//
		bt_mesh_app_key_add(net_idx, app_idx, app_key);
	
		// Bind the AppKey
		//
		bt_mesh_cfg_cli_app_key_add(net_idx, addr, net_idx, app_idx, app_key, NULL);

		// Bind the Radiator model to the node
		//
		bt_mesh_cfg_cli_mod_app_bind_vnd(net_idx, addr, addr, app_idx, BT_MESH_RADIATOR_CLI_VENDOR_MODEL_ID, BT_COMP_ID_LF, NULL);
	}

	printk("Mesh initialized\n");
}

int16_t buf;

struct adc_sequence sequence = {
	.buffer = &buf,
	.buffer_size = sizeof(buf),
	.calibrate = true,
};

uint16_t read(int i)
{
	int err = adc_sequence_init_dt(&adc_channels[i], &sequence);

	if (err < 0)
	{
		printk("Could initialise ADC (%d)\n", err);
		return -1;
	}
	
	err = adc_read(adc_channels[i].dev, &sequence);

	if (err < 0)
	{
		printk("Could not read (%d)\n", err);
		return -1;
	}

	int32_t val_mv = buf;

	float reading = (4096.0 / val_mv) - 1; // (4096/ADC - 1)

	reading = SERIESRESISTOR / reading;

	float steinhart;
	steinhart = reading / THERMISTORNOMINAL;		  // (R/Ro)
	steinhart = log(steinhart);						  // ln(R/Ro)
	steinhart /= BCOEFFICIENT;						  // 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart;					  // Invert
	steinhart -= 273.15;							  // Convert to Celcius	

	uint16_t value = (uint16_t)(steinhart * 10);

	return value;
}

void main(void)
{
	int err;

	printk("Initializing...\n");

	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
	{
		if (!device_is_ready(adc_channels[i].dev))
		{
			printk("ADC controller device not ready\n");
			return;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);

		if (err < 0)
		{
			printk("Could not setup channel #%d (%d)\n", i, err);
			return;
		}

		printk("Setup channel #%d (%d)\n", i, err);
	}

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	for(;;) {
		uint16_t flow_temperature = read(0);
		uint16_t return_temperature = read(1);

		cmd_send_reading(flow_temperature, return_temperature);

		k_sleep(K_SECONDS(5));
	}
}
