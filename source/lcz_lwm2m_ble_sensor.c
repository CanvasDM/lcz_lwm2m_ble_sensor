/**
 * @file lcz_lwm2m_ble_sensor.c
 * @brief Process BLE advertisements for Laird Connectivity sensors,
 * add LwM2M object instances, and update resource instances when values change.
 *
 * Copyright (c) 2021-2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(lwm2m_ble_sensor, CONFIG_LCZ_LWM2M_BLE_SENSOR_LOG_LEVEL);

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#include <zephyr.h>
#include <init.h>
#include <bluetooth/addr.h>

#include "lwm2m_resource_ids.h"

#include "lcz_lwm2m_gateway_obj.h"
#include "lcz_lwm2m_util.h"
#include "lcz_bt_scan.h"
#include "lcz_sensor_event.h"
#include "lcz_sensor_adv_format.h"
#include "lcz_sensor_adv_match.h"

#if defined(CONFIG_LCZ_LWM2M_TEMPERATURE)
#include "lcz_lwm2m_temperature.h"
#endif

#if defined(CONFIG_LCZ_LWM2M_CURRENT)
#include "lcz_lwm2m_current.h"
#endif

#if defined(CONFIG_LCZ_LWM2M_PRESSURE)
#include "lcz_lwm2m_pressure.h"
#endif

#if defined(CONFIG_LCZ_LWM2M_BATTERY)
#include "lcz_lwm2m_battery.h"
#include "lcz_lwm2m_battery_get_level.h"
#endif

#if defined(CONFIG_LCZ_LWM2M_FILL_LEVEL)
#include "lcz_lwm2m_fill_level.h"
#endif

#if defined(CONFIG_LCZ_LWM2M_BLE_SENSOR_LED)
#include "lcz_led.h"
#include "led_config.h"
#endif

/**************************************************************************************************/
/* Local Constant, Macro and Type Definitions                                                     */
/**************************************************************************************************/
#define LIFETIME CONFIG_LCZ_LWM2M_BLE_EVENT_TIMEOUT_SECONDS
#define MAX_INSTANCES CONFIG_LCZ_LWM2M_GATEWAY_MAX_INSTANCES

struct lwm2m_ble_sensor {
	uint8_t last_record_type;
	uint16_t last_event_id;
	int product_id;
};

struct stats {
	uint32_t ads;
	uint32_t legacy_ads;
	uint32_t rsp_ads;
	uint32_t coded_ads;
	uint32_t accepted_ads;
	uint32_t indexed_ads;
	uint32_t processed_ads;
	uint32_t set_events;
	uint32_t set_errors;
	uint32_t name_updates;
};

#if defined(CONFIG_LCZ_LWM2M_BLE_SENSOR_STATS)
#define INCR_STAT(field) lbs.stats.field += 1
#else
#define INCR_STAT(field)
#endif

static struct bt_le_scan_param scan_parameters = BT_LE_SCAN_PARAM_INIT(
	BT_LE_SCAN_TYPE_ACTIVE, (BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_FILTER_DUPLICATE),
	CONFIG_LCZ_BT_SCAN_DEFAULT_INTERVAL, CONFIG_LCZ_BT_SCAN_DEFAULT_WINDOW);

/**************************************************************************************************/
/* Local Data Definitions                                                                         */
/**************************************************************************************************/

static struct {
	int scan_user_id;
	bool table_full;
	struct lwm2m_obj_agent agent;
	struct lwm2m_ble_sensor table[MAX_INSTANCES];
#if defined(CONFIG_LCZ_LWM2M_BLE_SENSOR_STATS)
	struct stats stats;
#endif
} lbs;

/**************************************************************************************************/
/* Local Function Prototypes                                                                      */
/**************************************************************************************************/
static int get_index(const bt_addr_le_t *addr, bool add);
static bool valid_index(int idx);

static void ad_handler(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		       struct net_buf_simple *ad);
static bool ad_discard(LczSensorAdEvent_t *p);
static int ad_filter(const bt_addr_le_t *addr, LczSensorAdEvent_t *p, int8_t rssi);
static void ad_process(int idx, LczSensorAdEvent_t *p, int8_t rssi);

static void name_handler(int idx, struct net_buf_simple *ad);
static int rsp_handler(const bt_addr_le_t *addr, LczSensorRsp_t *p);

static int gw_obj_removed(int idx, void *context);

#if defined(CONFIG_LCZ_LWM2M_BLE_SENSOR_LOG_LEVEL_DBG)
static const char *get_name(int idx);
#endif

/**************************************************************************************************/
/* SYS INIT                                                                                       */
/**************************************************************************************************/
static int lcz_lwm2m_ble_sensor_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	int idx;
	int r;

	for (idx = 0; idx < MAX_INSTANCES; idx++) {
		lbs.table[idx].product_id = INVALID_PRODUCT_ID;
	}

	if (!lcz_bt_scan_register(&lbs.scan_user_id, ad_handler)) {
		LOG_ERR("LWM2M sensor module failed to register with scan module");
	}

	r = lcz_bt_scan_update_parameters(lbs.scan_user_id, &scan_parameters);
	if (r < 0) {
		LOG_ERR("Unable to update scan parameters: %d", r);
	}
	r = lcz_bt_scan_start(lbs.scan_user_id);
	if (r < 0) {
		LOG_ERR("Unable to start scanning: %d", r);
	}

	lbs.agent.gw_obj_deleted = gw_obj_removed;
	lcz_lwm2m_util_register_agent(&lbs.agent);

	return 0;
}

SYS_INIT(lcz_lwm2m_ble_sensor_init, APPLICATION, LCZ_LWM2M_UTIL_USER_INIT_PRIORITY);

/**************************************************************************************************/
/* Occurs in BT RX Thread context                                                                 */
/**************************************************************************************************/
static void ad_handler(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		       struct net_buf_simple *ad)
{
	int idx;
	LczSensorAdCoded_t *coded;
	AdHandle_t handle;

	INCR_STAT(ads);
	handle = AdFind_Type(ad->data, ad->len, BT_DATA_MANUFACTURER_DATA, BT_DATA_INVALID);

	/* Only one of these types can occur at a time.
	 * The are ordered by probability of occurrence.
	 */
	if (lcz_sensor_adv_match_1m(&handle)) {
		ad_filter(addr, (LczSensorAdEvent_t *)handle.pPayload, rssi);
		INCR_STAT(legacy_ads);
		return;
	}

	if (lcz_sensor_adv_match_coded(&handle)) {
		INCR_STAT(coded_ads);
		/* The coded phy contains the TLVs of the 1M ad and scan response */
		coded = (LczSensorAdCoded_t *)handle.pPayload;
		ad_filter(addr, &coded->ad, rssi);
		idx = rsp_handler(addr, &coded->rsp);
		name_handler(idx, ad);
		return;
	}

	if (lcz_sensor_adv_match_rsp(&handle)) {
		INCR_STAT(rsp_ads);
		idx = rsp_handler(addr, &(((LczSensorRspWithHeader_t *)handle.pPayload)->rsp));
		name_handler(idx, ad);
		return;
	}
}

/**************************************************************************************************/
/* Local Function Definitions                                                                     */
/**************************************************************************************************/
static bool valid_index(int idx)
{
	return (idx >= 0 && idx < MAX_INSTANCES);
}

static int get_index(const bt_addr_le_t *addr, bool add)
{
	static char addr_str[BT_ADDR_LE_STR_LEN];
	int idx;

	/* If the device isn't in the database,
	 * the ad has been filtered, the table isn't full, and it isn't blocked;
	 * try to add it.
	 */
	idx = lcz_lwm2m_gw_obj_lookup_ble(addr);
	if (!valid_index(idx) && add && !lbs.table_full) {
		bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
		idx = lcz_lwm2m_gw_obj_create(addr);
		/* Limit logging for blocked devices */
		if (idx != -EPERM || IS_ENABLED(CONFIG_LCZ_LWM2M_BLE_CREATE_LOG_VERBOSE)) {
			LOG_DBG("Gateway object create request %s: idx: %d inst: %d name: %s",
				addr_str, idx, lcz_lwm2m_gw_obj_get_instance(idx), get_name(idx));
		}
		if (idx == -ENOMEM) {
			lbs.table_full = true;
		}
	}

	if (idx >= MAX_INSTANCES) {
		LOG_ERR("Invalid index");
		return -EPERM;
	}

	return idx;
}

static int ad_filter(const bt_addr_le_t *addr, LczSensorAdEvent_t *p, int8_t rssi)
{
	int idx = -EPERM;

	do {
		if (p == NULL) {
			break;
		}

		/* Short circuit if event type isn't supported */
		if (ad_discard(p)) {
			break;
		}

		INCR_STAT(accepted_ads);
		idx = get_index(addr, true);
		if (!valid_index(idx)) {
			break;
		}
		INCR_STAT(indexed_ads);

		/* Filter out duplicate events.
		 * If both devices have just powered-up, don't filter event 0.
		 */
		if ((p->id != 0) && (p->id == lbs.table[idx].last_event_id) &&
		    (p->recordType == lbs.table[idx].last_record_type)) {
			break;
		}

		if (IS_ENABLED(CONFIG_LCZ_LWM2M_BLE_EVENT_LOG_VERBOSE)) {
			LOG_INF("%s idx: %d RSSI: %d id: %u",
				lcz_sensor_event_get_string(p->recordType), idx, rssi, p->id);
		}

		lbs.table[idx].last_event_id = p->id;
		lbs.table[idx].last_record_type = p->recordType;

#if defined(CONFIG_LCZ_LWM2M_BLE_SENSOR_LED)
		lcz_led_blink(BLE_LED, &BLE_ACTIVITY_LED_PATTERN);
#endif
		ad_process(idx, p, rssi);

		/* An ad that we care about has been seen; prevent device from being removed from table */
		if (lcz_lwm2m_gw_obj_set_lifetime(idx, LIFETIME) != 0) {
			LOG_ERR("Unable to set lifetime");
		}
	} while (0);

	return idx;
}

/* need to filter on if objects are enabled */
static bool ad_discard(LczSensorAdEvent_t *p)
{
	bool discard = true;

	switch (p->recordType) {
	case SENSOR_EVENT_TEMPERATURE:
		if (IS_ENABLED(CONFIG_LCZ_LWM2M_TEMPERATURE)) {
			discard = false;
		}
		break;

	case SENSOR_EVENT_BATTERY_GOOD:
	case SENSOR_EVENT_BATTERY_BAD:
		if (IS_ENABLED(CONFIG_LCZ_LWM2M_BATTERY)) {
			discard = false;
		}
		break;

	case SENSOR_EVENT_TEMPERATURE_1:
	case SENSOR_EVENT_TEMPERATURE_2:
	case SENSOR_EVENT_TEMPERATURE_3:
	case SENSOR_EVENT_TEMPERATURE_4:
		if (IS_ENABLED(CONFIG_LCZ_LWM2M_TEMPERATURE)) {
			discard = false;
		}
		break;

	case SENSOR_EVENT_CURRENT_1:
	case SENSOR_EVENT_CURRENT_2:
	case SENSOR_EVENT_CURRENT_3:
	case SENSOR_EVENT_CURRENT_4:
		if (IS_ENABLED(CONFIG_LCZ_LWM2M_CURRENT)) {
			discard = false;
		}
		break;

	case SENSOR_EVENT_PRESSURE_1:
	case SENSOR_EVENT_PRESSURE_2:
		if (IS_ENABLED(CONFIG_LCZ_LWM2M_PRESSURE)) {
			discard = false;
		}
		break;

	case SENSOR_EVENT_ULTRASONIC_1:
		if (IS_ENABLED(CONFIG_LCZ_LWM2M_FILL_LEVEL)) {
			discard = false;
		}
		break;

	default:
		discard = true;
		break;
	}

	return discard;
}

static void ad_process(int idx, LczSensorAdEvent_t *p, int8_t rssi)
{
	uint16_t offset = 0;
	double data = 0.0;
	uint8_t percentage;
	int r = -EPERM;

	switch (p->recordType) {
	case SENSOR_EVENT_TEMPERATURE:
#if defined(CONFIG_LCZ_LWM2M_TEMPERATURE)
		data = (((double)((int16_t)p->data.u16)) / 100.0);
		r = lcz_lwm2m_managed_temperature_set(idx, offset, data);
#endif
		break;

	case SENSOR_EVENT_TEMPERATURE_1:
	case SENSOR_EVENT_TEMPERATURE_2:
	case SENSOR_EVENT_TEMPERATURE_3:
	case SENSOR_EVENT_TEMPERATURE_4:
#if defined(CONFIG_LCZ_LWM2M_TEMPERATURE)
		data = (double)p->data.f;
		offset = (p->recordType - SENSOR_EVENT_TEMPERATURE_1);
		r = lcz_lwm2m_managed_temperature_set(idx, offset, data);
#endif
		break;

	case SENSOR_EVENT_BATTERY_GOOD:
	case SENSOR_EVENT_BATTERY_BAD:
#if defined(CONFIG_LCZ_LWM2M_BATTERY)
		switch (lbs.table[idx].product_id) {
		case BT510_PRODUCT_ID:
			data = ((double)((uint32_t)p->data.u16)) / 1000.0;
			percentage = lcz_lwm2m_battery_get_level_bt510(data);
			break;
		case BT6XX_PRODUCT_ID:
			data = ((double)((uint32_t)p->data.s32)) / 1000.0;
			percentage = lcz_lwm2m_battery_get_level_bt610(data);
			break;
		default:
			data = 0;
			percentage = 0;
			break;
		}

		r = lcz_lwm2m_managed_battery_set(idx, offset, data, percentage);
#endif
		break;

	case SENSOR_EVENT_CURRENT_1:
	case SENSOR_EVENT_CURRENT_2:
	case SENSOR_EVENT_CURRENT_3:
	case SENSOR_EVENT_CURRENT_4:
#if defined(CONFIG_LCZ_LWM2M_CURRENT)
		data = (double)p->data.f;
		offset = (p->recordType - SENSOR_EVENT_CURRENT_1);
		r = lcz_lwm2m_managed_current_set(idx, offset, data);
#endif
		break;

	case SENSOR_EVENT_PRESSURE_1:
	case SENSOR_EVENT_PRESSURE_2:
#if defined(CONFIG_LCZ_LWM2M_PRESSURE)
		data = (double)p->data.f;
		offset = (p->recordType - SENSOR_EVENT_PRESSURE_1);
		r = lcz_lwm2m_managed_pressure_set(idx, offset, data);
#endif
		break;

	case SENSOR_EVENT_ULTRASONIC_1:
#if defined(CONFIG_LCZ_LWM2M_FILL_LEVEL)
		/* Convert from mm (reported) to cm (filling sensor) */
		data = (double)p->data.f / 10.0;
		r = lcz_lwm2m_managed_fill_level_set(idx, offset, data);
#endif
		break;

	default:
		LOG_WRN("Unhandled advertisement event");
		break;
	}

	INCR_STAT(processed_ads);
	if (r == 0) {
		INCR_STAT(set_events);
	} else {
		INCR_STAT(set_errors);
	}
}

/**
 * @brief The scan response is used to determine the sensor type.
 * Which is used to determine the battery conversion.
 */
static int rsp_handler(const bt_addr_le_t *addr, LczSensorRsp_t *p)
{
	int idx = -EPERM;

	if (p == NULL) {
		return idx;
	}

	idx = get_index(addr, false);
	if (idx >= 0) {
		lbs.table[idx].product_id = p->productId;
	}
	return idx;
}

static void name_handler(int idx, struct net_buf_simple *ad)
{
	AdHandle_t handle;
	int r;

	if (!valid_index(idx)) {
		return;
	}

	handle = AdFind_Name(ad->data, ad->len);
	if (handle.pPayload == NULL) {
		return;
	}

	/* If the LwM2M connection has been proxied or named already, then don't try to set name. */
	if (lcz_lwm2m_gw_obj_inst_created(idx)) {
		return;
	}

	r = lcz_lwm2m_gw_obj_set_endpoint_name(idx, handle.pPayload, handle.size);
	if (r == 0) {
		INCR_STAT(name_updates);
	}
	LOG_DBG("Set endpoint name in database[%d]: %d", idx, r);
}

static int gw_obj_removed(int idx, void *context)
{
	ARG_UNUSED(context);

	if (valid_index(idx)) {
		lbs.table_full = false;
		lbs.table[idx].product_id = INVALID_PRODUCT_ID;
	}

	return 0;
}

#if defined(CONFIG_LCZ_LWM2M_BLE_SENSOR_LOG_LEVEL_DBG)
static const char *get_name(int idx)
{
	static char name[SENSOR_NAME_MAX_SIZE];
	int r;

	memset(name, 0, sizeof(name));
	r = lcz_lwm2m_gw_obj_get_endpoint_name(idx, name, SENSOR_NAME_MAX_STR_LEN);
	if (r < 0) {
		return "?";
	} else {
		return name;
	}
}
#endif