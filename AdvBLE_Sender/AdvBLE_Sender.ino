#include <Arduino.h>
#include "esp_sleep.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "driver/temp_sensor.h"

#define LED_PIN   2
#define LED_LEVEL LOW

const uint32_t SLEEP_TIME = 30; // 30 sec.

static SemaphoreHandle_t ble_sem = NULL;

const esp_ble_gap_ext_adv_params_t legacy_adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_NONCONN,
    .interval_min = 0x45,
    .interval_max = 0x45,
    .channel_map = ADV_CHNL_ALL,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,
    .sid = 0,
    .scan_req_notif = false
};

const esp_ble_gap_ext_adv_params_t ext_adv_params_coded = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
    .interval_min = 0x50,
    .interval_max = 0x50,
    .channel_map = ADV_CHNL_ALL,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_CODED,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_CODED,
    .sid = 1,
    .scan_req_notif = false
};

const esp_ble_gap_ext_adv_t ext_adv[2] = {
    // instance, duration, peroid
    [0] = { 0, 0, 10 },
    [1] = { 1, 0, 10 }
};

static bool ble_check(esp_err_t err) {
    if (err == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
        return true;
    }
    return false;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    static uint8_t instances = 0;

    switch (event) {
        case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
            xSemaphoreGive(ble_sem);
            break;
/*
        case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
//            xSemaphoreGive(ble_sem);
            break;
*/
        case ESP_GAP_BLE_ADV_TERMINATED_EVT:
            instances |= (1 << param->adv_terminate.adv_instance);
            if (instances == 0x03)
                xSemaphoreGive(ble_sem);
            break;
        default:
            break;
    }
}

static inline void error(const char *msg) {
    Serial.println(msg);
}

static void halt(const char *msg) {
    error(msg);
    Serial.flush();
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    Serial.println();

#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_LEVEL);
#endif

    {
        const temp_sensor_config_t tsens = TSENS_CONFIG_DEFAULT();

        if (temp_sensor_set_config(tsens) != ESP_OK)
            halt("TSENS configuration failed!");
    }
    if (temp_sensor_start() != ESP_OK)
        halt("TSENS start failed!");

    {
        const esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        if (esp_bt_controller_init((esp_bt_controller_config_t*)&bt_cfg) != ESP_OK)
            halt("BLE initialize controller failed!");
    }
    if (esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK)
        halt("BLE enable controller failed!");
    if (esp_bluedroid_init() != ESP_OK)
        halt("Init bluetooth failed!");
    if (esp_bluedroid_enable() != ESP_OK)
        halt("Enable bluetooth failed!");
    if (esp_ble_gap_register_callback(gap_event_handler) != ESP_OK)
        halt("GAP callback register error!");

    ble_sem = xSemaphoreCreateBinary();

    // 1M phy legacy adv
    if (! ble_check(esp_ble_gap_ext_adv_set_params(0, &legacy_adv_params)))
        halt("BLE legacy adv prepare failed!");

    // coded phy extend adv
    if (! ble_check(esp_ble_gap_ext_adv_set_params(1, &ext_adv_params_coded)))
        halt("BLE coded adv prepare failed!");

    {
        uint8_t adv_data[sizeof(float) + 2] = { sizeof(float) + 1, 0xFF };

        if (temp_sensor_read_celsius((float*)&adv_data[2]) != ESP_OK)
            *((float*)&adv_data[2]) = NAN;
        if ((! ble_check(esp_ble_gap_config_ext_adv_data_raw(0, sizeof(adv_data), adv_data))) ||
            (! ble_check(esp_ble_gap_config_ext_adv_data_raw(1, sizeof(adv_data), adv_data))))
            halt("BLE adv data failed!");
    }

    // start all adv
    if (! ble_check(esp_ble_gap_ext_adv_start(2, &ext_adv[0])))
        halt("BLE adv start failed!");

    // stop all adv
    {
        const uint8_t insts[] = { 0, 1 };

        if (! ble_check(esp_ble_gap_ext_adv_stop(2, insts)))
            error("BLE adv stop failed!");
    }

    if (esp_bluedroid_disable() != ESP_OK)
        error("Disable bluetooth failed!");
/*
    if (esp_bluedroid_deinit() != ESP_OK)
        error("Deinit bluetooth failed!");
*/
    if (esp_bt_controller_disable() != ESP_OK)
        error("BLE disable controller failed!");
/*
    if (esp_bt_controller_deinit() != ESP_OK)
        error("BLE deinitialize controller failed!");
*/

    Serial.printf("Sleeping for %d sec...\r\n", SLEEP_TIME);
    Serial.flush();
    esp_deep_sleep(SLEEP_TIME * 1000000);
}

void loop() {}
