#include <Arduino.h>
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

static SemaphoreHandle_t ble_sem = NULL;

static bool ble_check(esp_err_t err) {
    if (err == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
        return true;
    }
    return false;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
            xSemaphoreGive(ble_sem);
            break;
        case ESP_GAP_BLE_EXT_ADV_REPORT_EVT:
            if ((param->ext_adv_report.params.data_status == ESP_BLE_GAP_EXT_ADV_DATA_COMPLETE) &&
				        (param->ext_adv_report.params.adv_data_len == sizeof(float) + 2) && (param->ext_adv_report.params.adv_data[1] == 0xFF)) { // Probably ESP-C3
                float *temp = (float*)&param->ext_adv_report.params.adv_data[2];

                for (uint8_t i = 0; i < ESP_BD_ADDR_LEN; ++i) {
                    if (i)
                        Serial.print(':');
                    Serial.printf("%02X", param->ext_adv_report.params.addr[i]);
                }
                Serial.print(" (BLE");
                if (param->ext_adv_report.params.event_type & ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY)
                    Serial.print('4');
                else
                    Serial.print('5');
                Serial.printf(") RSSI %d\t", param->ext_adv_report.params.rssi);
                if (isnan(*temp))
                    Serial.println("t ERROR!");
                else
                    Serial.printf("%0.2f C\r\n", *temp);
            }
            break;
        default:
            break;
    }
}

static void halt(const char *msg) {
    Serial.println(msg);
    Serial.flush();
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    Serial.println();

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

    {
        const esp_ble_ext_scan_params_t params = {
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
            .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK | ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
            .uncoded_cfg = {
                .scan_type = BLE_SCAN_TYPE_ACTIVE,
                .scan_interval = 40,
                .scan_window = 40
            },
            .coded_cfg = {
                .scan_type = BLE_SCAN_TYPE_ACTIVE,
                .scan_interval = 40,
                .scan_window = 40
            }
        };

        if (! ble_check(esp_ble_gap_set_ext_scan_params(&params)))
            halt("BLE scan prepare failed!");
    }
    if (! ble_check(esp_ble_gap_start_ext_scan(0, 0)))
        halt("BLE scan start failed!");
}

void loop() {}
