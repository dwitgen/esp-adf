/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/event_groups.h"
 #include "audio_mem.h"
 #include "string.h"
 #include "adc_button.h"
 #include "esp_log.h"
 #include "audio_thread.h"
 #include "audio_idf_version.h"
 #include <esp_adc/adc_oneshot.h> 
 #include <esp_adc/adc_cali.h> 
 #include <esp_adc/adc_cali_scheme.h> 
 
 #if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
 #define ADC_ATTEN_11db ADC_ATTEN_DB_11
 #define ADC_WIDTH_12Bit ADC_BITWIDTH_12
 #define ADC_WIDTH_13Bit ADC_BITWIDTH_13
 #endif
 
 #define V_REF                           1100
 
 #define ADC_SAMPLES_NUM                 10
 #define ADC_SAMPLE_INTERVAL_TIME_MS     20
 #define DIAL_VOL_INTERVAL_TIME_MS       150
 
 #define ADC_BTN_INVALID_ID              -1
 #define ADC_BTN_INVALID_ACT_ID          -2
 #define ADC_BTN_DETECT_TIME_MS          20
 #define ADC_BTN_DETECTED_CNT            2
 
 #ifndef ENABLE_ADC_VOLUME
 #define USER_KEY_MAX                    7
 #endif
 
 static char *TAG = "ADC_BTN";
 static EventGroupHandle_t g_event_bit;

 static adc_oneshot_unit_handle_t adc_handle;
 static adc_cali_handle_t cali_handle;

 esp_err_t adc_init(adc_unit_t unit, adc_channel_t channel) {
    ESP_LOGE(TAG, "Initializing adc unit: %d on channel: %d", unit, channel);
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t ch_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &ch_config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));

    return ESP_OK;
}

int adc_read(adc_channel_t channel) {
    ESP_LOGE(TAG, "Reading adc on channel: %d", channel);
    int raw = 0, voltage = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage));
    return voltage;
}

esp_err_t adc_deinit(void) {
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
    ESP_ERROR_CHECK(adc_cali_delete_scheme(cali_handle));
    return ESP_OK;
}
 
 typedef struct {
     adc_button_callback btn_callback;
     adc_btn_list *head;
     void *user_data;
     audio_thread_t audio_thread;
 } adc_btn_tag_t;
 
 static const int default_step_level[USER_KEY_MAX] = {0, 683, 1193, 1631, 2090, 2578, 3103};
 static const int DESTROY_BIT = BIT0;
 static bool _task_flag;
 
 adc_btn_list *adc_btn_create_list(adc_arr_t *adc_conf, int channels) {
    adc_btn_list *head = NULL, *node = NULL, *find = NULL;
    for (int i = 0; i < channels; i++) {
        node = (adc_btn_list *)audio_calloc(1, sizeof(adc_btn_list));
        if (!node) return NULL;

        memcpy(&node->adc_info, &adc_conf[i], sizeof(adc_arr_t));
        node->next = NULL;

        if (!head) {
            head = node;
            find = head;
        } else {
            find->next = node;
            find = node;
        }
    }
    return head;
}
esp_err_t adc_btn_destroy_list(adc_btn_list *head) {
    adc_btn_list *current = head, *temp;
    ESP_LOGE(TAG, "Destroying adc button list");
    while (current) {
        temp = current->next;
        audio_free(current);
        current = temp;
    }
    return ESP_OK;
}

int get_adc_voltage(int channel) {
    ESP_LOGE(TAG, "Getting adc voltage on channel: %d", channel);
    return adc_read(channel);
}
 static int get_button_id(adc_btn_list *node, int adc)
 {
     int m = ADC_BTN_INVALID_ID;
     adc_arr_t *info = &(node->adc_info);
     for (int i = 0; i < info->total_steps; i++) {
         ESP_LOGV(TAG, "max:%d, adc:%d, i:%d, %d, %d", info->total_steps, adc, i, info->adc_level_step[i], info->adc_level_step[i + 1]);
         if ((adc > info->adc_level_step[i])
             && (adc <= info->adc_level_step[i + 1])) {
             m = i;
             break;
         }
     }
     return m;
 }
 
 static void reset_btn(btn_decription *btn_dscp, int btn_num)
 {
     memset(btn_dscp, 0, sizeof(btn_decription) * btn_num);
     for (int i = 0; i < btn_num; ++i) {
         btn_dscp[i].active_id = ADC_BTN_INVALID_ID;
     }
 }
 
 static adc_btn_state_t get_adc_btn_state(int adc_value, int act_id, adc_btn_list *node)
 {
     adc_btn_state_t st = ADC_BTN_STATE_IDLE;
     adc_arr_t *info = &(node->adc_info);
     btn_decription *btn_dscp = node->btn_dscp;
     int id = get_button_id(node, adc_value);
     if (id == ADC_BTN_INVALID_ID) {
         if (act_id == ADC_BTN_INVALID_ACT_ID) {
             // No old act id and new act id.
             return ADC_BTN_STATE_IDLE;
         }
         if (btn_dscp[act_id].click_cnt <= 1) {
             return ADC_BTN_STATE_IDLE;
         }
         // Have old act ID, new id is invalid
         // Need to send release event
         if (btn_dscp[act_id].click_cnt < (info->press_judge_time / ADC_BTN_DETECT_TIME_MS)) {
             ESP_LOGD(TAG, "pressed: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
             st = ADC_BTN_STATE_RELEASE;
         } else {
             ESP_LOGD(TAG, "long press release: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
             st = ADC_BTN_STATE_LONG_RELEASE;
         }
         btn_dscp[act_id].active_id = -1;
         btn_dscp[act_id].long_click = 0;
         btn_dscp[act_id].click_cnt = 0;
         return st;
     }
     // 1.ID is valid and act ID is invalid.
     if (act_id == ADC_BTN_INVALID_ACT_ID) {
         // First new act id
         btn_dscp[id].active_id = id;
         return ADC_BTN_STATE_IDLE;
     }
     // 2.ID and act ID are valid, but not equal.
     if (id != act_id) {
         ESP_LOGW(TAG, "Old ID:%d, New ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
         // Invalid the act ID
         btn_dscp[act_id].active_id = -1;
         btn_dscp[act_id].long_click = 0;
         // Set the new id act ID
         btn_dscp[id].active_id = id;
         // Maybe need to check release long pressed.
         if (btn_dscp[act_id].click_cnt < ADC_BTN_DETECTED_CNT) {
             btn_dscp[act_id].click_cnt = 0;
             return ADC_BTN_STATE_IDLE;
         }
         btn_dscp[act_id].click_cnt = 0;
         // Have old act ID, new id is invalid
         // Need to send release event
         if (btn_dscp[act_id].click_cnt < (info->press_judge_time / ADC_BTN_DETECT_TIME_MS)) {
             ESP_LOGD(TAG, "pressed: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
             return ADC_BTN_STATE_RELEASE;
         } else {
             ESP_LOGD(TAG, "long press release: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
             return ADC_BTN_STATE_LONG_RELEASE;
         }
     }
     // 3.ID and act ID are valid, and equal.
     btn_dscp[act_id].click_cnt++;
     if (btn_dscp[act_id].click_cnt == ADC_BTN_DETECTED_CNT) {
         return ADC_BTN_STATE_PRESSED;
     }
 
     if (btn_dscp[act_id].long_click) {
         return ADC_BTN_STATE_IDLE;
     }
     if (btn_dscp[act_id].click_cnt >= (info->press_judge_time / ADC_BTN_DETECT_TIME_MS)) {
         //Send long click event.
         ESP_LOGD(TAG, "long press: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
         st = ADC_BTN_STATE_LONG_PRESSED;
         btn_dscp[act_id].long_click = 1;
     }
     return st;
 }
 
 static void button_task(void *parameters) {
    ESP_LOGE(TAG, "Button Task Started");
    ESP_LOGE(TAG, "Button Task running on core: %d", xPortGetCoreID());
    
    adc_btn_list *node = (adc_btn_list *)parameters;
    ESP_LOGE(TAG, "Button Task: Node=%p, ADC Channel Before Loop=%d", node, node->adc_info.adc_ch);
    ESP_LOGE(TAG, "Button Task: Node=%p, tag->head=%p", node, ((adc_btn_tag_t *)parameters)->head);
    
    ESP_LOGE(TAG, "Button Task: Node: %p", node);
    while (1) {
        ESP_LOGE(TAG, "Button Task Loop Running");

        if (node == NULL) {
            ESP_LOGW(TAG, "Button Task: Node is NULL");
        } else {
            adc_btn_list *current_node = node;
            while (current_node) {
                ESP_LOGE(TAG, "Button Task running on core: %d", xPortGetCoreID());
                ESP_LOGE(TAG, "Reading ADC on channel: %d", current_node->adc_info.adc_ch);
                int voltage = adc_read((adc_channel_t)ADC_CHANNEL_7); //current_node->adc_info.adc_ch);
                ESP_LOGE(TAG, "Channel %d Voltage: %d", current_node->adc_info.adc_ch, voltage);
                current_node = current_node->next;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));  // Add delay to avoid flooding logs
    }
}



 void adc_btn_delete_task(void)
 {
     if (_task_flag) {
         _task_flag = false;
     }
 
     if (g_event_bit) {
         xEventGroupWaitBits(g_event_bit, DESTROY_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
         vEventGroupDelete(g_event_bit);
         g_event_bit = NULL;
     }
 }
 
 void adc_btn_init(void *user_data, adc_button_callback cb, adc_btn_list *head, adc_btn_task_cfg_t *task_cfg) {
    ESP_LOGE(TAG, "ADC Button Init");
    ESP_LOGE(TAG, "ADC Button Init: Received head=%p", head);
    adc_btn_list *node = head;
    while (node) {
        ESP_ERROR_CHECK(adc_init(ADC_UNIT_1, node->adc_info.adc_ch));
        node = node->next;
    }
    ESP_LOGE(TAG, "Before malloc: Free Heap: %d bytes", esp_get_free_heap_size());
    //adc_btn_tag_t *tag = (adc_btn_tag_t *)malloc(sizeof(adc_btn_tag_t));
    adc_btn_tag_t *tag = (adc_btn_tag_t *)audio_calloc(1, sizeof(adc_btn_tag_t));

    ESP_ERROR_CHECK_WITHOUT_ABORT(tag ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGE(TAG, "After malloc: Free Heap: %d bytes", esp_get_free_heap_size());
    if (!tag) {
        ESP_LOGE(TAG, "Memory allocation failed!");
        return;
    }
    ESP_LOGE(TAG, "ADC Button Init: ADC Channel=%d, Node=%p", head->adc_info.adc_ch, head);
    ESP_LOGE(TAG, "ADC Button Init: Received head=%p", head);
    tag->user_data = user_data;
    tag->head = node; //(void *)head;
    tag->btn_callback = cb;  // ✅ Ensure callback is set
    ESP_LOGE(TAG, "ADC Button Init: Received head=%p", head);

    g_event_bit = xEventGroupCreate();

    audio_thread_create(&tag->audio_thread,
                        "button_task", button_task,
                        (void *)tag,
                        task_cfg->task_stack,
                        task_cfg->task_prio,
                        task_cfg->ext_stack,
                        task_cfg->task_core);
    
    ESP_LOGE(TAG, "Button Task running on core: %d", xPortGetCoreID());
}