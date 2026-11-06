/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_lvgl_port.h"
#include "esp_lvgl_port_priv.h"
#include "lvgl.h"
#include "src/misc/lv_timer_private.h" /* Cyberdeck: timer period (lvgl_port_align_delay) */

static const char *TAG = "LVGL";

#define ESP_LVGL_PORT_TASK_MUX_DELAY_MS 10000

/*******************************************************************************
 * Types definitions
 *******************************************************************************/

typedef struct lvgl_port_ctx_s {
    TaskHandle_t lvgl_task;
    SemaphoreHandle_t lvgl_mux;
    SemaphoreHandle_t timer_mux;
    EventGroupHandle_t lvgl_events;
    SemaphoreHandle_t task_init_mux;
    esp_timer_handle_t tick_timer;
    bool running;
    int task_max_sleep_ms;
    int timer_period_ms;
} lvgl_port_ctx_t;

/*******************************************************************************
 * Local variables
 *******************************************************************************/
static lvgl_port_ctx_t lvgl_port_ctx;

/*******************************************************************************
 * Function definitions
 *******************************************************************************/
static void lvgl_port_task(void *arg);
static esp_err_t lvgl_port_tick_init(void);
static void lvgl_port_task_deinit(void);

/*******************************************************************************
 * Public API functions
 *******************************************************************************/

esp_err_t lvgl_port_init(const lvgl_port_cfg_t *cfg)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(cfg->task_affinity < (configNUM_CORES), ESP_ERR_INVALID_ARG, err, TAG,
                      "Bad core number for task! Maximum core number is %d", (configNUM_CORES - 1));

    memset(&lvgl_port_ctx, 0, sizeof(lvgl_port_ctx));

    /* Tick init */
    lvgl_port_ctx.timer_period_ms = cfg->timer_period_ms;
    /* Create task */
    lvgl_port_ctx.task_max_sleep_ms = cfg->task_max_sleep_ms;
    if (lvgl_port_ctx.task_max_sleep_ms == 0) {
        lvgl_port_ctx.task_max_sleep_ms = 500;
    }
    /* Timer semaphore */
    lvgl_port_ctx.timer_mux = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.timer_mux, ESP_ERR_NO_MEM, err, TAG, "Create timer mutex fail!");
    /* LVGL semaphore */
    lvgl_port_ctx.lvgl_mux = xSemaphoreCreateRecursiveMutex();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.lvgl_mux, ESP_ERR_NO_MEM, err, TAG, "Create LVGL mutex fail!");
    /* Task init semaphore */
    lvgl_port_ctx.task_init_mux = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.task_init_mux, ESP_ERR_NO_MEM, err, TAG, "Create LVGL task sem fail!");
    /* Task queue */
    lvgl_port_ctx.lvgl_events = xEventGroupCreate();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.lvgl_events, ESP_ERR_NO_MEM, err, TAG, "Create LVGL Event Group fail!");

    BaseType_t res;
    if (cfg->task_affinity < 0) {
        res = xTaskCreate(lvgl_port_task, "taskLVGL", cfg->task_stack, xTaskGetCurrentTaskHandle(), cfg->task_priority,
                          &lvgl_port_ctx.lvgl_task);
    } else {
        res = xTaskCreatePinnedToCore(lvgl_port_task, "taskLVGL", cfg->task_stack, xTaskGetCurrentTaskHandle(),
                                      cfg->task_priority, &lvgl_port_ctx.lvgl_task, cfg->task_affinity);
    }
    ESP_GOTO_ON_FALSE(res == pdPASS, ESP_FAIL, err, TAG, "Create LVGL task fail!");

    // Wait until taskLVGL starts
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) == 0) {
        ret = ESP_ERR_TIMEOUT;
    }

err:
    if (ret != ESP_OK) {
        lvgl_port_deinit();
    }

    return ret;
}

esp_err_t lvgl_port_resume(void)
{
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    if (lvgl_port_ctx.tick_timer != NULL) {
        lv_timer_enable(true);
        ret = esp_timer_start_periodic(lvgl_port_ctx.tick_timer, lvgl_port_ctx.timer_period_ms * 1000);
    }

    return ret;
}

esp_err_t lvgl_port_stop(void)
{
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    if (lvgl_port_ctx.tick_timer != NULL) {
        lv_timer_enable(false);
        ret = esp_timer_stop(lvgl_port_ctx.tick_timer);
    }

    return ret;
}

esp_err_t lvgl_port_deinit(void)
{
    /* Stop and delete timer */
    if (lvgl_port_ctx.tick_timer != NULL) {
        esp_timer_stop(lvgl_port_ctx.tick_timer);
        esp_timer_delete(lvgl_port_ctx.tick_timer);
        lvgl_port_ctx.tick_timer = NULL;
    }

    /* Stop running task */
    if (lvgl_port_ctx.running) {
        lvgl_port_ctx.running = false;
    }

    /* Wait for stop task */
    if (xSemaphoreTake(lvgl_port_ctx.task_init_mux, pdMS_TO_TICKS(ESP_LVGL_PORT_TASK_MUX_DELAY_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to stop LVGL task");
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "Stopped LVGL task");

    lvgl_port_task_deinit();

    return ESP_OK;
}

bool lvgl_port_lock(uint32_t timeout_ms)
{
    assert(lvgl_port_ctx.lvgl_mux && "lvgl_port_init must be called first");

    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_port_ctx.lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_port_unlock(void)
{
    assert(lvgl_port_ctx.lvgl_mux && "lvgl_port_init must be called first");
    xSemaphoreGiveRecursive(lvgl_port_ctx.lvgl_mux);
}

/* Cyberdeck: an LVGL timer paused while the screen is off (the shell's input loop) that a USER event resumes
 * and runs at once - a key or work from another thread wakes the UI without polling every few ms. */
static lv_timer_t *s_wake_timer;

void lvgl_port_set_wake_timer(lv_timer_t *timer)
{
    s_wake_timer = timer;
}

/* Cyberdeck: measurement - an lv_timer_handler pass longer than 50 ms is reported to the application (e.g. to log
 * the screen name). */
static void (*volatile s_pass_hook)(uint32_t ms);

void lvgl_port_set_pass_hook(void (*hook)(uint32_t ms))
{
    s_pass_hook = hook;
}

/* Cyberdeck: alignment of slow LVGL timers (lvgl_port_set_idle_align) - with the screen off, 1 s timers
 * (shell, alarms) each had their own phase, i.e. a separate CPU wake-up from light sleep. */
static volatile uint32_t s_idle_align_ms;

void lvgl_port_set_idle_align(uint32_t align_ms)
{
    s_idle_align_ms = align_ms;
}

/* Under the LVGL lock: the wait time aligned to a multiple of align_ms when all running timers are slow. */
static uint32_t lvgl_port_align_delay(uint32_t delay_ms)
{
    const uint32_t align = s_idle_align_ms;
    if (align == 0 || delay_ms == LV_NO_TIMER_READY || delay_ms == 0) {
        return delay_ms;
    }
    for (lv_timer_t *t = lv_timer_get_next(NULL); t != NULL; t = lv_timer_get_next(t)) {
        if (!lv_timer_get_paused(t) && t->period < align) {
            return delay_ms;  /* fast timer (animation, screen refresh) - unchanged */
        }
    }
    const int64_t now = esp_timer_get_time() / 1000;
    const int64_t due = now + delay_ms;
    /* +2 ms: a task woken by a tick can wake a fraction of a ms before the boundary, and the LVGL clock counts
     * whole ms */
    const int64_t at = ((due + align - 1) / align) * align + 2;
    return (uint32_t)(at - now);
}

esp_err_t lvgl_port_task_wake(lvgl_port_event_type_t event, void *param)
{
    EventBits_t bits = 0;
    if (!lvgl_port_ctx.lvgl_events) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Get unprocessed bits */
    if (xPortInIsrContext() == pdTRUE) {
        bits = xEventGroupGetBitsFromISR(lvgl_port_ctx.lvgl_events);
    } else {
        bits = xEventGroupGetBits(lvgl_port_ctx.lvgl_events);
    }

    /* Set event */
    bits |= event;

    /* Save */
    if (xPortInIsrContext() == pdTRUE) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(lvgl_port_ctx.lvgl_events, bits, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    } else {
        xEventGroupSetBits(lvgl_port_ctx.lvgl_events, bits);
    }

    return ESP_OK;
}

IRAM_ATTR bool lvgl_port_task_notify(uint32_t value)
{
    BaseType_t need_yield = pdFALSE;

    // Notify LVGL task
    if (xPortInIsrContext() == pdTRUE) {
        xTaskNotifyFromISR(lvgl_port_ctx.lvgl_task, value, eNoAction, &need_yield);
    } else {
        xTaskNotify(lvgl_port_ctx.lvgl_task, value, eNoAction);
    }

    return (need_yield == pdTRUE);
}

/*******************************************************************************
 * Private functions
 *******************************************************************************/

static void lvgl_port_task(void *arg)
{
    TaskHandle_t task_to_notify = (TaskHandle_t)arg;
    EventBits_t events          = 0;
    uint32_t task_delay_ms      = 0;
    lv_indev_t *indev           = NULL;

    /* Take the task semaphore */
    if (xSemaphoreTake(lvgl_port_ctx.task_init_mux, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take LVGL task sem");
        lvgl_port_task_deinit();
        vTaskDelete(NULL);
    }

    /* LVGL init */
    lv_init();
    /* LVGL is initialized, notify lvgl_port_init() function about it */
    xTaskNotifyGive(task_to_notify);
    /* Tick init */
    lvgl_port_tick_init();

    ESP_LOGI(TAG, "Starting LVGL task");
    lvgl_port_ctx.running = true;
    while (lvgl_port_ctx.running) {
        /* Wait for queue or timeout (sleep task) */
        TickType_t wait = (pdMS_TO_TICKS(task_delay_ms) >= 1 ? pdMS_TO_TICKS(task_delay_ms) : 1);
        events          = xEventGroupWaitBits(lvgl_port_ctx.lvgl_events, 0xFF, pdTRUE, pdFALSE, wait);

        if (lv_display_get_default() && lvgl_port_lock(0)) {
            /* Call read input devices */
            if (events & LVGL_PORT_EVENT_TOUCH) {
                xSemaphoreTake(lvgl_port_ctx.timer_mux, portMAX_DELAY);
                indev = lv_indev_get_next(NULL);
                while (indev != NULL) {
                    lv_indev_read(indev);
                    indev = lv_indev_get_next(indev);
                }
                xSemaphoreGive(lvgl_port_ctx.timer_mux);
            }

            /* Cyberdeck: USER event - the paused shell timer runs at once (lvgl_port_set_wake_timer) */
            if ((events & LVGL_PORT_EVENT_USER) && s_wake_timer != NULL) {
                lv_timer_resume(s_wake_timer);
                lv_timer_ready(s_wake_timer);
            }

            /* Handle LVGL */
            const int64_t pass_t0 = esp_timer_get_time();
            task_delay_ms = lvgl_port_align_delay(lv_timer_handler());
            const uint32_t pass_ms = (uint32_t)((esp_timer_get_time() - pass_t0) / 1000);
            void (*hook)(uint32_t) = s_pass_hook;
            if (pass_ms > 50 && hook != NULL) {
                hook(pass_ms);
            }
            lvgl_port_unlock();
        } else {
            task_delay_ms = 1; /*Keep trying*/
        }

        if (task_delay_ms == LV_NO_TIMER_READY) {
            task_delay_ms = lvgl_port_ctx.task_max_sleep_ms;
        }

        /* Minimal dealy for the task. When there is too much events, it takes time for other tasks and interrupts. */
        vTaskDelay(1);
    }

    /* Give semaphore back */
    xSemaphoreGive(lvgl_port_ctx.task_init_mux);

    /* Close task */
    vTaskDelete(NULL);
}

static void lvgl_port_task_deinit(void)
{
    if (lvgl_port_ctx.timer_mux) {
        vSemaphoreDelete(lvgl_port_ctx.timer_mux);
    }
    if (lvgl_port_ctx.lvgl_mux) {
        vSemaphoreDelete(lvgl_port_ctx.lvgl_mux);
    }
    if (lvgl_port_ctx.task_init_mux) {
        vSemaphoreDelete(lvgl_port_ctx.task_init_mux);
    }
    if (lvgl_port_ctx.lvgl_events) {
        vEventGroupDelete(lvgl_port_ctx.lvgl_events);
    }
    memset(&lvgl_port_ctx, 0, sizeof(lvgl_port_ctx));
#if LV_ENABLE_GC || !LV_MEM_CUSTOM
    /* Deinitialize LVGL */
    lv_deinit();
#endif
}

static void lvgl_port_tick_increment(void *arg)
{
    xSemaphoreTake(lvgl_port_ctx.timer_mux, portMAX_DELAY);
    /* Tell LVGL how many milliseconds have elapsed */
    lv_tick_inc(lvgl_port_ctx.timer_period_ms);
    xSemaphoreGive(lvgl_port_ctx.timer_mux);
}

/* Cyberdeck: LVGL time straight from esp_timer (lv_tick_set_cb) instead of a periodic interrupt every few ms - that
 * woke the CPU ~200 times a second even with the screen off and prevented automatic light sleep (in tickless mode
 * sleep needs >= 8 ms to the nearest timer). No periodic timer is created (tick_timer stays NULL). */
static uint32_t lvgl_port_tick_get(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static esp_err_t lvgl_port_tick_init(void)
{
    lv_tick_set_cb(lvgl_port_tick_get);
    return ESP_OK;
}

static esp_err_t __attribute__((unused)) lvgl_port_tick_init_periodic(void)
{
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_port_tick_increment,
        .name     = "LVGL tick",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&lvgl_tick_timer_args, &lvgl_port_ctx.tick_timer), TAG,
                        "Creating LVGL timer filed!");
    return esp_timer_start_periodic(lvgl_port_ctx.tick_timer, lvgl_port_ctx.timer_period_ms * 1000);
}
