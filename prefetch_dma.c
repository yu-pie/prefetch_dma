#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>
#include "ts_device.h"
#include "ts_hal_dma.h"
#include "ts_fast_dma.h"
#include "console.h"
#include "mcu.h"

#include "ts_prefetch_dma.h"

#ifndef CACHE_MEM_SIZE
#define CACHE_MEM_SIZE 30 * 1024
#endif

static uint8_t cache_memory[CACHE_MEM_SIZE];
HAL_SEMA_T prefetch_config_ready;
static HAL_SEMA_T prefetch_ready;
static HAL_SEMA_T prefetch_consumed;

static void _dma_prefetch_task(void *arg)
{
    PrefetchArray_t *prefetch_array = (PrefetchArray_t *)arg;
    uint32_t prefetch_max_num = prefetch_array->num;
    uint32_t prefetch_num = 0;
    PrefetchArrayMem_t *prefetch_member = prefetch_array->array_mem;
    while (1) {
        if (prefetch_num >= prefetch_max_num) {
            prefetch_num = 0;
            prefetch_member = prefetch_array->array_mem;
            continue;
        }
        ts_tiny_printf("1111\n");
        dma0_memcpy_llp(cache_memory, (void *)prefetch_member->src, prefetch_member->size, MODE_FLASH_TO_MEM);
        ts_tiny_printf("2222\n");
        HAL_SEMA_GIVE(prefetch_ready);
        ts_tiny_printf("3333\n");
        HAL_SEMA_TAKE(prefetch_consumed, portMAX_DELAY);
        ts_tiny_printf("4444\n");
        prefetch_num++;
        prefetch_member++;
    
    }

}

HAL_RET_T init_prefetch_dma(uint8_t task_prio, uint32_t stack_size, void *prefetch_array)
{
    static uint8_t prefetch_dma_init = 0;
    if (1 == prefetch_dma_init) {
        return HAL_OK;
    }

    prefetch_config_ready = HAL_SEMA_INIT(1, 0);
    if (NULL == prefetch_config_ready) {
        ts_tiny_printf("prefetch_config_ready sema init failed!\n");
        return HAL_FAIL;
    }

    prefetch_ready = HAL_SEMA_INIT(1, 0);
    if (NULL == prefetch_ready) {
        ts_tiny_printf("prefetch_ready sema init failed!\n");
        return HAL_FAIL;
    }

    prefetch_consumed = HAL_SEMA_INIT(1, 0);
    if (NULL == prefetch_consumed) {
        ts_tiny_printf("prefetch_consumed sema init failed!\n");
        return HAL_FAIL;
    }

    BaseType_t xTask;
    HAL_SANITY_CHECK(task_prio < CONFIG_MAX_PRIORITIES);
    xTask = xTaskCreate(_dma_prefetch_task, "_dma_prefetch_task", stack_size, prefetch_array, task_prio, NULL);
    if (xTask == pdPASS) {
        prefetch_dma_init = 1;
        return HAL_OK;
    } else {
        ts_tiny_printf("_dma_prefetch_task create failed!\n");
        return HAL_FAIL;
    }

}

void prefetch_dma(void *prefetch_array)
{
    PrefetchArray_t *array = (PrefetchArray_t *)prefetch_array;
    uint32_t prefetch_max = array->num;
    static uint32_t num = 0;
    if (num >= prefetch_max) {
        num = 0;
    }
    ts_tiny_printf("1\n");
    HAL_SEMA_TAKE(prefetch_ready, portMAX_DELAY);
    ts_tiny_printf("2\n");
    dma0_memcpy((void *)((PrefetchArrayMem_t *)(array->array_mem))[num].dest, cache_memory, 
                    ((PrefetchArrayMem_t *)(array->array_mem))[num].size, MODE_MEM_TO_MEM);
    ts_tiny_printf("3\n");
    HAL_SEMA_GIVE(prefetch_consumed);
    HAL_SEMA_TAKE(prefetch_config_ready, portMAX_DELAY);
    num++;
    
}

