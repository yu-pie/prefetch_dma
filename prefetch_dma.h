#ifndef __TS_PREFETCH_DMA_H__
#define __TS_PREFETCH_DMA_H__

#include <stdint.h>

typedef struct {
    uint32_t src;
    uint32_t dest;
    uint32_t size;
}PrefetchArrayMem_t;

typedef struct {
    uint32_t num;
    PrefetchArrayMem_t *array_mem;
}PrefetchArray_t;

/**
 * @brief Create a task to do prefetch. If you want to use prefetch_dma, init_prefetch_dma is necessary.
 * 
 * @param task_prio the priority of the prefetch task, recommend to be app task(who use prefetch_dma)'s prio -1;
 * @param stack_size task stack size, configMINIMAL_STACK_SIZE recommend;
 * @param prefetch_array PrefetchArray_t, contains the num, offset and size of dma prefetch;
 * @return HAL_RET_T HAL_OK means task creating ok or already created, HAL_FAIL means task creating failed.
 */
HAL_RET_T init_prefetch_dma(uint8_t task_prio, uint32_t stack_size, void *prefetch_array);

/**
 * @brief Do prefetch_dma.
 * 
 * @param prefetch_array PrefetchArray_t contains the num, offset and size of dma prefetch.
 */
void prefetch_dma(void *prefetch_array);

#endif