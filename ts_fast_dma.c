#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include "ts_device.h"
#include "ts_hal_dma.h"
#include "ts_fast_dma.h"
#include "console.h"
#include "mcu.h"

#if defined(CONFIG_WITH_CLUSTER)
#include "libcc.h"
#include "cc_utils.h"
#endif

/* we use CH7 to avoid extra compute */
/*dma src gather*/
#define DW_DMAC_SG_EN   0x20000
#define DW_DMAC_SGC_MSK 0xfff
#define DW_DMAC_SGI_MSK 0xfffff
/*dma dst gather*/
#define DW_DMAC_DS_EN   0x40000
#define DW_DMAC_DSC_MSK 0xfff
#define DW_DMAC_DSI_MSK 0xfffff

#define DMAC_MAX_SIZE_PER_BLOCK (2044)
#define DMAC_TIMEOUT            (2000)

#define USE_CH  DMACHN_ID0
#define DMA_REG ((DMABaseReg_t *)DMAC_BASE)

#ifdef CONFIG_AT820
#define DMA0_REG ((DMABaseReg_t *)DMA0_BASE)
#define DMA0_CHN DMACHN_ID0
static DMAHnd_t    dma0_handler = NULL;
static DMAChnHnd_t dma0_chn     = NULL;
static NORHnd_t    qspi_handler = NULL;
#endif

static DMAHnd_t          dmahandler    = NULL;
static SemaphoreHandle_t fast_dma_lock = NULL;

/* This function should be called at very early stage of system initialization to gurantee that reserved channel is not
 *occupied. */
ITCM_FUNC void init_fast_dma(void)
{
    static uint8_t fast_dma_init = 0;
    if (1 == fast_dma_init) {
        return;
    }
    DMAChnHnd_t chnhnd = NULL;

    dmahandler = sys_get_dma_hnd(SYS_IP_DMA);
    if (dmahandler == NULL) {
        dmahandler = HAL_DMA_Init(SYS_IP_DMA);
        HAL_SANITY_CHECK(dmahandler != NULL);
        if (1 == fast_dma_init) {
            fast_dma_init = 0;
        }
    }

    chnhnd = HAL_DMA_CreateChannel(dmahandler, USE_CH, 1000);
    HAL_SANITY_CHECK(chnhnd != NULL);
    fast_dma_init = 1;

    if (NULL == fast_dma_lock) {
        fast_dma_lock = xSemaphoreCreateMutex();
        if (!fast_dma_lock) {
            ts_tiny_printf("Error!Fail to create fast_dma_lock!\n");
            configASSERT(fast_dma_lock);
        }
    }

    ts_ll_dma_enable(DMA_REG);

    int32_t ctrl_value = DMA_REG->CHANNEL[USE_CH].CTL.value[0];
    /* CTL[0] reg */
    ctrl_value = (ctrl_value & (DW_DMAC_DS_EN | DW_DMAC_SG_EN)) |
                 (DMA_TRANS_TYPE_M2M << 20) |     /* control flow */
                 (DMA_TRANS_BURSTSIZE_64 << 14) | /* src burst size */
                 (DMA_TRANS_BURSTSIZE_64 << 11) | /* dest burst size */
                 (DMA_ADDR_INCREMENT << 9) |      /* src addr update */
                 (DMA_ADDR_INCREMENT << 7) |      /* dest addr update */
                 (DMA_TRANS_WIDTH_32b << 4) |     /* src trans width */
                 (DMA_TRANS_WIDTH_32b << 1);      /* dest trans width */

    DMA_REG->CHANNEL[USE_CH].CTL.value[0] = ctrl_value;

    /* CFG[0] */
    DMA_REG->CHANNEL[USE_CH].CFG.value[0] = 0 << 20 | 0 << 19 | 0 << 18 | 0 << 8 | 0 << 5;

    /* CFG[1] */
    int32_t cfg_value = DMA_REG->CHANNEL[USE_CH].CFG.value[1];
    cfg_value |= 1 << 1;
    cfg_value &= ~0x7f80;
    DMA_REG->CHANNEL[USE_CH].CFG.value[1] = cfg_value;
}

static ITCM_FUNC void init_fast_dma_with_config(DMABaseReg_t *base, int chan, DMAAddrUpdateMode_t add_update, DMAChnTransWidth_t width)
{
    ts_ll_dma_enable(base);
    int32_t ctrl_value = base->CHANNEL[chan].CTL.value[0];

    ctrl_value = (DMA_TRANS_TYPE_M2M << 20) |    /* control flow */
                 (DMA_TRANS_BURSTSIZE_1 << 14) | /* src burst size */
                 (DMA_TRANS_BURSTSIZE_1 << 11) | /* dest burst size */
                 (add_update << 9) |             /* src addr update */
                 (add_update << 7) |             /* dest addr update */
                 (width << 4) |                  /* src trans width */
                 (width << 1);                   /* dest trans width */

    base->CHANNEL[chan].CTL.value[0] = ctrl_value;

    /* CFG[0] */
    base->CHANNEL[chan].CFG.value[0] = 0 << 20 | 0 << 19 | 0 << 18 | 0 << 8 | 0 << 5;
    /* CFG[1] */
    int32_t cfg_value = base->CHANNEL[chan].CFG.value[1];
    cfg_value |= 1 << 1;
    cfg_value &= ~0x7f80;
    base->CHANNEL[chan].CFG.value[1] = cfg_value;
}

static ITCM_FUNC void fast_dma_m2m(DMABaseReg_t *base, int chn, int32_t dest, int32_t src, uint32_t size)
{
    /* set src, dest and size */
    base->CHANNEL[chn].SAR.ADDR     = src;
    base->CHANNEL[chn].DAR.ADDR     = dest;
    base->CHANNEL[chn].CTL.value[1] = size & 0xfff;

    /* start */
    ts_ll_dma_ch_enable(base, chn);

    /* wait for done */
    while (ts_ll_dma_ch_is_enable(base, chn) == 1)
        ;
}

/* Use CPU to do the memory copy */
/* this is used to copy the L2 data to L1 DTCM for small amount of data */
/* the caller should make sure that the src and dest base 4 aligned */
ITCM_FUNC void tiny_memcpy(int8_t *_dest, int8_t *_src, uint32_t size)
{
    int32_t  cnt  = size >> 2;
    int32_t  done = cnt << 2;
    int32_t *src  = (int32_t *)_src;
    int32_t *dest = (int32_t *)_dest;
    int32_t  i    = 0;
    do {
        *(dest + i) = *(src + i);
    } while (++i < cnt);

    if (done != size) {
        i = done;
        do {
            *(_dest + i) = *(_src + i);
        } while (++i < size);
    }
    return;
}

#if 0

static ITCM_FUNC int32_t is_overlapped(uint32_t src, uint32_t dest, uint32_t size)
{
    return ((src < dest) && (src+size > dest)) ||((dest < src) && (dest + size > src));
}

static ITCM_FUNC void fast_memcpy_unalign(void *dest, void *src, uint32_t size)
{
    int32_t count_per_block = 0;
    int32_t done_bytes = 0;
    int32_t count = size;

    init_fast_dma_with_config(DMA_REG, USE_CH, INCREASE, BYTE_WIDTH);

    while (count > 0) {
        count_per_block= count > DMAC_MAX_SIZE_PER_BLOCK ? DMAC_MAX_SIZE_PER_BLOCK : count;
        fast_dma_m2m((DMA_REG, USE_CH, (int32_t)(int8_t *)dest+done_bytes), (int32_t)((int8_t *)src+done_bytes), count_per_block);
        count-= count_per_block;
        done_bytes += count_per_block;
    }

    init_fast_dma();
}
#endif

#ifdef CONFIG_WITH_CLUSTER
static DmaArg_t dma_param;
static void     mcu_transferData1D_cc(void *arg)
{
    DmaArg_t *param = (DmaArg_t *)arg;
    cc_fast_memcpy(param->dst, param->src, param->len);
}
void mcu_transferData1D(void *dst, void *src, int len)
{
    dma_param.dst = dst;
    dma_param.src = src;
    dma_param.len = len;

    mcu_send_task_wait_done(mcu_transferData1D_cc, &dma_param);
}
#endif

/* unaligned and overlap situations are consider */
static ITCM_FUNC void mcu_fast_memcpy(void *dest, void *src, uint32_t size)
{
    if (src == dest)
        return;
#ifdef CONFIG_WITH_CLUSTER
    if (((is_in_CCL1((u32)dest) ^ is_in_CCL1((u32)src)) && size > 1024)) {
        mcu_transferData1D(dest, src, size);
        return;
    }
#endif
    int32_t offset;
    offset = (uint32_t)dest > (uint32_t)src ? (uint32_t)dest - (uint32_t)src : (uint32_t)src - (uint32_t)dest;

    int8_t isAlign = !((((uint32_t)dest) & 3) || (((uint32_t)src) & 3));

    if (offset < size && offset < 0x40 && isAlign) { /* memcpy is faster when the offset is too small when overlapped,
                                                      * this is the empirical value */
        memmove(dest, src, size);
        return;
    }
    xSemaphoreTake(fast_dma_lock, portMAX_DELAY);

    int32_t count_per_block = 0;
    int32_t done_bytes      = 0;

    int32_t unit = 0;

    if (src < dest) {
        done_bytes = size - 1;
        if (isAlign) {
            init_fast_dma_with_config(DMA_REG, USE_CH, DMA_ADDR_DECREMENT, DMA_TRANS_WIDTH_32b);
            unit = 2;
        } else {
            init_fast_dma_with_config(DMA_REG, USE_CH, DMA_ADDR_DECREMENT, DMA_TRANS_WIDTH_8b);
        }
    } else {
        done_bytes = 0;
        if (isAlign) {
            init_fast_dma_with_config(DMA_REG, USE_CH, DMA_ADDR_INCREMENT, DMA_TRANS_WIDTH_32b);
            unit = 2;
        } else {
            init_fast_dma_with_config(DMA_REG, USE_CH, DMA_ADDR_INCREMENT, DMA_TRANS_WIDTH_8b);
        }
    }

    int32_t count = size >> unit;
    int32_t left  = 0;
    if (unit > 0) {
        left = size % (1 << unit);
    }
    int32_t max_transfer_cnt = (offset >> unit) > DMAC_MAX_SIZE_PER_BLOCK ? DMAC_MAX_SIZE_PER_BLOCK : (offset >> unit);

    if (left && (src < dest) && isAlign) { /* do the high left bytes first if src < dest */
        for (int32_t i = 0; i < left; i++) {
            *((int8_t *)dest + done_bytes - i) = *((int8_t *)src + done_bytes - i);
        }
        done_bytes -= left;
    }

    while (count > 0) {
        count_per_block = count > max_transfer_cnt ? max_transfer_cnt : count;
        fast_dma_m2m(DMA_REG, USE_CH, (int32_t)((int8_t *)dest + done_bytes), (int32_t)((int8_t *)src + done_bytes), count_per_block);
        count -= count_per_block;
        if (src < dest) {
            done_bytes -= count_per_block << unit;
        } else {
            done_bytes += count_per_block << unit;
        }
    }

    if (left && (src > dest) && isAlign) { /* do the low left bytes at last if src > dest */
        for (int32_t i = 0; i < left; i++) {
            *((int8_t *)dest + done_bytes + i) = *((int8_t *)src + done_bytes + i);
        }
    }

    // init_fast_dma();
    xSemaphoreGive(fast_dma_lock);
}

/* this is the wrapper of fast memcpy that works on both MCU and Cluster */
void fast_memcpy(void *dest, void *src, uint32_t size)
{
#ifdef CONFIG_WITH_CLUSTER
    if (get_cluster_id() == 32) { /* mcu */
        init_fast_dma();
        mcu_fast_memcpy(dest, src, size);
    } else { /* cc */
        cc_fast_memcpy(dest, src, size);
    }
#else
    init_fast_dma();
    mcu_fast_memcpy(dest, src, size);
#endif
}

#ifdef CONFIG_AT820
static void clear_dma_status(DMABaseReg_t *base, int chan)
{
    ts_ll_dma_ch_clear_tfr_status(base, chan);
    ts_ll_dma_ch_clear_block_status(base, chan);
    ts_ll_dma_ch_clear_src_tran_status(base, chan);
    ts_ll_dma_ch_clear_dst_tran_status(base, chan);
    ts_ll_dma_ch_clear_err_status(base, chan);
}

static void init_qspi_dma_with_config(DMABaseReg_t *base, int chan, DMAChnTransWidth_t width)
{
    ts_ll_dma_enable(base);
    int32_t ctrl_value = base->CHANNEL[chan].CTL.value[0];

    ctrl_value = (DMA_TRANS_TYPE_P2M << 20) |    /* control flow */
                 (DMA_TRANS_BURSTSIZE_4 << 14) | /* src burst size */
                 (DMA_TRANS_BURSTSIZE_4 << 11) | /* dest burst size */
                 (DMA_ADDR_NOCHANGE << 9) |      /* src addr update */
                 (DMA_ADDR_INCREMENT << 7) |     /* dest addr update */
                 (width << 4) |                  /* src trans width */
                 (width << 1);                   /* dest trans width */

    base->CHANNEL[chan].CTL.value[0] = ctrl_value;

    /* CFG[0] */
    base->CHANNEL[chan].CFG.value[0] = 0 << 20 | 0 << 19 | 0 << 18 | 0 << 8 | DMA_PRIORITY4 << 5;
    /* CFG[1] */
    int32_t cfg_value = base->CHANNEL[chan].CFG.value[1];
    cfg_value &= ~0x7f80;
    cfg_value |= 1 << 1;
    cfg_value |= 1 << 7;
    base->CHANNEL[chan].CFG.value[1] = cfg_value;

    clear_dma_status(base, chan);
}

static void init_dma_fft_rx_config(DMABaseReg_t *base, int chan, DMAChnTransWidth_t width)
{
    ts_ll_dma_enable(base);
    int32_t ctrl_value = base->CHANNEL[chan].CTL.value[0];

    ctrl_value = (DMA_TRANS_TYPE_M2M << 20) |    /* control flow */
                 (DMA_TRANS_BURSTSIZE_8 << 14) | /* src burst size */
                 (DMA_TRANS_BURSTSIZE_8 << 11) | /* dest burst size */
                 (DMA_ADDR_INCREMENT << 9) |     /* src addr update */
                 (DMA_ADDR_NOCHANGE << 7) |      /* dest addr update */
                 (width << 4) |                  /* src trans width */
                 (width << 1);                   /* dest trans width */

    base->CHANNEL[chan].CTL.value[0] = ctrl_value;

    /* CFG[0] */
    base->CHANNEL[chan].CFG.value[0] = 0 << 20 | 0 << 19 | 0 << 18 | 0 << 8 | DMA_PRIORITY4 << 5;
    /* CFG[1] */
    int32_t cfg_value = base->CHANNEL[chan].CFG.value[1];
    cfg_value &= ~0x7f80;
    cfg_value |= 0 << 7;
    base->CHANNEL[chan].CFG.value[1] = cfg_value;
    ts_ll_dma_ch_set_dst_hw_handshake_interface(&base->CHANNEL[chan],
                                                LL_DMA_CHNID_TO_DSTREQID(SYS_IP_FFT, 0));

    clear_dma_status(base, chan);
}

static void init_dma_fft_tx_config(DMABaseReg_t *base, int chan, DMAChnTransWidth_t width)
{
    ts_ll_dma_enable(base);
    int32_t ctrl_value = base->CHANNEL[chan].CTL.value[0];

    ctrl_value = (DMA_TRANS_TYPE_P2M << 20) |     /* control flow */
                 (DMA_TRANS_BURSTSIZE_16 << 14) | /* src burst size */
                 (DMA_TRANS_BURSTSIZE_4 << 11) |  /* dest burst size */
                 (DMA_ADDR_INCREMENT << 9) |      /* src addr update */
                 (DMA_ADDR_INCREMENT << 7) |      /* dest addr update */
                 (width << 4) |                   /* src trans width */
                 (width << 1);                    /* dest trans width */

    base->CHANNEL[chan].CTL.value[0] = ctrl_value;

    /* CFG[0] */
    base->CHANNEL[chan].CFG.value[0] = 0 << 20 | 0 << 19 | 0 << 18 | 0 << 8 | DMA_PRIORITY4 << 5;
    /* CFG[1] */
    int32_t cfg_value = base->CHANNEL[chan].CFG.value[1];
    cfg_value &= ~0x7f80;
    base->CHANNEL[chan].CFG.value[1] = cfg_value;
    ts_ll_dma_ch_set_src_hw_handshake_interface(&base->CHANNEL[chan],
                                                LL_DMA_CHNID_TO_DSTREQID(SYS_IP_FFT, 0));

    clear_dma_status(base, chan);
}

static void dma0_memcpy_init(void)
{
    if (dma0_handler != NULL)
        return;

    dma0_handler = sys_get_dma_hnd(SYS_IP_DMA0);
    if (dma0_handler == NULL) {
        dma0_handler = HAL_DMA_Init(SYS_IP_DMA0);
        HAL_SANITY_CHECK(dma0_handler != NULL);
    }

    if (dma0_chn == NULL) {
        dma0_chn = HAL_DMA_CreateChannel(dma0_handler, DMA0_CHN, 1000);
        HAL_SANITY_CHECK(dma0_chn != NULL);
    }

    if (qspi_handler == NULL) {
        NORConf_t norconf = NOR_DEFAULT_CONF();
        qspi_handler      = HAL_NOR_Init(SYS_IP_QSPI);
        HAL_SANITY_CHECK(qspi_handler != NULL);
        HAL_RET_T result = HAL_NOR_Config(qspi_handler, &norconf);
        HAL_SANITY_CHECK(result == HAL_OK);
    }
}

/* FFT TX module need to check dma done */
int dma0_memcpy_check_done(void)
{
    if (dma0_handler == NULL)
        return 0;

    if (ts_ll_dma_ch_is_enable(dma0_handler->base, DMA0_CHN))
        return 0;

    return 1;
}

void dma0_memcpy_2d(void *dest, void *src, uint32_t len, uint32_t stride, uint32_t total_len, ModeArg_t mode)
{
    dma0_memcpy_init();
    if (mode == MODE_FLASH_TO_MEM) {
        uint32_t src_addr = (uint32_t)src - QSPI_BASE_ADDR;
        uint32_t dst_addr = (uint32_t)dest;

        HAL_LOCK(qspi_handler->lock);
        init_qspi_dma_with_config(DMA0_REG, DMA0_CHN, DMA_TRANS_WIDTH_32b);
        ts_ll_qspi_set_dma_single_size(qspi_handler->qspi_hnd->base, 2); /*4Bytes*/
        ts_ll_qspi_set_dma_burst_size(qspi_handler->qspi_hnd->base, 4);  /*16Bytes*/
        while (total_len > 0) {
            ts_ll_qspi_set_intmask(qspi_handler->qspi_hnd->base, QSPI_ALLINT_DIS);
            /*Indirect Read Transfer Start Address Register*/
            ts_ll_qspi_set_indac_rd_addr(qspi_handler->qspi_hnd->base, src_addr);
            /*Indirect Read Transfer Number Bytes Register*/
            ts_ll_qspi_set_indac_rd_num(qspi_handler->qspi_hnd->base, len);
            /* Indirect Read Transfer Register*/
            ts_ll_qspi_start_indac_rd(qspi_handler->qspi_hnd->base); /* Start indac write*/

            fast_dma_m2m(DMA0_REG, DMA0_CHN, dst_addr, QSPI_BASE_ADDR, len >> 2);

            src_addr += stride;
            dst_addr += len;
            total_len -= len;
        }
        HAL_UNLOCK(qspi_handler->lock);

    } else {
        uint32_t src_addr = (uint32_t)src;
        uint32_t dst_addr = (uint32_t)dest;

        while (total_len > 0) {
            memcpy((void *)dst_addr, (void *)src_addr, len);

            src_addr += stride;
            dst_addr += len;
            total_len -= len;
        }
    }
}

void dma0_memcpy_lock(void)
{
    dma0_memcpy_init();
    HAL_LOCK(qspi_handler->lock);
}

void dma0_memcpy_unlock(void)
{
    HAL_UNLOCK(qspi_handler->lock);
}

/// @brief last mode used in dma0_memcpy
static ModeArg_t last_dma0_memcpy_mode = 0xffffffff;

void dma0_memcpy(void *dest, void *src, uint32_t size, ModeArg_t mode)
{
    if (src == dest)
        return;

    dma0_memcpy_init();
    if (mode == MODE_FLASH_TO_MEM) {
        uint32_t src_addr = (uint32_t)src - QSPI_BASE_ADDR;
        uint32_t dst_addr = (uint32_t)dest;
        uint32_t trans_cnt;
        // qspi_nor_para_t *qspi_nor_para;

        HAL_LOCK(qspi_handler->lock);
        if (mode == last_dma0_memcpy_mode) {
            // if the mode is the same as last time, we don't need to re-init the dma
            clear_dma_status(DMA0_REG, DMA0_CHN);
        } else {
            init_qspi_dma_with_config(DMA0_REG, DMA0_CHN, DMA_TRANS_WIDTH_32b);
        }
        /*burst size*/
        ts_ll_qspi_set_dma_single_size(qspi_handler->qspi_hnd->base, 2); /*4Bytes*/
        ts_ll_qspi_set_dma_burst_size(qspi_handler->qspi_hnd->base, 4);  /*16Bytes*/
        while (size > 0) {
#define QSPI_NOR_DMA_TFR_BLOCK_SIZE (8188)
            trans_cnt = size > QSPI_NOR_DMA_TFR_BLOCK_SIZE ? QSPI_NOR_DMA_TFR_BLOCK_SIZE : size;

            ts_ll_qspi_set_intmask(qspi_handler->qspi_hnd->base, QSPI_ALLINT_DIS);
            /*Indirect Read Transfer Start Address Register*/
            ts_ll_qspi_set_indac_rd_addr(qspi_handler->qspi_hnd->base, src_addr);
            /*Indirect Read Transfer Number Bytes Register*/
            ts_ll_qspi_set_indac_rd_num(qspi_handler->qspi_hnd->base, trans_cnt);
            /* Indirect Read Transfer Register*/
            ts_ll_qspi_start_indac_rd(qspi_handler->qspi_hnd->base); /* Start indac write*/

            fast_dma_m2m(DMA0_REG, DMA0_CHN, dst_addr, QSPI_BASE_ADDR, trans_cnt >> 2);

            src_addr += trans_cnt;
            dst_addr += trans_cnt;
            size -= trans_cnt;
        }
        HAL_UNLOCK(qspi_handler->lock);
    } else if (mode == MODE_FFT_TO_MEM) {
        init_dma_fft_tx_config(DMA0_REG, DMA0_CHN, DMA_TRANS_WIDTH_32b);
        ts_ll_dma_ch_set_src_addr(dma0_chn->base, (uint32_t)src);
        ts_ll_dma_ch_set_dst_addr(dma0_chn->base, (uint32_t)dest);
        ts_ll_dma_ch_set_block_size(dma0_chn->base, (size >> 2) & 0xfff);
        ts_ll_dma_ch_enable(DMA0_REG, DMA0_CHN);
    } else if (mode == MODE_MEM_TO_FFT) {
        init_dma_fft_rx_config(DMA0_REG, DMA0_CHN, DMA_TRANS_WIDTH_32b);
        fast_dma_m2m(DMA0_REG, DMA0_CHN, (int32_t)dest, (int32_t)src, size >> 2);
    } else {
        int32_t offset  = (uint32_t)dest > (uint32_t)src ? (uint32_t)dest - (uint32_t)src : (uint32_t)src - (uint32_t)dest;
        int8_t  isAlign = !((((uint32_t)dest) & 3) || (((uint32_t)src) & 3));

        /*
         * memcpy is faster when the offset is too small when overlapped,
         * this is the empirical value
         */
        if (offset < size && offset < 0x40 && isAlign) {
            memmove(dest, src, size);
            return;
        }

        int32_t count_per_block = 0;
        int32_t done_bytes      = 0;
        int32_t unit            = 0;

        if (src < dest) {
            done_bytes = size;
            if (isAlign) {
                init_fast_dma_with_config(DMA0_REG, DMA0_CHN, DMA_ADDR_INCREMENT, DMA_TRANS_WIDTH_32b);
                unit = 2;
            } else {
                init_fast_dma_with_config(DMA0_REG, DMA0_CHN, DMA_ADDR_INCREMENT, DMA_TRANS_WIDTH_8b);
            }
        } else {
            done_bytes = 0;
            if (isAlign) {
                init_fast_dma_with_config(DMA0_REG, DMA0_CHN, DMA_ADDR_INCREMENT, DMA_TRANS_WIDTH_32b);
                unit = 2;
            } else {
                init_fast_dma_with_config(DMA0_REG, DMA0_CHN, DMA_ADDR_INCREMENT, DMA_TRANS_WIDTH_8b);
            }
        }
        clear_dma_status(DMA0_REG, DMA0_CHN);

        int32_t count            = size >> unit;
        int32_t left             = unit > 0 ? size % (1 << unit) : 0;
        int32_t max_transfer_cnt = (offset >> unit) > DMAC_MAX_SIZE_PER_BLOCK ? DMAC_MAX_SIZE_PER_BLOCK : (offset >> unit);

        if (src < dest) {
            /* do the high left bytes first if src < dest */
            if (left && isAlign) {
                for (int32_t i = 1; i <= left; i++) {
                    *((int8_t *)dest + done_bytes - i) = *((int8_t *)src + done_bytes - i);
                }
                done_bytes -= left;
            }

            while (count > 0) {
                count_per_block = count > max_transfer_cnt ? max_transfer_cnt : count;
                done_bytes -= count_per_block << unit;
                count -= count_per_block;
                fast_dma_m2m(DMA0_REG, DMA0_CHN, (int32_t)((int8_t *)dest + done_bytes),
                             (int32_t)((int8_t *)src + done_bytes), count_per_block);
            }
        } else {
            while (count > 0) {
                count_per_block = count > max_transfer_cnt ? max_transfer_cnt : count;
                fast_dma_m2m(DMA0_REG, DMA0_CHN, (int32_t)((int8_t *)dest + done_bytes),
                             (int32_t)((int8_t *)src + done_bytes), count_per_block);
                count -= count_per_block;
                done_bytes += count_per_block << unit;
            }

            /* do the low left bytes at last if src > dest */
            if (left && isAlign) {
                for (int32_t i = 0; i < left; i++) {
                    *((int8_t *)dest + done_bytes + i) = *((int8_t *)src + done_bytes + i);
                }
            }
        }
    }

    last_dma0_memcpy_mode = mode;
}


extern HAL_SEMA_T prefetch_config_ready;
void dma0_memcpy_llp(void *dest, void *src, uint32_t size, ModeArg_t mode)
{
    if (src == dest)
        return;

    dma0_memcpy_init();
    if (mode == MODE_FLASH_TO_MEM) {
        uint32_t src_addr = (uint32_t)src - QSPI_BASE_ADDR;
        uint32_t dst_addr = (uint32_t)dest;
        uint32_t trans_cnt;
        // qspi_nor_para_t *qspi_nor_para;
        ts_tiny_printf("d0\n");

        HAL_LOCK(qspi_handler->lock);
        if (mode == last_dma0_memcpy_mode) {
            // if the mode is the same as last time, we don't need to re-init the dma
            clear_dma_status(DMA0_REG, DMA0_CHN);
        } else {
            init_qspi_dma_with_config(DMA0_REG, DMA0_CHN, DMA_TRANS_WIDTH_32b);
        }
        /*burst size*/
        ts_ll_qspi_set_dma_single_size(qspi_handler->qspi_hnd->base, 2); /*4Bytes*/
        ts_ll_qspi_set_dma_burst_size(qspi_handler->qspi_hnd->base, 4);  /*16Bytes*/
        ts_tiny_printf("d1\n");
        HAL_SEMA_GIVE(prefetch_config_ready);
        ts_tiny_printf("d2\n");
        while (size > 0) {
#define QSPI_NOR_DMA_TFR_BLOCK_SIZE (8188 * 2)
            trans_cnt = size > QSPI_NOR_DMA_TFR_BLOCK_SIZE ? QSPI_NOR_DMA_TFR_BLOCK_SIZE : size;

            ts_ll_qspi_set_intmask(qspi_handler->qspi_hnd->base, QSPI_ALLINT_DIS);
            /*Indirect Read Transfer Start Address Register*/
            ts_ll_qspi_set_indac_rd_addr(qspi_handler->qspi_hnd->base, src_addr);
            /*Indirect Read Transfer Number Bytes Register*/
            ts_ll_qspi_set_indac_rd_num(qspi_handler->qspi_hnd->base, trans_cnt);
            /* Indirect Read Transfer Register*/
            ts_ll_qspi_start_indac_rd(qspi_handler->qspi_hnd->base); /* Start indac write*/

            fast_dma_m2m(DMA0_REG, DMA0_CHN, dst_addr, QSPI_BASE_ADDR, trans_cnt >> 2);

            src_addr += trans_cnt;
            dst_addr += trans_cnt;
            size -= trans_cnt;
        }
        HAL_UNLOCK(qspi_handler->lock);
    }
    ts_tiny_printf("d3\n");
}


#endif