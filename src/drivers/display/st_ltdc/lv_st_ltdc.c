/**
 * @file lv_st_ltdc.c
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "../../../lv_conf_internal.h"
#if LV_USE_ST_LTDC

#include "lv_st_ltdc.h"
#include "../../../display/lv_display_private.h"
#include "../../../draw/sw/lv_draw_sw.h"
#include "ChibiOS-Contrib/os/hal/ports/STM32/LLD/LTDCv1/hal_stm32_ltdc.h"
#include <string.h>
#include "lvgl_thread.h"

#if LV_ST_LTDC_USE_DMA2D_FLUSH
    #if LV_USE_DRAW_DMA2D
        #error cannot use LV_ST_LTDC_USE_DMA2D_FLUSH with LV_USE_DRAW_DMA2D
    #endif /*LV_USE_DRAW_DMA2D*/

    #include "dma2d.h"
#endif /*LV_ST_LTDC_USE_DMA2D_FLUSH*/

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

#if LV_USE_OS != LV_OS_NONE
    typedef lv_thread_sync_t sync_t;
#else
    typedef volatile bool sync_t;
#endif

/**********************
 *  STATIC PROTOTYPES
 **********************/

 extern const ltdc_color_t adu_palette[256];

static const ltdc_window_t ltdc_fullscreen_wincfg = {
    0,
    400 - 1,
    0,
    800 - 1,
};

static const ltdc_frame_t ltdc_view_frmcfg1 = {
    view_buffer,
    400,
    800,
    400 * sizeof(uint8_t),
    LTDC_FMT_L8,
};

static const ltdc_laycfg_t ltdc_view_laycfg1 = {
    &ltdc_view_frmcfg1,
    &ltdc_fullscreen_wincfg,
    LTDC_COLOR_FUCHSIA,
    0xFF,
    0x980088,
    adu_palette,
    256,
    LTDC_BLEND_FIX1_FIX2,
    (LTDC_LEF_ENABLE | LTDC_LEF_PALETTE),
};

static const ltdc_frame_t ltdc_screen_frmcfg1 = {
    frame_buffer,
    400,
    800,
    400 * 3,
    LTDC_FMT_RGB888,
};

static const ltdc_laycfg_t ltdc_screen_laycfg1 = {
    &ltdc_screen_frmcfg1,
    &ltdc_fullscreen_wincfg,
    LTDC_COLOR_FUCHSIA,
    0xFF,
    0x980088,
    NULL,
    0,
    LTDC_BLEND_FIX1_FIX2,
    LTDC_LEF_ENABLE,
};

static const LTDCConfig ltdc_cfg = {
    /* Display specifications.*/
    400, /**< Screen pixel width.*/
    800, /**< Screen pixel height.*/
    10,  /**< Horizontal sync pixel width.*/
    2,   /**< Vertical sync pixel height.*/
    20,  /**< Horizontal back porch pixel width.*/
    2,   /**< Vertical back porch pixel height.*/
    10,  /**< Horizontal front porch pixel width.*/
    4,   /**< Vertical front porch pixel height.*/
    0,   /**< Driver configuration flags.*/

    /* ISR callbacks.*/
    NULL, /**< Line Interrupt ISR, or @p NULL.*/
    NULL, /**< Register Reload ISR, or @p NULL.*/
    NULL, /**< FIFO Underrun ISR, or @p NULL.*/
    NULL, /**< Transfer Error ISR, or @p NULL.*/

    /* Color and layer settings.*/
    LTDC_COLOR_TEAL,
    &ltdc_view_laycfg1,
    NULL,
};

static lv_display_t * create(void * buf1, void * buf2, uint32_t buf_size, uint32_t layer_idx,
                             lv_display_render_mode_t mode);
static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void flush_wait_cb(lv_display_t * disp);
static lv_color_format_t get_lv_cf_from_layer_cf(uint32_t cf);
static void reload_event_callback(LTDCDriver * LTDCD1);

#if LV_ST_LTDC_USE_DMA2D_FLUSH
    static void transfer_complete_callback(DMA2D_HandleTypeDef * hdma2d);
    static uint32_t get_dma2d_output_cf_from_layer_cf(uint32_t cf);
    static uint32_t get_dma2d_input_cf_from_lv_cf(uint32_t cf);
#endif

/**********************
 *  STATIC VARIABLES
 **********************/

static struct {
    bool disp_flushed_in_flush_cb[MAX_LAYER];
    sync_t sync[MAX_LAYER];
    volatile bool layer_interrupt_is_owned[MAX_LAYER];
#if LV_ST_LTDC_USE_DMA2D_FLUSH
    volatile uint32_t dma2d_interrupt_owner; /*layer_idx + 1, or 0 for none*/
#endif
} g_data;

/**********************
 *      MACROS
 **********************/

#if LV_USE_OS != LV_OS_NONE
    #define SYNC_INIT(layer_idx) lv_thread_sync_init(&g_data.sync[layer_idx])
    #define SYNC_WAIT(layer_idx) lv_thread_sync_wait(&g_data.sync[layer_idx])
    #define SYNC_SIGNAL_ISR(layer_idx) lv_thread_sync_signal_isr(&g_data.sync[layer_idx])
#else
    #define SYNC_INIT(layer_idx) do { g_data.sync[layer_idx] = false; } while(0)
    #define SYNC_WAIT(layer_idx) do { while(!g_data.sync[layer_idx]); g_data.sync[layer_idx] = false; } while(0)
    #define SYNC_SIGNAL_ISR(layer_idx) do { g_data.sync[layer_idx] = true; } while(0)
#endif

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_display_t * lv_st_ltdc_create_direct(void * fb_adr_1, void * fb_adr_2, uint32_t layer_idx)
{
    return create(fb_adr_1, fb_adr_2, 0, layer_idx, LV_DISPLAY_RENDER_MODE_DIRECT);
}

lv_display_t * lv_st_ltdc_create_partial(void * render_buf_1, void * render_buf_2, uint32_t buf_size,
                                         uint32_t layer_idx)
{
    return create(render_buf_1, render_buf_2, buf_size, layer_idx, LV_DISPLAY_RENDER_MODE_PARTIAL);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

 static lv_display_t * create(void * buf1, void * buf2, uint32_t buf_size, uint32_t layer_idx, lv_display_render_mode_t mode)
 {
     LTDCConfig *layer_cfg = &ltdc_cfg;
     uint16_t layer_width = layer_cfg->screen_width;
     uint16_t layer_height = layer_cfg->screen_height;
     lv_color_format_t cf = LV_COLOR_FORMAT_RGB888;
 
     lv_display_t * disp = lv_display_create(layer_width, layer_height);
     lv_display_set_color_format(disp, cf);
     lv_display_set_flush_cb(disp, flush_cb);
     lv_display_set_flush_wait_cb(disp, flush_wait_cb);
     lv_display_set_driver_data(disp, (void *)(uintptr_t)layer_idx);
 
     if(mode == LV_DISPLAY_RENDER_MODE_DIRECT) {
         uint32_t cf_size = lv_color_format_get_size(cf);
         lv_display_set_buffers(disp, buf1, buf2, layer_width * layer_height * cf_size, LV_DISPLAY_RENDER_MODE_DIRECT);
 
         if(buf1 != NULL && buf2 != NULL) {
             ltdcStartReload(&LTDCD1, true); // Start reloading
             SYNC_INIT(layer_idx);
         }
     }
     else {
         lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
         SYNC_INIT(layer_idx);
     }
 
     return disp;
 }

 static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
 {
     uint32_t layer_idx = (uint32_t)(uintptr_t)lv_display_get_driver_data(disp);
     g_data.disp_flushed_in_flush_cb[layer_idx] = false;
 
     ltdc_laycfg_t *layer_cfg = (layer_idx == 0) ? &ltdc_cfg.bg_laycfg : &ltdc_cfg.fg_laycfg;
 
     uint32_t fb_stride = lv_color_format_get_size(layer_cfg->frame->fmt) * disp->hor_res;
     uint8_t *fb = (uint8_t *)layer_cfg->frame->bufferp;
     uint8_t *px = px_map;
 
     for(int y = area->y1; y <= area->y2; y++) {
         memcpy(fb + y * fb_stride + area->x1 * sizeof(lv_color_t),
                px, (area->x2 - area->x1 + 1) * sizeof(lv_color_t));
         px += (area->x2 - area->x1 + 1) * sizeof(lv_color_t);
     }
 
     g_data.disp_flushed_in_flush_cb[layer_idx] = true;
 }

static void flush_wait_cb(lv_display_t * disp)
{
    uint32_t layer_idx = (uint32_t)(uintptr_t)lv_display_get_driver_data(disp);
    if(!g_data.disp_flushed_in_flush_cb[layer_idx]) {
        SYNC_WAIT(layer_idx);
    }
}

static lv_color_format_t get_lv_cf_from_layer_cf(uint32_t cf)
{
    switch(cf) {
        case LTDC_FMT_ARGB8888:
            return LV_COLOR_FORMAT_ARGB8888;
        case LTDC_FMT_RGB888:
            return LV_COLOR_FORMAT_RGB888;
        case LTDC_FMT_RGB565:
            return LV_COLOR_FORMAT_RGB565;
        case LTDC_FMT_L8:
            return LV_COLOR_FORMAT_L8;
        case LTDC_FMT_AL88:
            return LV_COLOR_FORMAT_AL88;
        default:
            LV_ASSERT_MSG(0, "the LTDC color format is not supported");
    }
}

static void reload_event_callback(LTDCDriver * LTDCD1)
{
    uint32_t i;
    for(i = 0; i < MAX_LAYER; i++) {
        if(g_data.layer_interrupt_is_owned[i]) {
            g_data.layer_interrupt_is_owned[i] = false;
            SYNC_SIGNAL_ISR(i);
        }
    }
}

#if LV_ST_LTDC_USE_DMA2D_FLUSH
static void transfer_complete_callback(DMA2D_HandleTypeDef * hdma2d)
{
    DMA2D->IFCR = 0x3FU;
    uint32_t owner = g_data.dma2d_interrupt_owner;
    if(owner) {
        g_data.dma2d_interrupt_owner = 0;
        owner -= 1;
        SYNC_SIGNAL_ISR(owner);
    }
}

static uint32_t get_dma2d_output_cf_from_layer_cf(uint32_t cf)
{
    switch(cf) {
        case LTDC_PIXEL_FORMAT_ARGB8888:
            return DMA2D_OUTPUT_ARGB8888;
        case LTDC_PIXEL_FORMAT_RGB888:
            return DMA2D_OUTPUT_RGB888;
        case LTDC_PIXEL_FORMAT_RGB565:
            return DMA2D_OUTPUT_RGB565;
        default:
            LV_ASSERT_MSG(0, "DMA2D cannot output to the LTDC color format");
    }
}

static uint32_t get_dma2d_input_cf_from_lv_cf(uint32_t cf)
{
    switch(cf) {
        case LV_COLOR_FORMAT_ARGB8888:
            return DMA2D_INPUT_ARGB8888;
        case LV_COLOR_FORMAT_RGB888:
            return DMA2D_INPUT_RGB888;
        case LV_COLOR_FORMAT_RGB565:
            return DMA2D_INPUT_RGB565;
        case LV_COLOR_FORMAT_L8:
            return DMA2D_INPUT_L8;
        case LV_COLOR_FORMAT_AL88:
            return DMA2D_INPUT_AL88;
        case LV_COLOR_FORMAT_A8:
            return DMA2D_INPUT_A8;
        default:
            LV_ASSERT_MSG(0, "the LVGL color format is not a DMA2D input color format");
    }
}
#endif /*LV_ST_LTDC_USE_DMA2D_FLUSH*/

#endif /*LV_USE_ST_LTDC*/
