/**
 * @file lv_draw_sdl.h
 *
 */

#ifndef LV_DRAW_SDL_H
#define LV_DRAW_SDL_H

<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream
#include "../lv_draw.h"

#if LV_USE_DRAW_SDL

#include "../../misc/cache/lv_cache.h"
#include "../../misc/lv_area.h"
#include "../../misc/lv_color.h"
#include "../../display/lv_display.h"
#include "../../osal/lv_os.h"
#include "../../draw/lv_draw_label.h"
#include "../../draw/lv_draw_rect.h"
#include "../../draw/lv_draw_arc.h"
#include "../../draw/lv_draw_image.h"
#include "../../draw/lv_draw_triangle.h"
#include "../../draw/lv_draw_line.h"
=======
#include "liblvgl/lv_conf_internal.h"

#if LV_USE_GPU_SDL

#include LV_GPU_SDL_INCLUDE_PATH

#include "../lv_draw.h"
#include "../../core/lv_disp.h"
>>>>>>> Stashed changes

/*********************
 *      DEFINES
 *********************/

<<<<<<< Updated upstream
=======
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
#define LV_DRAW_SDL_TEXTURE_FORMAT SDL_PIXELFORMAT_ARGB8888
#else
#define LV_DRAW_SDL_TEXTURE_FORMAT SDL_PIXELFORMAT_RGBA8888
#endif

>>>>>>> Stashed changes
/**********************
 *      TYPEDEFS
 **********************/

<<<<<<< Updated upstream
typedef struct {
    lv_draw_unit_t base_unit;
    lv_draw_task_t * task_act;
    lv_cache_t * texture_cache;
} lv_draw_sdl_unit_t;
=======
struct lv_draw_sdl_context_internals_t;

typedef struct {
    /**
     * Render for display driver
     */
    SDL_Renderer * renderer;
    void * user_data;
} lv_draw_sdl_drv_param_t;

typedef struct {
    lv_draw_ctx_t base_draw;
    SDL_Renderer * renderer;
    struct lv_draw_sdl_context_internals_t * internals;
} lv_draw_sdl_ctx_t;
>>>>>>> Stashed changes

/**********************
 * GLOBAL PROTOTYPES
 **********************/

<<<<<<< Updated upstream
void lv_draw_sdl_init(void);

void /* LV_ATTRIBUTE_FAST_MEM */ lv_draw_sdl_image(lv_draw_unit_t * draw_unit, const lv_draw_image_dsc_t * draw_dsc,
                                                   const lv_area_t * coords);

void lv_draw_sdl_fill(lv_draw_unit_t * draw_unit, const lv_draw_fill_dsc_t * dsc, const lv_area_t * coords);

void lv_draw_sdl_border(lv_draw_unit_t * draw_unit, const lv_draw_border_dsc_t * dsc, const lv_area_t * coords);

void lv_draw_sdl_box_shadow(lv_draw_unit_t * draw_unit, const lv_draw_box_shadow_dsc_t * dsc, const lv_area_t * coords);

void lv_draw_sdl_label(lv_draw_unit_t * draw_unit, const lv_draw_label_dsc_t * dsc, const lv_area_t * coords);

void lv_draw_sdl_arc(lv_draw_unit_t * draw_unit, const lv_draw_arc_dsc_t * dsc, const lv_area_t * coords);

void /* LV_ATTRIBUTE_FAST_MEM */ lv_draw_sdl_line(lv_draw_unit_t * draw_unit, const lv_draw_line_dsc_t * dsc);

void lv_draw_sdl_layer(lv_draw_unit_t * draw_unit, const lv_draw_image_dsc_t * draw_dsc, const lv_area_t * coords);

void lv_draw_sdl_triangle(lv_draw_unit_t * draw_unit, const lv_draw_triangle_dsc_t * dsc);

void lv_draw_sdl_mask_rect(lv_draw_unit_t * draw_unit, const lv_draw_mask_rect_dsc_t * dsc, const lv_area_t * coords);

/***********************
 * GLOBAL VARIABLES
 ***********************/
=======
void lv_draw_sdl_init_ctx(lv_disp_drv_t * disp_drv, lv_draw_ctx_t * draw_ctx);

/**
 * @brief Free caches
 *
 */
void lv_draw_sdl_deinit_ctx(lv_disp_drv_t * disp_drv, lv_draw_ctx_t * draw_ctx);

SDL_Texture * lv_draw_sdl_create_screen_texture(SDL_Renderer * renderer, lv_coord_t hor, lv_coord_t ver);

/*======================
 * Add/remove functions
 *=====================*/

/*=====================
 * Setter functions
 *====================*/

/*=====================
 * Getter functions
 *====================*/

/*=====================
 * Other functions
 *====================*/
>>>>>>> Stashed changes

/**********************
 *      MACROS
 **********************/

<<<<<<< Updated upstream
#endif /*LV_USE_DRAW_SDL*/
=======
#endif /*LV_USE_GPU_SDL*/
>>>>>>> Stashed changes

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRAW_SDL_H*/
