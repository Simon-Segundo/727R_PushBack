/**
 * @file lv_draw_line.h
 *
 */

#ifndef LV_DRAW_LINE_H
#define LV_DRAW_LINE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream
#include "../lv_conf_internal.h"
#include "../misc/lv_color.h"
#include "../misc/lv_area.h"
#include "../misc/lv_style.h"
=======
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_style.h"
>>>>>>> Stashed changes

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef struct {
<<<<<<< Updated upstream
    lv_draw_dsc_base_t base;

    lv_point_precise_t p1;
    lv_point_precise_t p2;
    lv_color_t color;
    int32_t width;
    int32_t dash_width;
    int32_t dash_gap;
=======
    lv_color_t color;
    lv_coord_t width;
    lv_coord_t dash_width;
    lv_coord_t dash_gap;
>>>>>>> Stashed changes
    lv_opa_t opa;
    lv_blend_mode_t blend_mode  : 2;
    uint8_t round_start : 1;
    uint8_t round_end   : 1;
<<<<<<< Updated upstream
    uint8_t raw_end     : 1;    /**< Do not bother with perpendicular line ending if it's not visible for any reason */
} lv_draw_line_dsc_t;

=======
    uint8_t raw_end     : 1;    /*Do not bother with perpendicular line ending if it's not visible for any reason*/
} lv_draw_line_dsc_t;

struct _lv_draw_ctx_t;

>>>>>>> Stashed changes
/**********************
 * GLOBAL PROTOTYPES
 **********************/

<<<<<<< Updated upstream
/**
 * Initialize a line draw descriptor
 * @param dsc       pointer to a draw descriptor
 */
void lv_draw_line_dsc_init(lv_draw_line_dsc_t * dsc);

/**
 * Try to get a line draw descriptor from a draw task.
 * @param task      draw task
 * @return          the task's draw descriptor or NULL if the task is not of type LV_DRAW_TASK_TYPE_LINE
 */
lv_draw_line_dsc_t * lv_draw_task_get_line_dsc(lv_draw_task_t * task);

/**
 * Create a line draw task
 * @param layer     pointer to a layer
 * @param dsc       pointer to an initialized `lv_draw_line_dsc_t` variable
 */
void lv_draw_line(lv_layer_t * layer, const lv_draw_line_dsc_t * dsc);
=======
LV_ATTRIBUTE_FAST_MEM void lv_draw_line_dsc_init(lv_draw_line_dsc_t * dsc);

/**
 * Draw a line
 * @param point1 first point of the line
 * @param point2 second point of the line
 * @param clip the line will be drawn only in this area
 * @param dsc pointer to an initialized `lv_draw_line_dsc_t` variable
 */
void lv_draw_line(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_line_dsc_t * dsc, const lv_point_t * point1,
                  const lv_point_t * point2);

>>>>>>> Stashed changes

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRAW_LINE_H*/
