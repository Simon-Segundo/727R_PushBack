/**
 * @file lv_draw_arc.h
 *
 */

#ifndef LV_DRAW_ARC_H
#define LV_DRAW_ARC_H

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
<<<<<<< Updated upstream

typedef struct {
    lv_draw_dsc_base_t base;

    lv_color_t color;
    int32_t width;
    lv_value_precise_t start_angle;
    lv_value_precise_t end_angle;
    lv_point_t center;
    uint16_t radius;
    const void * img_src;
    lv_opa_t opa;
    uint8_t rounded : 1;
} lv_draw_arc_dsc_t;

=======
typedef struct {
    lv_color_t color;
    lv_coord_t width;
    uint16_t start_angle;
    uint16_t end_angle;
    const void * img_src;
    lv_opa_t opa;
    lv_blend_mode_t blend_mode  : 2;
    uint8_t rounded : 1;
} lv_draw_arc_dsc_t;

struct _lv_draw_ctx_t;

>>>>>>> Stashed changes
/**********************
 * GLOBAL PROTOTYPES
 **********************/

<<<<<<< Updated upstream
/**
 * Initialize an arc draw descriptor.
 * @param dsc       pointer to a draw descriptor
 */
void lv_draw_arc_dsc_init(lv_draw_arc_dsc_t * dsc);

/**
 * Try to get an arc draw descriptor from a draw task.
 * @param task      draw task
 * @return          the task's draw descriptor or NULL if the task is not of type LV_DRAW_TASK_TYPE_ARC
 */
lv_draw_arc_dsc_t * lv_draw_task_get_arc_dsc(lv_draw_task_t * task);

/**
 * Create an arc draw task.
 * @param layer         pointer to a layer
 * @param dsc           pointer to an initialized draw descriptor variable
 */
void lv_draw_arc(lv_layer_t * layer, const lv_draw_arc_dsc_t * dsc);
=======
void lv_draw_arc_dsc_init(lv_draw_arc_dsc_t * dsc);

/**
 * Draw an arc. (Can draw pie too with great thickness.)
 * @param center_x      the x coordinate of the center of the arc
 * @param center_y      the y coordinate of the center of the arc
 * @param radius        the radius of the arc
 * @param mask          the arc will be drawn only in this mask
 * @param start_angle   the start angle of the arc (0 deg on the bottom, 90 deg on the right)
 * @param end_angle     the end angle of the arc
 * @param clip_area     the arc will be drawn only in this area
 * @param dsc           pointer to an initialized `lv_draw_line_dsc_t` variable
 */
void lv_draw_arc(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_arc_dsc_t * dsc, const lv_point_t * center,
                 uint16_t radius,  uint16_t start_angle, uint16_t end_angle);
>>>>>>> Stashed changes

/**
 * Get an area the should be invalidated when the arcs angle changed between start_angle and end_ange
 * @param x             the x coordinate of the center of the arc
 * @param y             the y coordinate of the center of the arc
 * @param radius        the radius of the arc
 * @param start_angle   the start angle of the arc (0 deg on the bottom, 90 deg on the right)
 * @param end_angle     the end angle of the arc
 * @param w             width of the arc
 * @param rounded       true: the arc is rounded
 * @param area          store the area to invalidate here
 */
<<<<<<< Updated upstream
void lv_draw_arc_get_area(int32_t x, int32_t y, uint16_t radius,  lv_value_precise_t start_angle,
                          lv_value_precise_t end_angle,
                          int32_t w, bool rounded, lv_area_t * area);
=======
void lv_draw_arc_get_area(lv_coord_t x, lv_coord_t y, uint16_t radius,  uint16_t start_angle, uint16_t end_angle,
                          lv_coord_t w, bool rounded, lv_area_t * area);
>>>>>>> Stashed changes

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRAW_ARC_H*/
