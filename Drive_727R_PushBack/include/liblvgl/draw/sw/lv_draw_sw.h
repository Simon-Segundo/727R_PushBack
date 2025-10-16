/**
 * @file lv_draw_sw.h
 *
 */

#ifndef LV_DRAW_SW_H
#define LV_DRAW_SW_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream
#include "../lv_draw.h"
#if LV_USE_DRAW_SW

#include "../../misc/lv_area.h"
#include "../../misc/lv_color.h"
#include "../../display/lv_display.h"
#include "../../osal/lv_os.h"

#include "../lv_draw_vector.h"
#include "../lv_draw_triangle.h"
#include "../lv_draw_label.h"
#include "../lv_draw_image.h"
#include "../lv_draw_line.h"
#include "../lv_draw_arc.h"
=======
#include "lv_draw_sw_blend.h"
#include "liblvgl/draw/lv_draw.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/hal/lv_hal_disp.h"
>>>>>>> Stashed changes

/*********************
 *      DEFINES
 *********************/

/**********************
<<<<<<< Updated upstream
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Initialize the SW renderer. Called in internally.
 * It creates as many SW renderers as defined in LV_DRAW_SW_DRAW_UNIT_CNT
 */
void lv_draw_sw_init(void);

/**
 * Deinitialize the SW renderers
 */
void lv_draw_sw_deinit(void);

/**
 * Fill an area using SW render. Handle gradient and radius.
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 * @param coords        the coordinates of the rectangle
 */
void lv_draw_sw_fill(lv_draw_unit_t * draw_unit, lv_draw_fill_dsc_t * dsc, const lv_area_t * coords);

/**
 * Draw border with SW render.
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 * @param coords        the coordinates of the rectangle
 */
void lv_draw_sw_border(lv_draw_unit_t * draw_unit, const lv_draw_border_dsc_t * dsc, const lv_area_t * coords);

/**
 * Draw box shadow with SW render.
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 * @param coords        the coordinates of the rectangle for which the box shadow should be drawn
 */
void lv_draw_sw_box_shadow(lv_draw_unit_t * draw_unit, const lv_draw_box_shadow_dsc_t * dsc, const lv_area_t * coords);

/**
 * Draw an image with SW render. It handles image decoding, tiling, transformations, and recoloring.
 * @param draw_unit     pointer to a draw unit
 * @param draw_dsc      the draw descriptor
 * @param coords        the coordinates of the image
 */
void lv_draw_sw_image(lv_draw_unit_t * draw_unit, const lv_draw_image_dsc_t * draw_dsc,
                      const lv_area_t * coords);

/**
 * Draw a label with SW render.
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 * @param coords        the coordinates of the label
 */
void lv_draw_sw_label(lv_draw_unit_t * draw_unit, const lv_draw_label_dsc_t * dsc, const lv_area_t * coords);

/**
 * Draw an arc with SW render.
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 * @param coords        the coordinates of the arc
 */
void lv_draw_sw_arc(lv_draw_unit_t * draw_unit, const lv_draw_arc_dsc_t * dsc, const lv_area_t * coords);

/**
 * Draw a line with SW render.
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 */
void lv_draw_sw_line(lv_draw_unit_t * draw_unit, const lv_draw_line_dsc_t * dsc);

/**
 * Blend a layer with SW render
 * @param draw_unit     pointer to a draw unit
 * @param draw_dsc      the draw descriptor
 * @param coords        the coordinates of the layer
 */
void lv_draw_sw_layer(lv_draw_unit_t * draw_unit, const lv_draw_image_dsc_t * draw_dsc, const lv_area_t * coords);

/**
 * Draw a triangle with SW render.
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 */
void lv_draw_sw_triangle(lv_draw_unit_t * draw_unit, const lv_draw_triangle_dsc_t * dsc);

/**
 * Mask out a rectangle with radius from a current layer
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 * @param coords        the coordinates of the mask
 */
void lv_draw_sw_mask_rect(lv_draw_unit_t * draw_unit, const lv_draw_mask_rect_dsc_t * dsc, const lv_area_t * coords);

/**
 * Used internally to get a transformed are of an image
 * @param draw_unit     pointer to a draw unit
 * @param dest_area     area to calculate, i.e. get this area from the transformed image
 * @param src_buf       source buffer
 * @param src_w         source buffer width in pixels
 * @param src_h         source buffer height in pixels
 * @param src_stride    source buffer stride in bytes
 * @param draw_dsc      draw descriptor
 * @param sup           supplementary data
 * @param cf            color format of the source buffer
 * @param dest_buf      the destination buffer
 */
void lv_draw_sw_transform(lv_draw_unit_t * draw_unit, const lv_area_t * dest_area, const void * src_buf,
                          int32_t src_w, int32_t src_h, int32_t src_stride,
                          const lv_draw_image_dsc_t * draw_dsc, const lv_draw_image_sup_t * sup, lv_color_format_t cf, void * dest_buf);

#if LV_USE_VECTOR_GRAPHIC && LV_USE_THORVG
/**
 * Draw vector graphics with SW render.
 * @param draw_unit     pointer to a draw unit
 * @param dsc           the draw descriptor
 */
void lv_draw_sw_vector(lv_draw_unit_t * draw_unit, const lv_draw_vector_task_dsc_t * dsc);
#endif

/**
 * Swap the upper and lower byte of an RGB565 buffer.
 * Might be required if a 8bit parallel port or an SPI port send the bytes in the wrong order.
 * The bytes will be swapped in place.
 * @param buf           pointer to buffer
 * @param buf_size_px   number of pixels in the buffer
 */
void lv_draw_sw_rgb565_swap(void * buf, uint32_t buf_size_px);

/**
 * Invert a draw buffer in the I1 color format.
 * Conventionally, a bit is set to 1 during blending if the luminance is greater than 127.
 * Depending on the display controller used, you might want to have different behavior.
 * The inversion will be performed in place.
 * @param buf          pointer to the buffer to be inverted
 * @param buf_size     size of the buffer in bytes
 */
void lv_draw_sw_i1_invert(void * buf, uint32_t buf_size);

/**
 * Rotate a buffer into another buffer
 * @param src           the source buffer
 * @param dest          the destination buffer
 * @param src_width     source width in pixels
 * @param src_height    source height in pixels
 * @param src_stride     source stride in bytes (number of bytes in a row)
 * @param dest_stride   destination stride in bytes (number of bytes in a row)
 * @param rotation      LV_DISPLAY_ROTATION_0/90/180/270
 * @param color_format  LV_COLOR_FORMAT_RGB565/RGB888/XRGB8888/ARGB8888
 */
void lv_draw_sw_rotate(const void * src, void * dest, int32_t src_width, int32_t src_height, int32_t src_stride,
                       int32_t dest_stride, lv_display_rotation_t rotation, lv_color_format_t color_format);
=======
 *      TYPEDEFS
 **********************/

struct _lv_disp_drv_t;

typedef struct {
    lv_draw_ctx_t base_draw;

    /** Fill an area of the destination buffer with a color*/
    void (*blend)(lv_draw_ctx_t * draw_ctx, const lv_draw_sw_blend_dsc_t * dsc);
} lv_draw_sw_ctx_t;

typedef struct {
    lv_draw_layer_ctx_t base_draw;

    uint32_t buf_size_bytes: 31;
    uint32_t has_alpha : 1;
} lv_draw_sw_layer_ctx_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lv_draw_sw_init_ctx(struct _lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx);
void lv_draw_sw_deinit_ctx(struct _lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx);

void lv_draw_sw_wait_for_finish(lv_draw_ctx_t * draw_ctx);

void lv_draw_sw_arc(lv_draw_ctx_t * draw_ctx, const lv_draw_arc_dsc_t * dsc, const lv_point_t * center, uint16_t radius,
                    uint16_t start_angle, uint16_t end_angle);

void lv_draw_sw_rect(lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * dsc, const lv_area_t * coords);

void lv_draw_sw_bg(lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * dsc, const lv_area_t * coords);
void lv_draw_sw_letter(lv_draw_ctx_t * draw_ctx, const lv_draw_label_dsc_t * dsc, const lv_point_t * pos_p,
                       uint32_t letter);

LV_ATTRIBUTE_FAST_MEM void lv_draw_sw_img_decoded(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_img_dsc_t * draw_dsc,
                                                  const lv_area_t * coords, const uint8_t * src_buf, lv_img_cf_t cf);

LV_ATTRIBUTE_FAST_MEM void lv_draw_sw_line(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_line_dsc_t * dsc,
                                           const lv_point_t * point1, const lv_point_t * point2);

void lv_draw_sw_polygon(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * draw_dsc,
                        const lv_point_t * points, uint16_t point_cnt);

void lv_draw_sw_buffer_copy(lv_draw_ctx_t * draw_ctx,
                            void * dest_buf, lv_coord_t dest_stride, const lv_area_t * dest_area,
                            void * src_buf, lv_coord_t src_stride, const lv_area_t * src_area);

void lv_draw_sw_transform(lv_draw_ctx_t * draw_ctx, const lv_area_t * dest_area, const void * src_buf,
                          lv_coord_t src_w, lv_coord_t src_h, lv_coord_t src_stride,
                          const lv_draw_img_dsc_t * draw_dsc, lv_img_cf_t cf, lv_color_t * cbuf, lv_opa_t * abuf);

struct _lv_draw_layer_ctx_t * lv_draw_sw_layer_create(struct _lv_draw_ctx_t * draw_ctx, lv_draw_layer_ctx_t * layer_ctx,
                                                      lv_draw_layer_flags_t flags);

void lv_draw_sw_layer_adjust(struct _lv_draw_ctx_t * draw_ctx, struct _lv_draw_layer_ctx_t * layer_ctx,
                             lv_draw_layer_flags_t flags);

void lv_draw_sw_layer_blend(struct _lv_draw_ctx_t * draw_ctx, struct _lv_draw_layer_ctx_t * layer_ctx,
                            const lv_draw_img_dsc_t * draw_dsc);

void lv_draw_sw_layer_destroy(lv_draw_ctx_t * draw_ctx, lv_draw_layer_ctx_t * layer_ctx);
>>>>>>> Stashed changes

/***********************
 * GLOBAL VARIABLES
 ***********************/

/**********************
 *      MACROS
 **********************/

<<<<<<< Updated upstream
#include "blend/lv_draw_sw_blend.h"

#endif /*LV_USE_DRAW_SW*/

=======
>>>>>>> Stashed changes
#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRAW_SW_H*/
