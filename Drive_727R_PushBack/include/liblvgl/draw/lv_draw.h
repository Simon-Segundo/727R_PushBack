/**
 * @file lv_draw.h
 *
 */

<<<<<<< Updated upstream
/**
 * Modified by NXP in 2024
 */

=======
>>>>>>> Stashed changes
#ifndef LV_DRAW_H
#define LV_DRAW_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream
#include "../lv_conf_internal.h"

#include "../misc/lv_types.h"
#include "../misc/lv_style.h"
#include "../misc/lv_text.h"
#include "../misc/lv_profiler.h"
#include "../misc/lv_matrix.h"
#include "lv_image_decoder.h"
#include "../osal/lv_os.h"
#include "lv_draw_buf.h"
=======
#include "liblvgl/lv_conf_internal.h"

#include "liblvgl/misc/lv_style.h"
#include "liblvgl/misc/lv_txt.h"
#include "lv_img_decoder.h"
#include "lv_img_cache.h"

#include "lv_draw_rect.h"
#include "lv_draw_label.h"
#include "lv_draw_img.h"
#include "lv_draw_line.h"
#include "lv_draw_triangle.h"
#include "lv_draw_arc.h"
#include "lv_draw_mask.h"
#include "lv_draw_transform.h"
#include "lv_draw_layer.h"
>>>>>>> Stashed changes

/*********************
 *      DEFINES
 *********************/
<<<<<<< Updated upstream
#define LV_DRAW_UNIT_NONE  0
#define LV_DRAW_UNIT_IDLE  -1   /**< The draw unit is idle, new dispatching might be requested to try again */

#if LV_DRAW_TRANSFORM_USE_MATRIX
#if !LV_USE_MATRIX
#error "LV_DRAW_TRANSFORM_USE_MATRIX requires LV_USE_MATRIX = 1"
#endif
#endif
=======
>>>>>>> Stashed changes

/**********************
 *      TYPEDEFS
 **********************/

<<<<<<< Updated upstream
typedef enum {
    LV_DRAW_TASK_TYPE_NONE = 0,
    LV_DRAW_TASK_TYPE_FILL,
    LV_DRAW_TASK_TYPE_BORDER,
    LV_DRAW_TASK_TYPE_BOX_SHADOW,
    LV_DRAW_TASK_TYPE_LABEL,
    LV_DRAW_TASK_TYPE_IMAGE,
    LV_DRAW_TASK_TYPE_LAYER,
    LV_DRAW_TASK_TYPE_LINE,
    LV_DRAW_TASK_TYPE_ARC,
    LV_DRAW_TASK_TYPE_TRIANGLE,
    LV_DRAW_TASK_TYPE_MASK_RECTANGLE,
    LV_DRAW_TASK_TYPE_MASK_BITMAP,
    LV_DRAW_TASK_TYPE_VECTOR,
} lv_draw_task_type_t;

typedef enum {
    LV_DRAW_TASK_STATE_WAITING,     /*Waiting for something to be finished. E.g. rendering a layer*/
    LV_DRAW_TASK_STATE_QUEUED,
    LV_DRAW_TASK_STATE_IN_PROGRESS,
    LV_DRAW_TASK_STATE_READY,
} lv_draw_task_state_t;

struct _lv_layer_t  {

    /** Target draw buffer of the layer*/
    lv_draw_buf_t * draw_buf;

    /** The absolute coordinates of the buffer */
    lv_area_t buf_area;

    /** The color format of the layer. LV_COLOR_FORMAT_...  */
    lv_color_format_t color_format;

    /**
     * NEVER USE IT DRAW UNITS. USED INTERNALLY DURING DRAW TASK CREATION.
     * The current clip area with absolute coordinates, always the same or smaller than `buf_area`
     * Can be set before new draw tasks are added to indicate the clip area of the draw tasks.
     * Therefore `lv_draw_add_task()` always saves it in the new draw task to know the clip area when the draw task was added.
     * During drawing the draw units also sees the saved clip_area and should use it during drawing.
     * During drawing the layer's clip area shouldn't be used as it might be already changed for other draw tasks.
     */
    lv_area_t _clip_area;

    /**
     * The physical clipping area relative to the display.
     */
    lv_area_t phy_clip_area;

#if LV_DRAW_TRANSFORM_USE_MATRIX
    /** Transform matrix to be applied when rendering the layer */
    lv_matrix_t matrix;
#endif

    /** Linked list of draw tasks */
    lv_draw_task_t * draw_task_head;

    lv_layer_t * parent;
    lv_layer_t * next;
    bool all_tasks_added;
    void * user_data;
};

typedef struct {
    lv_obj_t * obj;
    lv_part_t part;
    uint32_t id1;
    uint32_t id2;
    lv_layer_t * layer;
    size_t dsc_size;
    void * user_data;
} lv_draw_dsc_base_t;
=======
typedef struct {
    void * user_data;
} lv_draw_mask_t;

typedef struct _lv_draw_layer_ctx_t {
    lv_area_t area_full;
    lv_area_t area_act;
    lv_coord_t max_row_with_alpha;
    lv_coord_t max_row_with_no_alpha;
    void * buf;
    struct {
        const lv_area_t * clip_area;
        lv_area_t * buf_area;
        void * buf;
        bool screen_transp;
    } original;
} lv_draw_layer_ctx_t;

typedef struct _lv_draw_ctx_t  {
    /**
     *  Pointer to a buffer to draw into
     */
    void * buf;

    /**
     * The position and size of `buf` (absolute coordinates)
     */
    lv_area_t * buf_area;

    /**
     * The current clip area with absolute coordinates, always the same or smaller than `buf_area`
     */
    const lv_area_t * clip_area;


    void (*draw_rect)(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * dsc, const lv_area_t * coords);

    void (*draw_arc)(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_arc_dsc_t * dsc, const lv_point_t * center,
                     uint16_t radius,  uint16_t start_angle, uint16_t end_angle);

    void (*draw_img_decoded)(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_img_dsc_t * dsc,
                             const lv_area_t * coords, const uint8_t * map_p, lv_img_cf_t color_format);

    lv_res_t (*draw_img)(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_img_dsc_t * draw_dsc,
                         const lv_area_t * coords, const void * src);

    void (*draw_letter)(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_label_dsc_t * dsc,  const lv_point_t * pos_p,
                        uint32_t letter);


    void (*draw_line)(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_line_dsc_t * dsc, const lv_point_t * point1,
                      const lv_point_t * point2);


    void (*draw_polygon)(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * draw_dsc,
                         const lv_point_t * points, uint16_t point_cnt);


    /**
     * Get an area of a transformed image (zoomed and/or rotated)
     * @param draw_ctx      pointer to a draw context
     * @param dest_area     get this area of the result image. It assumes that the original image is placed to the 0;0 position.
     * @param src_buf       the source image
     * @param src_w         width of the source image in [px]
     * @param src_h         height of the source image in [px]
     * @param src_stride    the stride in [px].
     * @param draw_dsc      an `lv_draw_img_dsc_t` descriptor containing the transformation parameters
     * @param cf            the color format of `src_buf`
     * @param cbuf          place the colors of the pixels on `dest_area` here in RGB format
     * @param abuf          place the opacity of the pixels on `dest_area` here
     */
    void (*draw_transform)(struct _lv_draw_ctx_t * draw_ctx, const lv_area_t * dest_area, const void * src_buf,
                           lv_coord_t src_w, lv_coord_t src_h, lv_coord_t src_stride,
                           const lv_draw_img_dsc_t * draw_dsc, lv_img_cf_t cf, lv_color_t * cbuf, lv_opa_t * abuf);

    /**
     * Replace the buffer with a rect without decoration like radius or borders
     */
    void (*draw_bg)(struct _lv_draw_ctx_t * draw_ctx, const lv_draw_rect_dsc_t * draw_dsc, const lv_area_t * coords);

    /**
     * Wait until all background operations are finished. (E.g. GPU operations)
     */
    void (*wait_for_finish)(struct _lv_draw_ctx_t * draw_ctx);

    /**
     * Copy an area from buffer to an other
     * @param draw_ctx      pointer to a draw context
     * @param dest_buf      copy the buffer into this buffer
     * @param dest_stride   the width of the dest_buf in pixels
     * @param dest_area     the destination area
     * @param src_buf       copy from this buffer
     * @param src_stride    the width of src_buf in pixels
     * @param src_area      the source area.
     *
     * @note dest_area and src_area must have the same width and height
     *       but can have different x and y position.
     * @note dest_area and src_area must be clipped to the real dimensions of the buffers
     */
    void (*buffer_copy)(struct _lv_draw_ctx_t * draw_ctx, void * dest_buf, lv_coord_t dest_stride,
                        const lv_area_t * dest_area,
                        void * src_buf, lv_coord_t src_stride, const lv_area_t * src_area);

    /**
     * Initialize a new layer context.
     * The original buffer and area data are already saved from `draw_ctx` to `layer_ctx`
     * @param draw_ctx      pointer to the current draw context
     * @param layer_area    the coordinates of the layer
     * @param flags         OR-ed flags from @lv_draw_layer_flags_t
     * @return              pointer to the layer context, or NULL on error
     */
    struct _lv_draw_layer_ctx_t * (*layer_init)(struct _lv_draw_ctx_t * draw_ctx, struct _lv_draw_layer_ctx_t * layer_ctx,
                                                lv_draw_layer_flags_t flags);

    /**
     * Adjust the layer_ctx and/or draw_ctx based on the `layer_ctx->area_act`.
     * It's called only if flags has `LV_DRAW_LAYER_FLAG_CAN_SUBDIVIDE`
     * @param draw_ctx      pointer to the current draw context
     * @param layer_ctx     pointer to a layer context
     * @param flags         OR-ed flags from @lv_draw_layer_flags_t
     */
    void (*layer_adjust)(struct _lv_draw_ctx_t * draw_ctx, struct _lv_draw_layer_ctx_t * layer_ctx,
                         lv_draw_layer_flags_t flags);

    /**
     * Blend a rendered layer to `layer_ctx->area_act`
     * @param draw_ctx      pointer to the current draw context
     * @param layer_ctx     pointer to a layer context
     * @param draw_dsc      pointer to an image draw descriptor
     */
    void (*layer_blend)(struct _lv_draw_ctx_t * draw_ctx, struct _lv_draw_layer_ctx_t * layer_ctx,
                        const lv_draw_img_dsc_t * draw_dsc);

    /**
     * Destroy a layer context. The original buffer and area data of the `draw_ctx` will be restored
     * and the `layer_ctx` itself will be freed automatically.
     * @param draw_ctx      pointer to the current draw context
     * @param layer_ctx     pointer to a layer context
     */
    void (*layer_destroy)(struct _lv_draw_ctx_t * draw_ctx, lv_draw_layer_ctx_t * layer_ctx);

    /**
     * Size of a layer context in bytes.
     */
    size_t layer_instance_size;

#if LV_USE_USER_DATA
    void * user_data;
#endif

} lv_draw_ctx_t;
>>>>>>> Stashed changes

/**********************
 * GLOBAL PROTOTYPES
 **********************/

<<<<<<< Updated upstream
/**
 * Used internally to initialize the drawing module
 */
void lv_draw_init(void);

/**
 * Deinitialize the drawing module
 */
void lv_draw_deinit(void);

/**
 * Allocate a new draw unit with the given size and appends it to the list of draw units
 * @param size      the size to allocate. E.g. `sizeof(my_draw_unit_t)`,
 *                  where the first element of `my_draw_unit_t` is `lv_draw_unit_t`.
 */
void * lv_draw_create_unit(size_t size);

/**
 * Add an empty draw task to the draw task list of a layer.
 * @param layer     pointer to a layer
 * @param coords    the coordinates of the draw task
 * @return          the created draw task which needs to be
 *                  further configured e.g. by added a draw descriptor
 */
lv_draw_task_t * lv_draw_add_task(lv_layer_t * layer, const lv_area_t * coords);

/**
 * Needs to be called when a draw task is created and configured.
 * It will send an event about the new draw task to the widget
 * and assign it to a draw unit.
 * @param layer     pointer to a layer
 * @param t         pointer to a draw task
 */
void lv_draw_finalize_task_creation(lv_layer_t * layer, lv_draw_task_t * t);

/**
 * Try dispatching draw tasks to draw units
 */
void lv_draw_dispatch(void);

/**
 * Used internally to try dispatching draw tasks of a specific layer
 * @param disp      pointer to a display on which the dispatching was requested
 * @param layer     pointer to a layer
 * @return          at least one draw task is being rendered (maybe it was taken earlier)
 */
bool lv_draw_dispatch_layer(lv_display_t * disp, lv_layer_t * layer);

/**
 * Wait for a new dispatch request.
 * It's blocking if `LV_USE_OS == 0` else it yields
 */
void lv_draw_dispatch_wait_for_request(void);

/**
 * Wait for draw finish in case of asynchronous task execution.
 * If `LV_USE_OS == 0` it just return.
 */
void lv_draw_wait_for_finish(void);

/**
 * When a draw unit finished a draw task it needs to request dispatching
 * to let LVGL assign a new draw task to it
 */
void lv_draw_dispatch_request(void);

/**
 * Get the total number of draw units.
  */
uint32_t lv_draw_get_unit_count(void);

/**
 * Find and available draw task
 * @param layer             the draw ctx to search in
 * @param t_prev            continue searching from this task
 * @param draw_unit_id      check the task where `preferred_draw_unit_id` equals this value or `LV_DRAW_UNIT_NONE`
 * @return                  tan available draw task or NULL if there is no any
 */
lv_draw_task_t * lv_draw_get_next_available_task(lv_layer_t * layer, lv_draw_task_t * t_prev, uint8_t draw_unit_id);

/**
 * Tell how many draw task are waiting to be drawn on the area of `t_check`.
 * It can be used to determine if a GPU shall combine many draw tasks into one or not.
 * If a lot of tasks are waiting for the current ones it makes sense to draw them one-by-one
 * to not block the dependent tasks' rendering
 * @param t_check   the task whose dependent tasks shall be counted
 * @return          number of tasks depending on `t_check`
 */
uint32_t lv_draw_get_dependent_count(lv_draw_task_t * t_check);

/**
 * Create (allocate) a new layer on a parent layer
 * @param parent_layer      the parent layer to which the layer will be merged when it's rendered
 * @param color_format      the color format of the layer
 * @param area              the areas of the layer (absolute coordinates)
 * @return                  the new target_layer or NULL on error
 */
lv_layer_t * lv_draw_layer_create(lv_layer_t * parent_layer, lv_color_format_t color_format, const lv_area_t * area);

/**
 * Initialize a layer which is allocated by the user
 * @param layer             pointer the layer to initialize (its lifetime needs to be managed by the user)
 * @param parent_layer      the parent layer to which the layer will be merged when it's rendered
 * @param color_format      the color format of the layer
 * @param area              the areas of the layer (absolute coordinates)
 * @return                  the new target_layer or NULL on error
 */
void lv_draw_layer_init(lv_layer_t * layer, lv_layer_t * parent_layer, lv_color_format_t color_format,
                        const lv_area_t * area);

/**
 * Try to allocate a buffer for the layer.
 * @param layer             pointer to a layer
 * @return                  pointer to the allocated aligned buffer or NULL on failure
 */
void * lv_draw_layer_alloc_buf(lv_layer_t * layer);

/**
 * Got to a pixel at X and Y coordinate on a layer
 * @param layer             pointer to a layer
 * @param x                 the target X coordinate
 * @param y                 the target X coordinate
 * @return                  `buf` offset to point to the given X and Y coordinate
 */
void * lv_draw_layer_go_to_xy(lv_layer_t * layer, int32_t x, int32_t y);

/**
 * Get the type of a draw task
 * @param t   the draw task to get the type of
 * @return    the draw task type
*/
lv_draw_task_type_t lv_draw_task_get_type(const lv_draw_task_t * t);

/**
 * Get the draw descriptor of a draw task
 * @param t   the draw task to get the draw descriptor of
 * @return    a void pointer to the draw descriptor
*/
void * lv_draw_task_get_draw_dsc(const lv_draw_task_t * t);

/**
 * Get the draw area of a draw task
 * @param t      the draw task to get the draw area of
 * @param area   the destination where the draw area will be stored
*/
void lv_draw_task_get_area(const lv_draw_task_t * t, lv_area_t * area);
=======
void lv_draw_init(void);


void lv_draw_wait_for_finish(lv_draw_ctx_t * draw_ctx);
>>>>>>> Stashed changes

/**********************
 *  GLOBAL VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

<<<<<<< Updated upstream
=======
/**********************
 *   POST INCLUDES
 *********************/

>>>>>>> Stashed changes
#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRAW_H*/
