/**
 * @file lv_indev_scroll.h
 *
 */

#ifndef LV_INDEV_SCROLL_H
#define LV_INDEV_SCROLL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
#include "lv_obj.h"
=======
#include "../core/lv_obj.h"
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
=======
#include "../core/lv_obj.h"
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/core/lv_indev_scroll.h

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Handle scrolling. Called by LVGL during input device processing
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
 * @param proc      pointer to an input device's proc field
 */
void _lv_indev_scroll_handler(_lv_indev_proc_t * proc);

/**
 * Handle throwing after scrolling. Called by LVGL during input device processing
 * @param proc      pointer to an input device's proc field
 */
void _lv_indev_scroll_throw_handler(_lv_indev_proc_t * proc);
=======
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
 * @param indev      pointer to an input device
 */
void lv_indev_scroll_handler(lv_indev_t * indev);

/**
 * Handle throwing after scrolling. Called by LVGL during input device processing
 * @param indev      pointer to an input device
 */
void lv_indev_scroll_throw_handler(lv_indev_t * indev);
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/core/lv_indev_scroll.h

/**
 * Predict where would a scroll throw end
 * @param indev     pointer to an input device
 * @param dir `     LV_DIR_VER` or `LV_DIR_HOR`
 * @return          the difference compared to the current position when the throw would be finished
 */
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
lv_coord_t lv_indev_scroll_throw_predict(lv_indev_t * indev, lv_dir_t dir);
=======
int32_t lv_indev_scroll_throw_predict(lv_indev_t * indev, lv_dir_t dir);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/core/lv_indev_scroll.h
=======
int32_t lv_indev_scroll_throw_predict(lv_indev_t * indev, lv_dir_t dir);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/core/lv_indev_scroll.h

/**
 * Get the distance of the nearest snap point
 * @param obj       the object on which snap points should be found
 * @param p         save the distance of the found snap point there
 */
void lv_indev_scroll_get_snap_dist(lv_obj_t * obj, lv_point_t * p);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_INDEV_SCROLL_H*/
