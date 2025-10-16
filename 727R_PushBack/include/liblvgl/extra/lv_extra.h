/**
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/lv_extra.h
 * @file lv_extra.h
 *
 */

#ifndef LV_EXTRA_H
#define LV_EXTRA_H
=======
 * @file lv_nuttx_image_cache.h
 *
 */

#ifndef LV_NUTTX_IMAGE_CACHE_H
#define LV_NUTTX_IMAGE_CACHE_H
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/lv_extra.h

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/lv_extra.h
#include "layouts/lv_layouts.h"
#include "libs/lv_libs.h"
#include "others/lv_others.h"
#include "themes/lv_themes.h"
#include "widgets/lv_widgets.h"
=======
#include "../../lv_conf_internal.h"
#include LV_STDBOOL_INCLUDE
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/lv_extra.h

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/lv_extra.h
/**
 * Initialize the extra components
 */
void lv_extra_init(void);
=======
void lv_nuttx_image_cache_init(bool use_independent_image_heap);

void lv_nuttx_image_cache_deinit(void);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/lv_extra.h

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/lv_extra.h
#endif /*LV_EXTRA_H*/
=======
#endif /*LV_NUTTX_IMAGE_CACHE_H*/
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/lv_extra.h
