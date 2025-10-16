/**
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h
 * @file lv_png.h
 *
 */

#ifndef LV_PNG_H
#define LV_PNG_H
=======
 * @file lv_libpng.h
 *
 */

#ifndef LV_LIBPNG_H
#define LV_LIBPNG_H
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h
#include "liblvgl/lv_conf_internal.h"
#if LV_USE_PNG
=======
#include "../../lv_conf_internal.h"
#if LV_USE_LIBPNG
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h

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
 * Register the PNG decoder functions in LVGL
 */
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h
void lv_png_init(void);
=======
void lv_libpng_init(void);

void lv_libpng_deinit(void);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h

/**********************
 *      MACROS
 **********************/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h
#endif /*LV_USE_PNG*/
=======
#endif /*LV_USE_LIBPNG*/
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h

#ifdef __cplusplus
} /* extern "C" */
#endif

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h
#endif /*LV_PNG_H*/
=======
#endif /*LV_LIBPNG_H*/
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/png/lv_png.h
