/**
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/layouts/lv_layouts.h
 * @file lv_layouts.h
 *
 */

#ifndef LV_LAYOUTS_H
#define LV_LAYOUTS_H
=======
 * @file lv_vg_lite_decoder.h
 *
 */

#ifndef LV_VG_LITE_DECODER_H
#define LV_VG_LITE_DECODER_H
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/layouts/lv_layouts.h

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/layouts/lv_layouts.h
#include "flex/lv_flex.h"
#include "grid/lv_grid.h"
=======

#include "../lv_image_decoder.h"

#if LV_USE_DRAW_VG_LITE
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/layouts/lv_layouts.h

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/layouts/lv_layouts.h
/**********************
 *      MACROS
 **********************/
#if LV_USE_LOG && LV_LOG_TRACE_LAYOUT
#  define LV_TRACE_LAYOUT(...) LV_LOG_TRACE(__VA_ARGS__)
#else
#  define LV_TRACE_LAYOUT(...)
#endif
=======
void lv_vg_lite_decoder_init(void);

void lv_vg_lite_decoder_deinit(void);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_DRAW_VG_LITE*/
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/layouts/lv_layouts.h

#ifdef __cplusplus
} /*extern "C"*/
#endif

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/layouts/lv_layouts.h
#endif /*LV_LAYOUTS_H*/
=======
#endif /*LV_VG_LITE_DECODER_H*/
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/layouts/lv_layouts.h
