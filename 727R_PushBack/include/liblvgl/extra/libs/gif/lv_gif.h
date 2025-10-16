/**
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
 * @file lv_gif.h
 *
 */

#ifndef LV_GIF_H
#define LV_GIF_H
=======
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
 * @file lv_gif_private.h
 *
 */

#ifndef LV_GIF_PRIVATE_H
#define LV_GIF_PRIVATE_H
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
#include "liblvgl/lvgl.h"
#if LV_USE_GIF

#include "gifdec.h"
=======
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
#include "../../widgets/image/lv_image_private.h"
#include "lv_gif.h"

#if LV_USE_GIF
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
typedef struct {
    lv_img_t img;
    gd_GIF * gif;
    lv_timer_t * timer;
    lv_img_dsc_t imgdsc;
    uint32_t last_call;
} lv_gif_t;

extern const lv_obj_class_t lv_gif_class;
=======
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
/**********************
 *      TYPEDEFS
 **********************/

struct _lv_gif_t {
    lv_image_t img;
    gd_GIF * gif;
    lv_timer_t * timer;
    lv_image_dsc_t imgdsc;
    uint32_t last_call;
};

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h

/**********************
 * GLOBAL PROTOTYPES
 **********************/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
lv_obj_t * lv_gif_create(lv_obj_t * parent);
void lv_gif_set_src(lv_obj_t * obj, const void * src);
void lv_gif_restart(lv_obj_t * gif);

=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
/**********************
 *      MACROS
 **********************/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
#endif /*LV_USE_GIF*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_GIF_H*/
=======
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
#endif /* LV_USE_GIF */

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_GIF_PRIVATE_H*/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/gif/lv_gif.h
