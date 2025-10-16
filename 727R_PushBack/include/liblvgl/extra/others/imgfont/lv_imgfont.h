/**
 * @file lv_imgfont.h
 *
 */

#ifndef LV_IMGFONT_H
#define LV_IMGFONT_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
#include "liblvgl/lvgl.h"
=======
#include "../../lv_conf_internal.h"
#include "../../font/lv_font.h"
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
=======
#include "../../lv_conf_internal.h"
#include "../../font/lv_font.h"
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h

#if LV_USE_IMGFONT

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/* gets the image path name of this character */
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
typedef bool (*lv_get_imgfont_path_cb_t)(const lv_font_t * font, void * img_src,
                                         uint16_t len, uint32_t unicode, uint32_t unicode_next);
=======
typedef const void * (*lv_imgfont_get_path_cb_t)(const lv_font_t * font,
                                                 uint32_t unicode, uint32_t unicode_next,
                                                 int32_t * offset_y, void * user_data);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
=======
typedef const void * (*lv_imgfont_get_path_cb_t)(const lv_font_t * font,
                                                 uint32_t unicode, uint32_t unicode_next,
                                                 int32_t * offset_y, void * user_data);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Creates a image font with info parameter specified.
 * @param height font size
 * @param path_cb a function to get the image path name of character.
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
 * @return pointer to the new imgfont or NULL if create error.
 */
lv_font_t * lv_imgfont_create(uint16_t height, lv_get_imgfont_path_cb_t path_cb);
=======
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
 * @param user_data pointer to user data
 * @return pointer to the new imgfont or NULL if create error.
 */
lv_font_t * lv_imgfont_create(uint16_t height, lv_imgfont_get_path_cb_t path_cb, void * user_data);
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h
=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/imgfont/lv_imgfont.h

/**
 * Destroy a image font that has been created.
 * @param font pointer to image font handle.
 */
void lv_imgfont_destroy(lv_font_t * font);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_IMGFONT*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* LV_IMGFONT_H */
