/**
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h
 * @file lv_theme_basic.h
 *
 */

#ifndef LV_THEME_BASIC_H
#define LV_THEME_BASIC_H
=======
 * @file lv_theme_simple.h
 *
 */

#ifndef LV_THEME_SIMPLE_H
#define LV_THEME_SIMPLE_H
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h
#include "liblvgl/core/lv_obj.h"

#if LV_USE_THEME_BASIC
=======
#include "../lv_theme.h"
#include "../../display/lv_display.h"

#if LV_USE_THEME_SIMPLE
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h

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
 * Initialize the theme
 * @param disp pointer to display to attach the theme
 * @return a pointer to reference this theme later
 */
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h
lv_theme_t * lv_theme_basic_init(lv_disp_t * disp);
=======
lv_theme_t * lv_theme_simple_init(lv_display_t * disp);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h

/**
* Check if the theme is initialized
* @return true if default theme is initialized, false otherwise
*/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h
bool lv_theme_basic_is_inited(void);
=======
bool lv_theme_simple_is_inited(void);

/**
 * Get simple theme
 * @return a pointer to simple theme, or NULL if this is not initialized
 */
lv_theme_t * lv_theme_simple_get(void);

/**
 * Deinitialize the simple theme
 */
void lv_theme_simple_deinit(void);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h

/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h
#endif /*LV_THEME_BASIC_H*/
=======
#endif /*LV_THEME_SIMPLE_H*/
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/basic/lv_theme_basic.h
