/**
 * @file lv_theme_mono.h
 *
 */

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h
#ifndef LV_USE_THEME_MONO_H
#define LV_USE_THEME_MONO_H
=======
#ifndef LV_THEME_MONO_H
#define LV_THEME_MONO_H
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h
#include "liblvgl/core/lv_obj.h"
=======
#include "../lv_theme.h"
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h

#if LV_USE_THEME_MONO

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
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h
 * @param color_primary the primary color of the theme
 * @param color_secondary the secondary color for the theme
 * @param font pointer to a font to use.
 * @return a pointer to reference this theme later
 */
lv_theme_t * lv_theme_mono_init(lv_disp_t * disp, bool dark_bg, const lv_font_t * font);
=======
 * @param disp pointer to display
 * @param dark_bg
 * @param font pointer to a font to use.
 * @return a pointer to reference this theme later
 */
lv_theme_t * lv_theme_mono_init(lv_display_t * disp, bool dark_bg, const lv_font_t * font);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h

/**
* Check if the theme is initialized
* @return true if default theme is initialized, false otherwise
*/
bool lv_theme_mono_is_inited(void);

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h
=======
/**
 * Deinitialize the mono theme
 */
void lv_theme_mono_deinit(void);

>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h
/**********************
 *      MACROS
 **********************/

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h
#endif /*LV_USE_THEME_MONO_H*/
=======
#endif /* LV_THEME_MONO_H */
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/themes/mono/lv_theme_mono.h
