/**
 * @file lv_keyboard.h
 *
 */

#ifndef LV_KEYBOARD_H
#define LV_KEYBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
#include "liblvgl/widgets/lv_btnmatrix.h"
=======
#include "../buttonmatrix/lv_buttonmatrix.h"
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h

#if LV_USE_KEYBOARD

/*Testing of dependencies*/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
#if LV_USE_BTNMATRIX == 0
#error "lv_kb: lv_btnm is required. Enable it in lv_conf.h (LV_USE_BTNMATRIX  1) "
#endif

#if LV_USE_TEXTAREA == 0
#error "lv_kb: lv_ta is required. Enable it in lv_conf.h (LV_USE_TEXTAREA  1) "
=======
#if LV_USE_BUTTONMATRIX == 0
#error "lv_buttonmatrix is required. Enable it in lv_conf.h (LV_USE_BUTTONMATRIX  1) "
#endif

#if LV_USE_TEXTAREA == 0
#error "lv_textarea is required. Enable it in lv_conf.h (LV_USE_TEXTAREA  1) "
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
#endif

/*********************
 *      DEFINES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
#define LV_KEYBOARD_CTRL_BTN_FLAGS (LV_BTNMATRIX_CTRL_NO_REPEAT | LV_BTNMATRIX_CTRL_CLICK_TRIG | LV_BTNMATRIX_CTRL_CHECKED)
=======
#define LV_KEYBOARD_CTRL_BUTTON_FLAGS (LV_BUTTONMATRIX_CTRL_NO_REPEAT | LV_BUTTONMATRIX_CTRL_CLICK_TRIG | LV_BUTTONMATRIX_CTRL_CHECKED)
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h

/**********************
 *      TYPEDEFS
 **********************/

/** Current keyboard mode.*/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
enum {
=======
typedef enum {
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
    LV_KEYBOARD_MODE_TEXT_LOWER,
    LV_KEYBOARD_MODE_TEXT_UPPER,
    LV_KEYBOARD_MODE_SPECIAL,
    LV_KEYBOARD_MODE_NUMBER,
    LV_KEYBOARD_MODE_USER_1,
    LV_KEYBOARD_MODE_USER_2,
    LV_KEYBOARD_MODE_USER_3,
    LV_KEYBOARD_MODE_USER_4,
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
};
typedef uint8_t lv_keyboard_mode_t;

/*Data of keyboard*/
typedef struct {
    lv_btnmatrix_t btnm;
    lv_obj_t * ta;              /*Pointer to the assigned text area*/
    lv_keyboard_mode_t mode;    /*Key map type*/
    uint8_t popovers : 1;       /*Show button titles in popovers on press*/
} lv_keyboard_t;

extern const lv_obj_class_t lv_keyboard_class;
=======
#if LV_USE_ARABIC_PERSIAN_CHARS == 1
    LV_KEYBOARD_MODE_TEXT_ARABIC
#endif
} lv_keyboard_mode_t;

#if LV_USE_OBJ_PROPERTY
enum {
    LV_PROPERTY_ID(KEYBOARD, TEXTAREA,            LV_PROPERTY_TYPE_OBJ,   0),
    LV_PROPERTY_ID(KEYBOARD, MODE,                LV_PROPERTY_TYPE_INT,   1),
    LV_PROPERTY_ID(KEYBOARD, POPOVERS,            LV_PROPERTY_TYPE_INT,   2),
    LV_PROPERTY_ID(KEYBOARD, SELECTED_BUTTON,     LV_PROPERTY_TYPE_INT,   3),
    LV_PROPERTY_KEYBOARD_END,
};
#endif

LV_ATTRIBUTE_EXTERN_DATA extern const lv_obj_class_t lv_keyboard_class;
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a Keyboard object
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param parent pointer to an object, it will be the parent of the new keyboard
 * @return pointer to the created keyboard
=======
 * @param parent    pointer to an object, it will be the parent of the new keyboard
 * @return          pointer to the created keyboard
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 */
lv_obj_t * lv_keyboard_create(lv_obj_t * parent);

/*=====================
 * Setter functions
 *====================*/

/**
 * Assign a Text Area to the Keyboard. The pressed characters will be put there.
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param kb pointer to a Keyboard object
 * @param ta pointer to a Text Area object to write there
=======
 * @param kb        pointer to a Keyboard object
 * @param ta        pointer to a Text Area object to write there
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 */
void lv_keyboard_set_textarea(lv_obj_t * kb, lv_obj_t * ta);

/**
 * Set a new a mode (text or number map)
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param kb pointer to a Keyboard object
 * @param mode the mode from 'lv_keyboard_mode_t'
=======
 * @param kb        pointer to a Keyboard object
 * @param mode      the mode from 'lv_keyboard_mode_t'
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 */
void lv_keyboard_set_mode(lv_obj_t * kb, lv_keyboard_mode_t mode);

/**
 * Show the button title in a popover when pressed.
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param kb pointer to a Keyboard object
 * @param en whether "popovers" mode is enabled
=======
 * @param kb        pointer to a Keyboard object
 * @param en        whether "popovers" mode is enabled
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 */
void lv_keyboard_set_popovers(lv_obj_t * kb, bool en);

/**
 * Set a new map for the keyboard
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param kb pointer to a Keyboard object
 * @param mode keyboard map to alter 'lv_keyboard_mode_t'
 * @param map pointer to a string array to describe the map.
 *            See 'lv_btnmatrix_set_map()' for more info.
 */
void lv_keyboard_set_map(lv_obj_t * kb, lv_keyboard_mode_t mode, const char * map[],
                         const lv_btnmatrix_ctrl_t ctrl_map[]);
=======
 * @param kb        pointer to a Keyboard object
 * @param mode      keyboard map to alter 'lv_keyboard_mode_t'
 * @param map       pointer to a string array to describe the map.
 *                  See 'lv_buttonmatrix_set_map()' for more info.
 * @param ctrl_map  See 'lv_buttonmatrix_set_ctrl_map()' for more info.

 */
void lv_keyboard_set_map(lv_obj_t * kb, lv_keyboard_mode_t mode, const char * const map[],
                         const lv_buttonmatrix_ctrl_t ctrl_map[]);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h

/*=====================
 * Getter functions
 *====================*/

/**
 * Assign a Text Area to the Keyboard. The pressed characters will be put there.
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param kb pointer to a Keyboard object
 * @return pointer to the assigned Text Area object
=======
 * @param kb        pointer to a Keyboard object
 * @return          pointer to the assigned Text Area object
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 */
lv_obj_t * lv_keyboard_get_textarea(const lv_obj_t * kb);

/**
 * Set a new a mode (text or number map)
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param kb pointer to a Keyboard object
 * @return the current mode from 'lv_keyboard_mode_t'
=======
 * @param kb        pointer to a Keyboard object
 * @return          the current mode from 'lv_keyboard_mode_t'
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 */
lv_keyboard_mode_t lv_keyboard_get_mode(const lv_obj_t * kb);

/**
 * Tell whether "popovers" mode is enabled or not.
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param kb pointer to a Keyboard object
 * @return true: "popovers" mode is enabled; false: disabled
 */
bool lv_btnmatrix_get_popovers(const lv_obj_t * obj);

/**
 * Get the current map of a keyboard
 * @param kb pointer to a keyboard object
 * @return the current map
 */
static inline const char ** lv_keyboard_get_map_array(const lv_obj_t * kb)
{
    return lv_btnmatrix_get_map(kb);
}
=======
 * @param obj       pointer to a Keyboard object
 * @return          true: "popovers" mode is enabled; false: disabled
 */
bool lv_keyboard_get_popovers(const lv_obj_t * obj);

/**
 * Get the current map of a keyboard
 * @param kb        pointer to a keyboard object
 * @return          the current map
 */
const char * const * lv_keyboard_get_map_array(const lv_obj_t * kb);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h

/**
 * Get the index of the lastly "activated" button by the user (pressed, released, focused etc)
 * Useful in the `event_cb` to get the text of the button, check if hidden etc.
 * @param obj       pointer to button matrix object
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @return          index of the last released button (LV_BTNMATRIX_BTN_NONE: if unset)
 */
static inline uint16_t lv_keyboard_get_selected_btn(const lv_obj_t * obj)
{
    return lv_btnmatrix_get_selected_btn(obj);
}
=======
 * @return          index of the last released button (LV_BUTTONMATRIX_BUTTON_NONE: if unset)
 */
uint32_t lv_keyboard_get_selected_button(const lv_obj_t * obj);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h

/**
 * Get the button's text
 * @param obj       pointer to button matrix object
 * @param btn_id    the index a button not counting new line characters.
 * @return          text of btn_index` button
 */
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
static inline const char * lv_keyboard_get_btn_text(const lv_obj_t * obj, uint16_t btn_id)
{
    return lv_btnmatrix_get_btn_text(obj, btn_id);
}
=======
const char * lv_keyboard_get_button_text(const lv_obj_t * obj, uint32_t btn_id);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h

/*=====================
 * Other functions
 *====================*/

/**
 * Default keyboard event to add characters to the Text area and change the map.
 * If a custom `event_cb` is added to the keyboard this function can be called from it to handle the
 * button clicks
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 * @param kb pointer to a keyboard
 * @param event the triggering event
=======
 * @param e the triggering event
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/keyboard/lv_keyboard.h
 */
void lv_keyboard_def_event_cb(lv_event_t * e);

/**********************
 *      MACROS
 **********************/

#endif  /*LV_USE_KEYBOARD*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_KEYBOARD_H*/
