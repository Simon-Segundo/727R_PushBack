/**
 * @file lv_monkey.h
 *
 */
#ifndef LV_MONKEY_H
#define LV_MONKEY_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
#include "liblvgl/lvgl.h"
=======
#include "../../lv_conf_internal.h"
#include "../../indev/lv_indev.h"
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h

#if LV_USE_MONKEY != 0

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
struct _lv_monkey;
typedef struct _lv_monkey lv_monkey_t;

typedef struct {
=======

typedef struct _lv_monkey_t lv_monkey_t;

struct _lv_monkey_config_t {
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
    /**< Input device type*/
    lv_indev_type_t type;

    /**< Monkey execution period*/
    struct {
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
        uint32_t min;
        uint32_t max;
=======
        //! @cond Doxygen_Suppress
        uint32_t min;
        uint32_t max;
        //! @endcond
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
    } period_range;

    /**< The range of input value*/
    struct {
        int32_t min;
        int32_t max;
    } input_range;
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
} lv_monkey_config_t;
=======
};
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Initialize a monkey config with default values
 * @param config pointer to 'lv_monkey_config_t' variable to initialize
 */
void lv_monkey_config_init(lv_monkey_config_t * config);

/**
 * Create monkey for test
 * @param config pointer to 'lv_monkey_config_t' variable
 * @return pointer to the created monkey
 */
lv_monkey_t * lv_monkey_create(const lv_monkey_config_t * config);

/**
 * Get monkey input device
 * @param monkey pointer to a monkey
 * @return pointer to the input device
 */
lv_indev_t * lv_monkey_get_indev(lv_monkey_t * monkey);

/**
 * Enable monkey
 * @param monkey pointer to a monkey
 * @param en set to true to enable
 */
void lv_monkey_set_enable(lv_monkey_t * monkey, bool en);

/**
 * Get whether monkey is enabled
 * @param monkey pointer to a monkey
 * @return return true if monkey enabled
 */
bool lv_monkey_get_enable(lv_monkey_t * monkey);

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
#if LV_USE_USER_DATA

=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
/**
 * Set the user_data field of the monkey
 * @param monkey   pointer to a monkey
 * @param user_data   pointer to the new user_data.
 */
void lv_monkey_set_user_data(lv_monkey_t * monkey, void * user_data);

/**
 * Get the user_data field of the monkey
 * @param monkey pointer to a monkey
 * @return the pointer to the user_data of the monkey
 */
void * lv_monkey_get_user_data(lv_monkey_t * monkey);

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
#endif/*LV_USE_USER_DATA*/

=======
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
/**
 * Delete monkey
 * @param monkey pointer to monkey
 */
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h
void lv_monkey_del(lv_monkey_t * monkey);
=======
void lv_monkey_delete(lv_monkey_t * monkey);
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/others/monkey/lv_monkey.h

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_MONKEY*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_MONKEY_H*/
