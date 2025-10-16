/**
 * @file lv_calendar_header_arrow.h
 *
 */

#ifndef LV_CALENDAR_HEADER_ARROW_H
#define LV_CALENDAR_HEADER_ARROW_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/calendar/lv_calendar_header_arrow.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/calendar/lv_calendar_header_arrow.h
#include "liblvgl/core/lv_obj.h"
#if LV_USE_CALENDAR_HEADER_ARROW
=======
#include "../../core/lv_obj.h"
#if LV_USE_CALENDAR && LV_USE_CALENDAR_HEADER_ARROW
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/calendar/lv_calendar_header_arrow.h
=======
#include "../../core/lv_obj.h"
#if LV_USE_CALENDAR && LV_USE_CALENDAR_HEADER_ARROW
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/calendar/lv_calendar_header_arrow.h

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/calendar/lv_calendar_header_arrow.h
<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/widgets/calendar/lv_calendar_header_arrow.h
extern const lv_obj_class_t lv_calendar_header_arrow_class;
=======
LV_ATTRIBUTE_EXTERN_DATA extern const lv_obj_class_t lv_calendar_header_arrow_class;
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/calendar/lv_calendar_header_arrow.h
=======
LV_ATTRIBUTE_EXTERN_DATA extern const lv_obj_class_t lv_calendar_header_arrow_class;
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/widgets/calendar/lv_calendar_header_arrow.h

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a calendar header with drop-drowns to select the year and month
 * @param parent    pointer to a calendar object.
 * @return          the created header
 */
lv_obj_t * lv_calendar_header_arrow_create(lv_obj_t * parent);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_CALENDAR_HEADER_ARROW*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_CALENDAR_HEADER_ARROW_H*/
