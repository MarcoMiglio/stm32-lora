/*
 * status_led.h
 *
 *  Created on: 21 ott 2026
 *      Author: LabMeas
 */

#ifndef INC_STATUS_LED_H_
#define INC_STATUS_LED_H_

#include "main.h"

typedef enum
{
    LED_STATUS_OFF = 0,

    LED_STATUS_DISCOVERY,
    LED_STATUS_LINK_TEST,
		LED_STATUS_DEPLOYED,

    LED_STATUS_LINK_GREEN,
    LED_STATUS_LINK_ORANGE,
    LED_STATUS_LINK_RED

} status_led_mode_t;


void status_led_init(void);

void status_led_set_mode(status_led_mode_t mode);

void status_led_tick(void);




#endif /* INC_STATUS_LED_H_ */
