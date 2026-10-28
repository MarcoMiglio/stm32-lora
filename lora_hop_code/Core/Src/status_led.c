/*
 * status_led.c
 *
 *  Created on: 21 ott 2026
 *      Author: LabMeas
 */

#include "status_led.h"


/* --------------------- STATIC VARIABLES --------------------- */

static status_led_mode_t led_mode = LED_STATUS_OFF;
static bool blink_state = false;




/* --------------------- STATIC FUNCTIONS --------------------- */

/*
 * ALL LEDs OFF.
 */
static void leds_all_off(void)
{
	HAL_GPIO_WritePin(
			GREEN_LED_GPIO_Port,
			GREEN_LED_Pin,
			GPIO_PIN_RESET);

	HAL_GPIO_WritePin(
			YELLOW_LED_GPIO_Port,
			YELLOW_LED_Pin,
			GPIO_PIN_RESET);

	HAL_GPIO_WritePin(
			RED_LED_GPIO_Port,
			RED_LED_Pin,
			GPIO_PIN_RESET);
}






/* --------------------- PUBLIC FUNCTIONS --------------------- */

/*
 * Init LEDs all OFF: blink = false, led_mode = LED_STATUS_OFF
 */
void status_led_init(void)
{
	led_mode = LED_STATUS_OFF;
	blink_state = false;

	leds_all_off();
}


/*
 * Update LED status.
 *
 * @param status_led_mode_t mode current LED status:
 *        - NO EFFECT FLAGS  : LED_STATUS_OFF = 0, LED_STATUS_DISCOVERY, LED_STATUS_LINK_TEST (require blinking operation)
 *        - GOOD STATUS FLAGS: LED_STATUS_LINK_GREEN, LED_STATUS_LINK_ORANGE, LED_STATUS_LINK_RED
 */
void status_led_set_mode(status_led_mode_t mode)
{
	led_mode = mode;

	/*
	 * Restart blinking from OFF each time
	 * the operating state changes.
	 */
	blink_state = false;

	leds_all_off();


	/*
	 * Fixed states can be applied immediately.
	 */
	switch (led_mode)
	{
		case LED_STATUS_LINK_GREEN:

			HAL_GPIO_WritePin(
					GREEN_LED_GPIO_Port,
					GREEN_LED_Pin,
					GPIO_PIN_SET);

			break;


		case LED_STATUS_LINK_ORANGE:

			HAL_GPIO_WritePin(
					YELLOW_LED_GPIO_Port,
					YELLOW_LED_Pin,
					GPIO_PIN_SET);

			break;


		case LED_STATUS_LINK_RED:

			HAL_GPIO_WritePin(
					RED_LED_GPIO_Port,
					RED_LED_Pin,
					GPIO_PIN_SET);

			break;


		default:

			break;
	}
}


/*
 * Blink LEDs based on currrent led_mode.
 * This function is to be called inside the timer callback responsible
 * for LED blinking operation.
 */
void status_led_tick(void)
{
	switch (led_mode)
	{
		/*
		 * During discovery blink all LEDs.
		 */
		case LED_STATUS_DISCOVERY:
		{
			blink_state = !blink_state;

			GPIO_PinState state =
					blink_state ?
					GPIO_PIN_SET :
					GPIO_PIN_RESET;

			HAL_GPIO_WritePin(
					GREEN_LED_GPIO_Port,
					GREEN_LED_Pin,
					state);

			HAL_GPIO_WritePin(
					YELLOW_LED_GPIO_Port,
					YELLOW_LED_Pin,
					state);

			HAL_GPIO_WritePin(
					RED_LED_GPIO_Port,
					RED_LED_Pin,
					state);

			break;
		}


		/*
		 * Pairing completed.
		 * Link quality evaluation in progress.
		 */
		case LED_STATUS_LINK_TEST:
		{
			blink_state = !blink_state;

			leds_all_off();

			HAL_GPIO_WritePin(
					YELLOW_LED_GPIO_Port,
					YELLOW_LED_Pin,
					blink_state ?
					GPIO_PIN_SET :
					GPIO_PIN_RESET);

			break;
		}


		/*
		 * Node deployed after pairing and link test.
		 * Decide if you want to keep an LED blinking or if you prefer
		 * saving battery.
		 */
		case LED_STATUS_DEPLOYED:
		{
			blink_state = !blink_state;

			leds_all_off();

			HAL_GPIO_WritePin(
					GREEN_LED_GPIO_Port,
					GREEN_LED_Pin,
					blink_state ?
					GPIO_PIN_SET :
					GPIO_PIN_RESET);

			break;
		}


		/*
		 * Fixed states need no periodic action.
		 */
		case LED_STATUS_LINK_GREEN:
		case LED_STATUS_LINK_ORANGE:
		case LED_STATUS_LINK_RED:
		case LED_STATUS_OFF:
		default:

				break;
	}
}




















