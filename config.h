/*
 * Team Id			: 1028
 * Author List		: Charvi Gupta, Aarohi Vaidya, Manish Gawali, Nairuti Suradkar
 * Filename			: config.h
 * Theme			: Bothoven
 * Functions		: left_encoder_pin_config(),right_encoder_pin_config(),lcd_port_config(),adc_pin_config(),motion_pin_config(),
						buzzer_pin_config(),MOSFET_switch_config()
 * Global Variables	: None
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/*
* Function Name	: left_encoder_pin_config(); 
* Input			: void
* Output		: void
* Logic			: configure INT4 (PORTE 4) pin as input for the right position encoder 
* Example Call	: left_encoder_pin_config();
*/

void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name	: right_encoder_pin_config();
* Input			: void
* Output		: void
* Logic			: configure INT5 (PORTE 5) pin as input for the right position encoder
* Example Call	: right_encoder_pin_config();
*/

void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name	: lcd_port_config();
* Input			: void
* Output		: void
* Logic			: LCD port is configured(Port C).
* Example Call	: lcd_port_config();
*/

void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

/*
* Function Name : adc_pin_config();
* Input		    : void
* Output		: void
* Logic			: ADC pin configuration(Port F)
* Example Call	: adc_pin_config();
*/

void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

/*
* Function Name	: motion_pin_config();
* Input			: void
* Output		: void
* Logic			: configure ports to enable robot's motion.(Port A,Port L)
* Example Call	: motion_pin_config();
*/

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
* Function Name	: buzzer_pin_config();
* Input			: void
* Output		: void
* Logic			: configure ports to enable buzzer.(PortC 3)
* Example Call	: buzzer_pin_config();
*/

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;	//Setting PORTC 3 logic low to turnoff buzzer
}

/*
* Function Name	: MOSFET_switch_config();
* Input			: void
* Output		: void
* Logic			: MOSFET switch port configuration(Port H,Port G)
* Example Call	: MOSFET_switch_config();
*/

void MOSFET_switch_config (void)
{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}
#endif /* CONFIG_H_ */