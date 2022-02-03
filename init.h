/*
 * Team Id			: 1028
 * Author List		: Charvi Gupta, Aarohi Vaidya, Manish Gawali, Nairuti Suradkar
 * Filename			: init.h
 * Theme			: Bothoven
 * Functions		: left_position_encoder_interrupt_init(),right_position_encoder_interrupt_init(),port_init(),timer5_init(),
						init_devices(),graph_init(),adc_init() >
 * Global Variables	: graph[][],i,j
 */

#ifndef INIT_H_
#define INIT_H_
//graph : adjacency matrix
//i,j : to initialize graph
 unsigned int graph[V][V],i=0,j=0;

/*
* Function Name	: left_position_encoder_interrupt_init()
* Input			: void
* Output		: void
* Logic			: Initialization of Left Position Encoder Interrupt
* Example Call	: left_position_encoder_interrupt_init();
*/

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli();					//Clears the global interrupt
	EICRB = EICRB | 0x02;	// INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10;	// Enable Interrupt INT4 for left position encoder
	sei();					// Enables the global interrupt
}

/*
* Function Name	: right_position_encoder_interrupt_init()
* Input			: void
* Output		: void
* Logic			: Initialization of Right Position Encoder Interrupt
* Example Call	: right_position_encoder_interrupt_init();
*/

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli();					//Clears the global interrupt
	EICRB = EICRB | 0x08;	// INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20;	// Enable Interrupt INT5 for right position encoder
	sei();					// Enables the global interrupt
}

/*
* Function Name	: port_init()
* Input			: void
* Output		: void
* Logic			: Initialize PORTS
* Example Call	: port_init();
*/

void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	left_encoder_pin_config();	//left encoder pin config
	right_encoder_pin_config(); //right encoder pin config	
	buzzer_pin_config();
	MOSFET_switch_config();
}

/*
* Function Name : timer5_init()
* Input			: void
* Output		: void
* Logic			: Timer 5 initialized in PWM mode for velocity control,Prescale:256,PWM 8bit fast,TOP=0x00FF,Timer Frequency:225.000 Hz
* Example Call	: timer5_init();
*/

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*
* Function Name	: adc_init()
* Input			: void
* Output		: void
* Logic			: Initialization of ADC
* Example Call	: adc_init();
*/

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*
* Function Name	: init_devices()
* Input			: void
* Output		: void
* Logic			: Initialisation of port,adc,timer5,left position encoder interrupt,right position encoder interrupt
* Example Call	: init_devices();
*/

void init_devices (void)
{
	cli();		//Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	uart0_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();		//Enables the global interrupts
}

/*
* Function Name	: graph_init()
* Input			: void
* Output		: void
* Logic			: Initialisation of adjacency matrix,
					If adj[i][j] = w, then there is an edge from vertex i to vertex j with weight w.
* Example Call	: graph_init();
*/

void graph_init (void)
{
	for(i=0;i<V;i++)
	{
		for(j=0;j<V;j++)
		{
			graph[i][j]=0;
		}
	}
	for(i=0;i<23;i++)
	{
		j=i+1;
		graph[i][j]=1;                 
		graph[j][i]=1;

	}
	graph[23][0]=1;graph[0][23]=1;
	graph[22][24]=1;graph[24][22]=1;
	graph[24][37]=1;graph[37][24]=1;
	graph[22][35]=1;graph[35][22]=1;
	graph[35][36]=1;graph[36][35]=1;
	graph[36][37]=1;graph[37][36]=1;
	graph[25][37]=1;graph[37][25]=1;
	graph[2][25]=1;graph[25][2]=1;
	graph[2][26]=1;graph[26][2]=1;
	graph[25][26]=1;graph[26][25]=1;
	graph[26][38]=1;graph[38][26]=1;
	graph[37][43]=1;graph[43][37]=1;
	graph[43][44]=1;graph[44][43]=1;
	graph[38][44]=1;graph[44][38]=1;
	graph[38][39]=1;graph[39][38]=1;
	graph[38][27]=1;graph[27][38]=1;
	graph[6][27]=1;graph[27][6]=1;
	graph[6][28]=1;graph[28][6]=1;
	graph[28][39]=1;graph[39][28]=1;
	graph[29][39]=1;graph[39][29]=1;
	graph[10][29]=1;graph[29][10]=1;
	graph[39][45]=1;graph[45][39]=1;
	graph[45][46]=1;graph[46][45]=1;
	graph[40][46]=1;graph[46][40]=1;
	graph[30][40]=1;graph[40][30]=1;
	graph[29][30]=1;graph[30][29]=1;
	graph[10][30]=1;graph[30][10]=1;
	graph[40][41]=1;graph[41][40]=1;
	graph[31][40]=1;graph[40][31]=1;
	graph[14][31]=1;graph[31][14]=1;
	graph[14][32]=1;graph[32][14]=1;
	graph[32][41]=1;graph[41][32]=1;
	graph[41][47]=1;graph[47][41]=1;
	graph[42][47]=1;graph[47][42]=1;
	graph[36][42]=1;graph[42][36]=1;
	graph[34][36]=1;graph[36][34]=1;
	graph[33][34]=1;graph[34][33]=1;
	graph[33][41]=1;graph[41][33]=1;
	graph[18][34]=1;graph[34][18]=1;
	graph[18][33]=1;graph[33][18]=1;	
}

#endif /* INIT_H_ */