/*
 * Team Id			: 1028
 * Author List		: Charvi Gupta, Aarohi Vaidya, Manish Gawali, Nairuti Suradkar
 * Filename			: directions.h
 * Theme			: Bothoven
 * Functions		: motion_set(),forward(),back(),left(),soft_left(),soft_right(),soft_left_2(),soft_right_2(),stop(),adjust_left(),
					adjust_right(),angle_rotate(),left_degrees(),right_degrees(),soft_left_degrees(),soft_right_degrees(),forward_mm(),
					back_mm(),buzzer_on(),buzzer_off()
 * Global Variables: V,threshold, node_count, dir, ADC_value, Left_white_line,Center_white_line,Right_white_line,ShaftCountLeft,ShaftCountRight
 */

#ifndef DIRECTIONS_H_
#define DIRECTIONS_H_
#define V 48
#define threshold 70
unsigned int node_count=0;
int dir=0;
unsigned char ADC_Conversion(unsigned char Ch);
unsigned char ADC_Value=0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
void print_sensor(char row, char coloumn,unsigned char channel);

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}
//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/*
* Function Name: adjust_left()
* Input		 : void
* Output    : void
* Logic		 : it will rotate left until robot comes on the black line
* Example Call : adjust_left();
*/

void adjust_left()
{
	while(1)
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if(Left_white_line<threshold && Center_white_line>threshold && Right_white_line<threshold) //this condition will check weather robot is on black line or not and if it is on black line than it will break the loop
		{
			break;
		}
		velocity(150,150);
		left();//this will take left turn until it find black line
	}
}

/*
* Function Name: adjust_right()
* Input		 : void
* Output		 : void
* Logic		 : it will rotate right until robot comes on the black line
* Example Call : adjust_right();
*/
void adjust_right()
{
	while(1)
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor


		if(Left_white_line<threshold && Center_white_line>threshold && Right_white_line<threshold) //this condition will check weather robot is on black line or not and if it is on black line than it will break the loop
		{
			break;
		}
		velocity(120,120);
		right();//this will take left turn until it find black line
	}
}

void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{
			break;
		}
	}
	stop(); //Stop robot
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}

//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

/*
* Function Name: follow_line()
* Input		 : void
* Output		 : void
* Logic		 :	The Robot follows the black line. 
				Three white line sensors are used to make sure that the bot follows the black line. 
				The node_count is incremented if the bot comes across a node. 
				 
* Example Call : follow_line();
*/

void follow_line(void)
{
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		if((((Left_white_line > threshold) & ( Center_white_line > threshold) )| ((Right_white_line> threshold )&( Center_white_line> threshold))) & ((ShaftCountLeft>9) |( ShaftCountRight>9)))
		{
			ShaftCountRight=0,ShaftCountLeft=0;		//this are the safety variables, which is enable to detect only one NODE at a time.
			node_count=node_count+1;
			//velocity(250,250);							// node count will increase after one node is detected properly
		}
		if(node_count)
		{
			stop();
			break;
		}
		forward();
		
		if((Left_white_line > threshold) & ( Center_white_line <threshold )&( Right_white_line <threshold))
		{
			stop();
			velocity(100,150);
			soft_left();	//soft left
		}
		
		if((Left_white_line<threshold) &( Center_white_line >threshold) & (Right_white_line <threshold))
		{
			
			velocity(250,250);
			forward();
		}
		
		if((Left_white_line<threshold )& (Center_white_line <threshold) & (Right_white_line >threshold))
		{
			stop();
			velocity(150,100);
			soft_right();
		}
		
	}
}

void follow_line_out(void)
{
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		if(((Left_white_line > threshold) & ( Center_white_line > threshold) )& (Right_white_line> threshold ) & ((ShaftCountLeft>9) |( ShaftCountRight>9)))
		{
			ShaftCountRight=0,ShaftCountLeft=0;		//this are the safety variables, which is enable to detect only one NODE at a time.
			node_count=node_count+1;
			//velocity(250,250);							// node count will increase after one node is detected properly
		}
		if(node_count)
		{
			stop();
			
			break;
		}
		forward();
		
		if((Left_white_line > threshold) & ( Center_white_line <threshold )&( Right_white_line <threshold))
		{
			stop();
			velocity(100,150);
			soft_left();	//soft left
		}
		
		if((Left_white_line<threshold) &( Center_white_line >threshold) & (Right_white_line <threshold))
		{
			
			velocity(250,250);
			forward();
		}
		
		if((Left_white_line<threshold )& (Center_white_line <threshold) & (Right_white_line >threshold))
		{
			stop();
			velocity(150,100);
			soft_right();
		}
		
	}
}

/*
* Function Name:linear_distance_mm()
* Input		:DistanceInMM->The distance to be traversed(in mm)
* Output	: void
* Logic		: Calcultaes distance and moves only that much distance
* Example Call :linear_distance_mm();
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			stop();
			break;
		}
	}
	stop(); //Stop robot
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
#endif /* DIRECTIONS_H_ */