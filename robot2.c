#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <limits.h>
#include "lcd.h"
#include "directions.h"
#include "config.h"
#include "init.h"
#include <stdbool.h>
unsigned int graph[V][V],i,j,one,two,total;
volatile unsigned int path[V],path1[V],path2[V],nodes,nodes1,nodes2,num,num1,num2,z=0,rbt2_tens;
unsigned char ADC_Value;
unsigned char Left_white_line;
unsigned char Center_white_line;
unsigned char Right_white_line;
volatile unsigned int len,len1,len2,present1,present2,shortest_new;
unsigned int node_count,dircount=2;
volatile int prev_angle,store,charvi,flag_break=0;
volatile int min_dis=99,distance;
volatile int current,curr,shortest,shortest1,shortest2,valid,flagir=0;
volatile int select_next,next,indexi,go_mnp;
volatile int retrdist,mnpno,begin,iteration;
volatile int prev_obs;
volatile unsigned long int ShaftCountLeft; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight; //to keep track of right position encoder
int mnp[9][6]={ {0,24,25,37,-1,-1},
                {25,26,37,38,43,44},
                {4,26,27,38,-1,-1},
                {8,28,29,39,-1,-1},
                {29,30,39,40,45,46},
                {12,30,31,40,-1,-1},
                {16,32,33,41,-1,-1},
                {33,34,36,41,42,47},
                {20,34,35,36,-1,-1}
    };
volatile unsigned char rbt2_len[2];
volatile unsigned int rbt2_length,isr_count=0,flag_isr=0;
int tx_flag=0x30;
volatile int nai=0;
volatile int mnplist_main[]={7,29,26,18,24,13,30,16,20};
volatile int mnp_robot1[20],mnp_robot2[20],robot1_index,robot2_index,robot2_mnp[20];
volatile int mnplength_main=9;
volatile unsigned char digit[40];
volatile unsigned int digit_index=0;
volatile int mnplist[]={1,2,3};
volatile int mnplength,man;
volatile unsigned char irsense;
volatile unsigned int count_isr;
void obsadjust(int first,int second);
//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
unsigned char rxbyte()
{
	unsigned char rx;
	while(!(UCSR0A & 0x80));
	rx=UDR0;
	return(rx);
}
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

void fi()
{
	unsigned char data;
	data=UDR0;
	lcd_wr_char(data);
}
SIGNAL(USART0_RX_vect) 		// ISR for receive complete interrupt
{
	UCSR0B = 0x18;
	unsigned char data;
	if(isr_count<2)
	{
		data=UDR0;
		
		rbt2_len[count_isr]=data;
		count_isr=isr_count+1;
		isr_count++;
		
		
		_delay_ms(5);
	}
	if ((isr_count>1) & (isr_count<rbt2_length*2+2))
	{
		data = UDR0; 
		//lcd_wr_char(data);
		digit[digit_index]=data;
		digit_index++;
		count_isr=isr_count+1;
		isr_count++;
		
	}
	if (isr_count>rbt2_length*2)
	{
		data=UDR0;
		lcd_wr_char(data);
		flag_isr=data-48;
		
		
	}		
	UCSR0B = 0x98;
}

void txbyte(unsigned char tx)
{
	while(!(UCSR0A & 0x20));
	UDR0=tx;
}

unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

void turn_on_sharp234_wl (void) //turn on Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG & 0xFB;
}

void turn_off_sharp234_wl (void) //turn off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG | 0x04;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

int minDistance(int dist[], bool sptSet[])
{
	// Initialize min value
	int min = INT_MAX, min_index;
	int v;
	for (v = 0; v < V; v++)
	if (sptSet[v] == false && dist[v] < min)
	min = dist[v], min_index = v;

	return min_index;
}

// Function to print shortest path from source to j
// using parent array

void printPath1(int parent[], int j)
{
	// Base Case : If j is source
	path1[nodes1]=j+1;
	nodes1=nodes1-1;
	if (nodes1==-1)
	{
		return;
	}
	printPath1(parent, parent[j]);
}

void path_follow_out(int first, int second, int third)
{
	if(third<25)
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line_out();		
		if(first%2==0)
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			forward_mm(40);			
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			right_degrees(40);
			prev_angle=-60;
			adjust_right();
			stop();
			back_mm(40);
			_delay_ms(100);
			dircount++;
		}
		else
		if(first%2==1)
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			forward_mm(60);
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			right_degrees(100);
			prev_angle=-120;	
			adjust_right();
			stop();
			back_mm(20);
			dircount++;
			_delay_ms(100);
		}
	}
	else
	if(((first==28)&(third==29))|((first==32)&(third==33))|((first==36)&(third==25)))
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line_out();
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		forward_mm(40);
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		right_degrees(90);
		prev_angle=-90;
		adjust_right();
		stop();
		_delay_ms(100);
	}
	else
	if(((first==29)&(third==28))|((first==33)&(third==32))|((first==25)&(third==36)))
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line_out();
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		forward_mm(40);
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		left_degrees(90);
		prev_angle=90;
		adjust_left();
		stop();
		_delay_ms(100);
	}
}
void path_follow(int first,int second,int third)
{
	if(second>24)
	dircount=2;
	if(first==third)
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line();
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		forward_mm(40);
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		right_degrees(140);
		dircount=dircount+1;
		prev_angle=-140;
		adjust_right();
		stop();
		_delay_ms(100);
	}
	else
	if(((first==27)&(third==40))|((first==41)&(third==34))|((first==40)&(third==27))|((first==37)&(third==26))|((first==31)&(third==42))|((first==39)&(third==30))|((first==42)&(third==31))|((first==38)&(third==35))|((first==26)&(third==37))|((first==30)&(third==39))|((first==35)&(third==38))|((first==34)&(third==41))|((first==41)&(third==15))|((first==38)&(third==3))|((first==41)&(third==11))|((first==40)&(third==11))|((first==39)&(third==3))|((first==37)&(third==19))|((first==42)&(third==19))|((first==42)&(third==15))|((first==37)&(third==23))|((first==38)&(third==23))|((first==39)&(third==7))|((first==40)&(third==7))|((first==3)&(third==39))|((first==3)&(third==38))|((first==7)&(third==39))|((first==7)&(third==40))|((first==11)&(third==40))|((first==11)&(third==41))|((first==15)&(third==41))|((first==15)&(third==42))|((first==19)&(third==37))|((first==19)&(third==42))|((first==23)&(third==37))|((first==23)&(third==38)))
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line();
		_delay_ms(100);
		prev_angle=0;
		
	}	
	else		
	if(((first==27)&(third==3))|((first==31)&(third==11))|((first==35)&(third==19))|((first==3)&(third==26))|((first==11)&(third==30))|((first==19)&(third==34))|((first==25)&(third==37))|((first==47)&(third==42))|((first==40)&(third==28))|((first==45)&(third==40))|((first==43)&(third==38))|((first==33)&(third==41))|((first==29)&(third==39)))
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line(); 
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		forward_mm(40);
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		right_degrees(90);
		prev_angle=-100;
		adjust_right();
		stop();
		_delay_ms(100);
	}
	else
	if(((first==27)&(third==45))|((first==41)&(third==48))|((first==39)&(third==46))|((first==37)&(third==44))|((first==48)&(third==34))|((first==47)&(third==32))|((first==46)&(third==30))|((first==45)&(third==28))|((first==44)&(third==26))|((first==43)&(third==36))|((first==28)&(third==27))|((first==42)&(third==32))|((first==38)&(third==36))|((first==29)&(third==46))|((first==32)&(third==31))|((first==33)&(third==48))|((first==36)&(third==35))|((first==25)&(third==44))|((first==31)&(third==47))|((first==26)&(third==25))|((first==30)&(third==29))|((first==35)&(third==43))|((first==34)&(third==33))|((first==26)&(third==39))|((first==30)&(third==41))|((first==34)&(third==37))|((first==41)&(third==46))|((first==47)&(third==40))|((first==37)&(third==48))|((first==43)&(third==42))|((first==39)&(third==44))|((first==45)&(third==38))|((first==42)&(third==35))|((first==38)&(third==27))|((first==40)&(third==31)))
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line();
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		forward_mm(50);
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		right_degrees(30);
		prev_angle=-60;
		adjust_right();
		stop();
		_delay_ms(100);
	}
	else
	if(((first==27)&(third==38))|((first==31)&(third==40))|((first==35)&(third==42))|((first==37)&(third==34))|((first==39)&(third==26))|((first==41)&(third==30))|((first==27)&(third==28))|((first==41)&(third==33))|((first==40)&(third==45))|((first==39)&(third==29))|((first==37)&(third==25))|((first==48)&(third==33))|((first==47)&(third==31))|((first==46)&(third==29))|((first==45)&(third==27))|((first==44)&(third==25))|((first==43)&(third==35))|((first==28)&(third==45))|((first==42)&(third==47))|((first==38)&(third==43))|((first==29)&(third==30))|((first==32)&(third==47))|((first==33)&(third==34))|((first==36)&(third==43))|((first==25)&(third==26))|((first==31)&(third==32))|((first==26)&(third==44))|((first==30)&(third==46))|((first==35)&(third==36))|((first==34)&(third==48))|((first==40)&(third==47))|((first==46)&(third==41))|((first==42)&(third==43))|((first==48)&(third==37))|((first==38)&(third==45))|((first==44)&(third==39)))
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line(); 
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		forward_mm(40);
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		left_degrees(40);
		prev_angle=40;
		adjust_left();
		stop();
		_delay_ms(100);
	}
	else
	if(((first==3)&(third==27))|((first==11)&(third==31))|((first==19)&(third==35))|((first==36)&(third==38))|((first==48)&(third==41))|((first==46)&(third==39))|((first==44)&(third==37))|((first==32)&(third==42))|((first==28)&(third==40))|((first==26)&(third==3))|((first==30)&(third==11))|((first==34)&(third==19)))
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line(); 
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		forward_mm(50);
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		left_degrees(90);
		prev_angle=100;
		adjust_left();
		stop();
		_delay_ms(100);
	}
	else
	if ((first<25) & (second==(first+1)))
	{
		if(dircount%2==1)
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			right_degrees(140);
			adjust_right();
			stop();
			_delay_ms(100);
			dircount=dircount+1;
		}		
		if(((third)==(first+2))|(third<second))
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line();
			_delay_ms(100);
			prev_angle=0;

		}
		else
		if((third==27)|(third==29)|(third==31)|(third==33)|(third==35)|(third==25))
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line();
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			forward_mm(40);
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			right_degrees(40);
			prev_angle=-60;
			adjust_right();
			stop();
			_delay_ms(100);
		}
		else
		if((third==26)|(third==28)|(third==30)|(third==32)|(third==34)|(third==36))
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line();
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			forward_mm(40);
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			right_degrees(100);
			prev_angle=-100;
			adjust_right();
			stop();
			_delay_ms(100);
		}
		else
		if ((first==23) & (second==24))
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line();
			prev_angle=0;
			_delay_ms(100);
		}
	}
	else		
		if ((first==24) & (second==1))
		{
			if(dircount%2==1)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				right_degrees(140);
				adjust_right();
				stop();
				_delay_ms(100);
				dircount=dircount+1;
			}
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line();
			_delay_ms(100);
			prev_angle=0;
		}
	else
	if ((first<25) & (second==(first-1)))
	{
		if(dircount%2==0)
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			left_degrees(140);
			adjust_left();
			stop();
			_delay_ms(100);
			dircount=dircount+1;
		}			
		if((third==(first-2))|((third>second)&(third<25)))
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line();
			_delay_ms(100);
			prev_angle=0;
		}			
		else
		if((third==27)|(third==29)|(third==31)|(third==33)|(third==35)|(third==25))
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line(); 
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			forward_mm(40);
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			left_degrees(100);
			prev_angle=100;
			adjust_left();
			stop();
			_delay_ms(100);
		}
		else
		if((third==26)|(third==28)|(third==30)|(third==32)|(third==34)|(third==36))
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line(); 
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			forward_mm(40);
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			left_degrees(50);
			prev_angle=60;
			adjust_left();
			stop();
			_delay_ms(100);
		}
		else
		if ((first==2) & (second==1))
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			follow_line(); 
			prev_angle=0;
			_delay_ms(100);
		}
	}
	else
	if ((first==1) & (second==24))
	{
		if(dircount%2==0)
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			left_degrees(140);
			adjust_left();
			stop();
			_delay_ms(100);
			dircount=dircount+1;
		}
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line(); 
		_delay_ms(100);
		prev_angle=0;
	}			
}

void strike(int second,int third,int mnp_strike)
{
	if((third<25)&(second<25))
	{
		if(mnp_strike<25)
		{
			if(dircount%2==0)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				left_degrees(90);
				right_degrees(90);
			}
			else
			if(dircount%2==1)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				right_degrees(90);
				left_degrees(90);
			}
		}
		else
		if((third==1)|(third==5)|(third==9)|(third==13)|(third==17)|(third==21))
		{
			if (dircount%2==0)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				right_degrees(90);
				left_degrees(90);
			}
			else
			if (dircount%2==1)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				left_degrees(90);
				right_degrees(90);
			}
		}
		else
		if((third==2)|(third==6)|(third==10)|(third==14)|(third==18)|(third==22))
		{
			if (dircount%2==0)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				right_degrees(120);
				left_degrees(120);
			}
			else
			if (dircount%2==1)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				left_degrees(60);
				right_degrees(60);
			}
		}
		else
		if((third==4)|(third==8)|(third==12)|(third==16)|(third==20)|(third==24))
		{
			if (dircount%2==0)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				right_degrees(60);
				left_degrees(60);
			}
			else
			if (dircount%2==1)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				left_degrees(120);
				right_degrees(120);
			}
		}
	}
	else
	if (third<25)
	{
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
	}
	else
	if(((second==39)&(third==40))|((second==41)&(third==42))|((second==38)&(third==37))|((second==40)&(third==39))|((second==37)&(third==38))|((second==42)&(third==41)))
	{
		if(third%2==0)
		{
			if((mnp_strike==25)|(mnp_strike==28)|(mnp_strike==31))
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				left_degrees(60);
				right_degrees(60);
			}
			if((mnp_strike==26)|(mnp_strike==29)|(mnp_strike==32))
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				right_degrees(60);
				left_degrees(60);
			}
		}
		else
		if(third%2==1)
		{
			if((mnp_strike==26)|(mnp_strike==29)|(mnp_strike==32))
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				left_degrees(60);
				right_degrees(60);
			}
			if((mnp_strike==25)|(mnp_strike==28)|(mnp_strike==31))
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				right_degrees(60);
				left_degrees(60);
			}
		}
	}
	else
	if(((second==42)&(third==34))|((second==39)&(third==27))|((second==38)&(third==26))|((second==40)&(third==30))|((second==41)&(third==31))|((second==37)&(third==35))|((second==27)&(third==39))|((second==26)&(third==38))|((second==30)&(third==40))|((second==31)&(third==41))|((second==35)&(third==37))|((second==34)&(third==42)))
	{
		if(third<37)
		{
			if(((third%2==0)&(second>36))|((third%2==1)&(second<25)))
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				right_degrees(120);
				left_degrees(120);
			}
			else
			if(((third%2==0)&(second<25))|((third%2==1)&(second>36)))
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				left_degrees(120);
				right_degrees(120);
			}
		}
		else
		if(third>=37)
		{
			if((mnp_strike==32)|(mnp_strike==26)|(mnp_strike==29))
			{
				if (third%2==0)
				{
					node_count=0;
					ShaftCountRight = 0;
					ShaftCountLeft = 0;
					left_degrees(120);
					right_degrees(120);
				}
				else
				if(third%2==1)
				{
					node_count=0;
					ShaftCountRight = 0;
					ShaftCountLeft = 0;
					right_degrees(120);
					left_degrees(120);
				}
			}
			else
			if((mnp_strike==33)|(mnp_strike==25)|(mnp_strike==27)|(mnp_strike==31)|(mnp_strike==30)|(mnp_strike==31))
			{
				if (third%2==0)
				{
					node_count=0;
					ShaftCountRight = 0;
					ShaftCountLeft = 0;
					right_degrees(120);
					left_degrees(120);
				}
				else
				if(third%2==1)
				{
					node_count=0;
					ShaftCountRight = 0;
					ShaftCountLeft = 0;
					left_degrees(120);
					right_degrees(120);
				}
			}
		}
	}
	else
	if(((second==28)&(third==39))|((second==29)&(third==40))|((second==32)&(third==41))|((second==33)&(third==42))||((second==36)&(third==37))|((second==25)&(third==38)))
	{
		if(third%2==0)
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			left_degrees(120);
			right_degrees(120);
		}
		else
		if(third%2==1)
		{
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			right_degrees(120);
			left_degrees(120);
		}
	}
	else
	if((second>42)&(third<43))
	{
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
	}
}

void transit()
{
	for(z=0;z<num1-1;z++)
	{	
		txbyte(tx_flag);
		_delay_ms(100);
		
		rbt2_tens=path1[z+2]/10;
		txbyte(rbt2_tens+48);
		_delay_ms(100);
		
		txbyte(path1[z+2]%10+48);
		_delay_ms(100);
		
		if(tx_flag==0x31)
		{tx_flag=0x30;}
		if(z==0)
		{
			prev_obs=1;
		}
		else
		{
			prev_obs=path1[z-1];
		}
		
		irsense=ADC_Conversion(11);
		
		if((irsense>80)&(graph[path1[z]-1][(path1[z+1]-1)]>0)&(path1[z]!=path1[z+2]))
		{
			
			stop();
			_delay_ms(5000);
			flagir=1;
		}
		if(flagir==1)
		{
			obsadjust(path1[z],path1[z+1]);
			flag_break=1;
			break;
		}
		if((path1[z]>=25)&(path1[z+1]<25)&(path1[z]!=path1[z+2]))
		{
			path_follow_out(path1[z],path1[z+1],path1[z+2]);
		}
		else
		{
		path_follow(path1[z],path1[z+1],path1[z+2]);
		}		
	}
	if(flag_break==0)
	{
		if(path1[z+1]>24)
		{	
			irsense=ADC_Conversion(11);
			if((irsense>80)&(graph[path1[z]-1][(path1[z+1]-1)]>0)&(path1[z]!=path1[z+2]))
			{
				flagir=1;
			}
			if(flagir==1)
			{
				prev_obs=path1[z-1];
				obsadjust(path1[z],path1[z+1]);
				flag_break=1;
			}
			else
			if(flagir==0)
			{
				node_count=0;
				ShaftCountRight = 0;
				ShaftCountLeft = 0;
				follow_line();
			}				
			

		}
		else
		{
			if((path1[z+1]>path1[z])|(path1[z+1]==1))
			{
				if(dircount%2==1)
				{
					node_count=0;
					ShaftCountRight = 0;
					ShaftCountLeft = 0;
					adjust_right();
					right_degrees(140);
					stop();
					dircount=dircount+1;
				}
				irsense=ADC_Conversion(11);
				
				if((irsense>80)&(graph[path1[z]-1][(path1[z+1]-1)]>0)&(path1[z]!=path1[z+2]))
				{
					flagir=1;
				}
				if(flagir==1)
				{
					prev_obs=path1[z-1];
					obsadjust(path1[z],path1[z+1]);
					flag_break=1;
				}
				else
				if(flagir==0)
				{
					node_count=0;
					ShaftCountRight = 0;
					ShaftCountLeft = 0;
					follow_line();
				}
			}
			else
			if(((path1[z+1]<path1[z])&(path1[z]<25))|((path1[z+1]==24)&(path1[z]==1)))
			{
				if(dircount%2==0)
				{
					node_count=0;
					ShaftCountRight = 0;
					ShaftCountLeft = 0;
					left_degrees(140);
					adjust_left();
					stop();
					dircount=dircount+1;
				}
				irsense=ADC_Conversion(11);
				
				if((irsense>80)&(graph[path1[z]-1][(path1[z+1]-1)]>0)&(path1[z]!=path1[z+2]))
				{
					flagir=1;
				}
				if(flagir==1)
				{
					prev_obs=path1[z-1];
					obsadjust(path1[z],path1[z+1]);
					flag_break=1;
				}
				else
				if(flagir==0)
				{
					node_count=0;
					ShaftCountRight = 0;
					ShaftCountLeft = 0;
					follow_line();
				}
			}
		}
		
		txbyte(tx_flag);
		_delay_ms(100);
		
		rbt2_tens=path1[z+1]/10;
		txbyte(rbt2_tens+48);
		_delay_ms(100);
		
		txbyte(path1[z+1]%10+48);
		_delay_ms(100);
		
		
		while(!(flag_isr));
		strike(path1[z],path1[z+1],mnp_robot1[go_mnp]);
		flag_isr=0x30;
		tx_flag=0x31;
	}
	flag_break=0;
}

// Funtion that implements Dijkstra's single source shortest path
// algorithm for a graph represented using adjacency matrix
// representation

void dijkstra1(unsigned int graph[V][V], int src, int dest)
{
	
	int dist[V];  // The output array. dist[i] will hold
	// the shortest distance from src to i

	// sptSet[i] will true if vertex i is included / in shortest
	// path tree or shortest distance from src to i is finalized
	bool sptSet[V];

	// Parent array to store shortest path tree
	int parent[V];

	// Initialize all distances as INFINITE and stpSet[] as false
	int i;
	for (i = 0; i < V; i++)
	{
		//parent[0] = -1;
		parent[src]=-1;
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}

	// Distance of source vertex from itself is always 0
	dist[src] = 0;

	// Find shortest path for all vertices
	int count;
	for (count = 1; count < V; count++)
	{
		// Pick the minimum distance vertex from the set of
		// vertices not yet processed. u is always equal to src
		// in first iteration.
		int u = minDistance(dist, sptSet);

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the
		// picked vertex.
		int v;
		for (v = 0; v < V; v++)
		{
			// Update dist[v] only if is not in sptSet, there is
			// an edge from u to v, and total weight of path from
			// src to v through u is smaller than current value of
			// dist[v]
			if (!sptSet[v] && graph[u][v] &&
			dist[u] + graph[u][v] < dist[v])
			{
				parent[v]  = u;//parent is subscript
				dist[v] = dist[u] + graph[u][v];
			}
		}
		if(sptSet[dest])
		{
			break;
		}
	}

	nodes1=dist[dest];
	num1=dist[dest];
	retrdist=dist[dest];
	len=dist[dest];
}

void node_decide(int source, int next)
{
	min_dis=99;
	if (next<25)
	{
		shortest_new=next-1;
		dijkstra1(graph,source,shortest_new);	
	}
	else
	{
		for(valid=0;valid<6;valid++)
		{
			if(mnp[next-25][valid]>=0)
			{
				dijkstra1(graph,source,mnp[next-25][valid]);
				distance=retrdist;
				if(distance<min_dis)
				{
					min_dis=distance;
					shortest=mnp[next-25][valid];
				}
			}
		}
		shortest_new=shortest;
		dijkstra1(graph,source,shortest);
	}	
}
void arrayadjust(unsigned int path1[])
{
	num1=num1+1;
	int adjust;
	for(adjust=num1;adjust>0;adjust--)
	{
		path1[adjust]=path1[adjust-1];
	}
	path1[adjust]=store;
	
}
void obsadjust(int first,int second)
{
	flagir=0;
	graph[second-1][first-1]=0;
	graph[first-1][second-1]=0;
	if(prev_angle>0)
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		right_degrees(prev_angle);
		back_mm(120);
		begin=first-1;
		store=prev_obs;
		node_decide(begin,robot2_mnp[go_mnp]);
		arrayadjust(path1);
		transit();		
	}
	if(prev_angle==0)
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		back_mm(50);
		begin=first-1;
		store=prev_obs;
		
		node_decide(begin,robot2_mnp[go_mnp]);
		arrayadjust(path1);
		
		transit();
	}
	if(prev_angle<0)
	{
		prev_angle=-prev_angle;
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		left_degrees(prev_angle);
		//back_mm(30);
		back_mm(200);
		begin=first-1;
		store=prev_obs;
		node_decide(begin,robot2_mnp[go_mnp]);
		arrayadjust(path1);		
		transit();
	}
}

//Main Function
int main()
{int aar;
	init_devices();
	lcd_set_4bit();
	lcd_init();
	graph_init();
	
	turn_on_sharp234_wl();
	while(1)
	{		
		
		if(count_isr>1)	
		{
			rbt2_length=((rbt2_len[0]-48)*10+(rbt2_len[1]-48));			
			break;
		}		
	}
	
	while(1)
	{
		if (digit_index==rbt2_length*2)
		{
			for (digit_index=0;digit_index<(rbt2_length*2-1);digit_index=digit_index+2)
			{
				robot2_mnp[nai]=((digit[digit_index]-48)*10+(digit[digit_index+1]-48));
				nai++;
			}
			break;
		}			
	}
	
	
	begin=12;
	mnplength=rbt2_length;
	if(mnplength%2==0)
		iteration=mnplength/2;
	else
	if(mnplength%2==1)
		iteration=mnplength/2+1;
	node_decide(begin,robot2_mnp[0]);

	for (mnpno=0;mnpno<iteration;mnpno++)
	{
		if(mnpno!=0)
		{
			go_mnp=mnpno*2;
			irsense=ADC_Conversion(11);
			if((irsense>80)&((graph[path1[z+1]-1][path1[z+1]]>0)|(graph[23][0]>0)|(graph[path1[z+1]-1][path1[z+1]-2]>0)))
			{
				if((path1[z+1]==24)|(path1[z+1]==1))
				{
					graph[23][0]=0;
					graph[0][23]=0;
				}
				else
				if(path1[z+1]>path1[z])
				{
					graph[path1[z+1]-1][path1[z+1]]=0;
					graph[path1[z+1]][path1[z+1]-1]=0;
				}
				else
				if(path1[z+1]<path1[z])
				{
					graph[path1[z+1]-1][path1[z+1]-2]=0;
					graph[path1[z+1]-2][path1[z+1]-1]=0;
				}
				back_mm(50);
				begin=path1[z+1]-1;	//check
				store=path1[z];		//check
				node_decide(begin,robot2_mnp[go_mnp]);
				arrayadjust(path1);
			}
			else
			if(irsense<90)
			{
				back_mm(50);
				begin=path1[z+1]-1; //check
				store=path1[z];//check
				node_decide(begin,robot2_mnp[go_mnp]);
				arrayadjust(path1);
			}		
		}
		transit();
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		_delay_ms(500);
		
		if((mnplength%2==1)&(mnpno==mnplength/2))
		break;
		irsense=ADC_Conversion(11);
		if((irsense>80)&((graph[path1[z+1]-1][path1[z+1]]>0)|(graph[23][0]>0)|(graph[path1[z+1]-1][path1[z+1]-2]>0)))
		{
			if((path1[z+1]==24)|(path1[z+1]==1))
			{
				graph[23][0]=0;
				graph[0][23]=0;
			}
			else
			if(path1[z+1]>path1[z])
			{
				graph[path1[z+1]-1][path1[z+1]]=0;
				graph[path1[z+1]][path1[z+1]-1]=0;				
			}
			else
			if(path1[z+1]<path1[z])
			{
				graph[path1[z+1]-1][path1[z+1]-2]=0;
				graph[path1[z+1]-2][path1[z+1]-1]=0;
			}
			
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			begin=path1[z+1]-1;
			store=path1[z];
			go_mnp=mnpno*2+1;
			back_mm(50);
			node_decide(begin,robot2_mnp[go_mnp]);
			arrayadjust(path1);
		}
		else
		if(irsense<90)
		{
			back_mm(50);
			node_count=0;
			ShaftCountRight = 0;
			ShaftCountLeft = 0;
			begin=path1[z+1]-1;
			store=path1[z];		
			go_mnp=mnpno*2+1;
			node_decide(begin,robot2_mnp[go_mnp]);
			arrayadjust(path1);
			
		}	
		transit();	
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		_delay_ms(500);
		if(mnpno==iteration-1)
			break;
		
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		begin=path1[z+1]-1;
		store=path1[z];
	}
	stop();
	txbyte(0x31);
}