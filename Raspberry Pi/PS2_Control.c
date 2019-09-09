/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         PS2_control.c
* @author       Danny
* @version      V1.0
* @date         2017.08.16
* @brief        PS2 control car
* @details
* @par History  
*
*/
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <wiringPiSPI.h>

#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

#define PS2_DAT_PIN   12//MOS
#define PS2_CMD_PIN   13//MIS
#define PS2_SEL_PIN   6 //CS
#define PS2_CLK_PIN   14//SCK

#define PSS_RX 5        
#define PSS_RY 6        
#define PSS_LX 7        
#define PSS_LY 8        

enum {
  enSTOP = 1,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT,
  enUPLEFT,
  enUPRIGHT,
  enDOWNLEFT,
  enDOWNRIGHT
}enCarState;

//Definition of Pin
int Left_motor_go = 28;       //AIN2 connects to wiringPi port 28 of Raspberry pi for control Left motor forward 
int Left_motor_back = 29;     //AIN1 connects to wiringPi port 29 of Raspberry pi for control Left motor back 

int Right_motor_go = 24;      //BIN2 connects to wiringPi port 24 of Raspberry pi for control Left motor forward 
int Right_motor_back = 25;    //BIN1 connects to wiringPi port 25 of Raspberry pi for control Left motor back

int Left_motor_pwm = 27;      //PWMA connects to wiringPi port 27 of Raspberry pi for control the speed of the left motor
int Right_motor_pwm = 23;     //PWMA connects to wiringPi port 23 of Raspberry pi for control the speed of the right motor

int buzzer = 10;              //buzzer is connected to  wiringPi port 10 of Raspberry pi

int CarSpeedControl = 150;

int ServoPin = 4;

int g_CarState = enSTOP;  
int g_ServoState = 0;     

int LED_R = 3;           //LED_R is connected to  wiringPi port 3 of Raspberry pi
int LED_G = 2;           //LED_G is connected to  wiringPi port 2 of Raspberry pi
int LED_B = 5;           //LED_B is connected to  wiringPi port 5 of Raspberry pi

unsigned char PS2_KEY;
unsigned char X1,Y1,X2,Y2; 
 
unsigned short MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_TRIANGLE,
    PSB_CIRCLE,
    PSB_CROSS,
    PSB_SQUARE
	};	
	
unsigned short  Handkey;
unsigned char scan[9]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.08.16
* @brief         Define a pulse function to generate the PWM value in the analog mode. 
*                The base pulse is 20ms, 
*                and the high level of the pulse is controlled at 0-180 degrees in 0.5-2.5ms.
* @param[in1]    ServPin
* @param[in2]    myangle
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;                    //Define the pulse width variable
  PulseWidth = (myangle * 11) + 500; //Convert the Angle to 500-2480 pulse width
  digitalWrite(ServoPin, HIGH);     
  delayMicroseconds(PulseWidth);    
  digitalWrite(ServoPin, LOW);       
  delay(20 - PulseWidth / 1000);     //Delay remaining time 
  return;
}

/**
* Function       run
* @author        Danny
* @date          2017.08.16
* @brief         run
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void run()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, HIGH); 
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       brake
* @author        Danny
* @date          2017.08.16
* @brief         brake
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void brake()
{
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
}

/**
* Function       left
* @author        Danny
* @date          2017.08.16
* @brief         turn left(left wheel stop, right wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void left()
{
  //Left motor stop
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, 0);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.08.16
* @brief         turn left in place(left wheel back, right wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_left()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       upleft
* @author        Danny
* @date          2017.08.16
* @brief         The car moves along the left front wheel (the left wheel advances, the right wheel advances, the difference between the two)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void upleft()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, CarSpeedControl - 30);     //The left motor speed is set to 120 (0-255)

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl + 30);   //The right motor speed is set to 180 (0-255)
}

/**
* Function       downleft
* @author        Danny
* @date          2017.08.16
* @brief         The car retreats along the left rear (the left wheel retreats, the right wheel retreats, the difference between the two)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void downleft()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);        
  digitalWrite(Left_motor_back, HIGH);      
  softPwmWrite(Left_motor_pwm, CarSpeedControl - 30); //The left motor speed is set to 120 (0-255)

  //Right motor back
  digitalWrite(Right_motor_go, LOW);        
  digitalWrite(Right_motor_back, HIGH);     
  softPwmWrite(Right_motor_pwm, CarSpeedControl + 30);//The right motor speed is set to 180 (0-255)
}

/**
* Function       upright
* @author        Danny
* @date          2017.08.16
* @brief         The car moves along the right front wheel (the left wheel advances, the right wheel advances, the difference between the two)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void upright()
{
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, CarSpeedControl + 30);     //The left motor speed is set to 180 (0-255)

  digitalWrite(Right_motor_go, HIGH);   
  digitalWrite(Right_motor_back, LOW);  
  softPwmWrite(Right_motor_pwm, CarSpeedControl - 30);    //The right motor speed is set to 180 (0-255)
}

/**
* Function       downright
* @author        Danny
* @date          2017.08.16
* @brief         The car retreats along the right rear (the left wheel retreats, the right wheel retreats, the difference between the two)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void downright()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl + 30);     //The left motor speed is set to 180 (0-255)

  //Right motor back
  digitalWrite(Right_motor_go, LOW);   
  digitalWrite(Right_motor_back, HIGH);
  softPwmWrite(Right_motor_pwm, CarSpeedControl - 30);   //The right motor speed is set to 120 (0-255)
}

/**
* Function       right
* @author        Danny
* @date          2017.08.16
* @brief         turn right(left wheel dvance,right wheel stop)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void right()
{
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, LOW);  
  softPwmWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.08.16
* @brief         turn right in place(left wheel dvance,right wheel back)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_right()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);   
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       back
* @author        Danny
* @date          2017.08.16
* @brief         back
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void back()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);    
  digitalWrite(Left_motor_back, HIGH);  
  softPwmWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  softPwmWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       whistle
* @author        Danny
* @date          2017.08.16
* @brief         whistle
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void whistle()
{
  digitalWrite(buzzer, LOW);   //sound
  delay(100);                  
  digitalWrite(buzzer, HIGH);  //no sound
  delay(1);                    

  digitalWrite(buzzer, LOW);   //sound
  delay(200);                  
  digitalWrite(buzzer, HIGH);  //no sound
  delay(2);                   
  return;
}

/**
* Function       servo_appointed_detection
* @author        Danny
* @date          2017.08.16
* @brief         The servo rotates to the specified angle
* @param[in]     pos
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 15; i++)    //Generate PWM, equivalent delay to ensure that it can be turned to the response angle
  {
    servo_pulse(ServoPin, pos); //Generate the PWM in the analog mode
  }
}

/**
* Function       color_led_pwm
* @author        Danny
* @date          2017.08.16
* @brief         The colorful light is in the designated color
* @param[in1]    v_iRed:（0-255）
* @param[in2]    v_iGreen:（0-255）
* @param[in3]    v_iBlue:（0-255）
* @param[out]    void
* @retval        void
* @par History   
*/
void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
  softPwmWrite(LED_R, v_iRed);
  softPwmWrite(LED_G, v_iGreen);
  softPwmWrite(LED_B, v_iBlue);
  return;
}

/**
* Function       PS2_Init
* @author        Danny
* @date          2017.08.16
* @brief         PS2_Init
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void PS2_Init()
{
	wiringPiSPISetup(0,500000);
	pinMode(PS2_CMD_PIN, OUTPUT);
	pinMode(PS2_CLK_PIN, OUTPUT);
	pinMode(PS2_DAT_PIN, INPUT);
	pinMode(PS2_SEL_PIN, OUTPUT);
	
	digitalWrite(PS2_CLK_PIN,HIGH);
	digitalWrite(PS2_SEL_PIN,HIGH);
	digitalWrite(PS2_CMD_PIN,HIGH);
}

/**
* Function       PS2_AnologData
* @author        Danny
* @date          2017.08.16
* @brief         Read the analog value of the PS2 joystick
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
unsigned char PS2_AnologData(unsigned char button)
{
	return Data[button];
}

/**
* Function       PS2_ClearData
* @author        Danny
* @date          2017.08.16
* @brief         PS2_ClearData
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void PS2_ClearData()
{
	memset(Data, 0 ,sizeof(Data)/sizeof(Data[0]));
	return;
}

/**
* Function       PS2_ReadData
* @author        Danny
* @date          2017.08.16
* @brief         Raed data of PS2
* @param[in]     command:
* @param[out]    void
* @retval        void
* @par History   
*/
unsigned char PS2_ReadData(unsigned char command)
{
	unsigned char i,j = 1;
	unsigned char res = 0; 
    for(i=0; i<=7; i++)          
    {
		if(command&0x01)
		digitalWrite(PS2_CMD_PIN,HIGH);
		else
		digitalWrite(PS2_CMD_PIN,LOW);
		command = command >> 1;
		delayMicroseconds(10);
		digitalWrite(PS2_CLK_PIN,LOW);
		delayMicroseconds(10);
		if(digitalRead(PS2_DAT_PIN) == HIGH) 
			res = res + j;
		j = j << 1; 
		digitalWrite(PS2_CLK_PIN,HIGH);
		delayMicroseconds(10);	 
    }
    digitalWrite(PS2_CMD_PIN,HIGH);
	delayMicroseconds(50);
 //   printf("res:%d\n",res);	
    return res;	
}

/**
* Function       PS2_DataKey
* @author        Danny
* @date          2017.08.16
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
//Process the data for the PS2 that is read
unsigned char PS2_DataKey()
{
	unsigned char index = 0, i = 0;
   	PS2_ClearData();
	digitalWrite(PS2_SEL_PIN,LOW);
	for(i=0;i<9;i++)	
	{
		Data[i] = PS2_ReadData(scan[i]);
//		printf("data[%d]:%d\n",i,Data[i]);	
	} 
	digitalWrite(PS2_SEL_PIN,HIGH);
	Handkey=(Data[4]<<8)|Data[3];     
	for(index=0;index<16;index++)
	{	 
		if((Handkey&(1<<(MASK[index]-1)))==0)
		{
			return index+1;
		}
//		printf("index:%d\n",index+1);
	}
	return 0;   
}

/**
* Function       main
* @author        Danny
* @date          2017.08.16
* @brief         Parsing the instructions sent by the handle and executing the corresponding instructions
* @param[in]     void
* @retval        void
* @par History   
*/
void main()
{
	wiringPiSetup();

    PS2_Init();
	
    //Initialize motor drive IO as output mode
    pinMode(Left_motor_go, OUTPUT);
    pinMode(Left_motor_back, OUTPUT);
    pinMode(Right_motor_go, OUTPUT);
    pinMode(Right_motor_back, OUTPUT);

    softPwmCreate(Left_motor_pwm,0,255); 
    softPwmCreate(Right_motor_pwm,0,255);
  
    //Initialize buzzer IO as output mode
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, HIGH);
 
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

    softPwmCreate(LED_R,0,255); 
    softPwmCreate(LED_G,0,255); 
    softPwmCreate(LED_B,0,255);
  
    pinMode(ServoPin, OUTPUT);
	
    while(1)
    {	   
      int flag = 0;
      PS2_KEY = PS2_DataKey();	 //PS2 handle button capture processing
	  switch(PS2_KEY)
	  {
	    case PSB_SELECT: 	puts("PSB_SELECT");  break;

	    case PSB_L3:     	g_CarState = enSTOP;  puts("PSB_L3");  break; 
		
	    case PSB_R3:     	servo_appointed_detection(90);	puts("PSB_R3");  break; 
		
	    case PSB_START:  	puts("PSB_START");  break;  

	    case PSB_PAD_UP: 	g_CarState = enRUN;   puts("PSB_PAD_UP");  break; 
	
	    case PSB_PAD_RIGHT:	g_CarState = enRIGHT; puts("PSB_PAD_RIGHT");  break;

	    case PSB_PAD_DOWN:	g_CarState = enBACK;  puts("PSB_PAD_DOWN");  break;
		
	    case PSB_PAD_LEFT:	g_CarState = enLEFT;  puts("PSB_PAD_LEFT");  break; 

	    case PSB_L2:      	CarSpeedControl += 50;
                            if (CarSpeedControl > 255)
                            {
                              CarSpeedControl = 255;
                            }
						    puts("PSB_L2");  break; 

	    case PSB_R2:      	CarSpeedControl -= 50;
                            if (CarSpeedControl < 50)
                            {
                              CarSpeedControl = 100;
                            }
						    puts("PSB_R2");  break; 

	    case PSB_L1:      	puts("PSB_L1");  break; 

	    case PSB_R1:      	puts("PSB_R1");  break; 
		
	    case PSB_TRIANGLE:	color_led_pwm(0, 255, 0);
                            delay(100);
						    puts("PSB_TRIANGLE");  break; 

	    case PSB_CIRCLE:  	color_led_pwm(0, 0, 255);
                            delay(100);
					        puts("PSB_CIRCLE");  break; 
	    

	    case PSB_SQUARE:  	color_led_pwm(255, 0, 0);
                            delay(100);
						    puts("PSB_SQUARE");  break;

		case PSB_CROSS:		whistle();
							color_led_pwm(0, 0, 0);break;
							
	    default: g_CarState = enSTOP; break; 
	   }
      
	  if(PS2_KEY == PSB_L1 || PS2_KEY == PSB_R1)
	  {
		X1 = PS2_AnologData(PSS_LX);
		Y1 = PS2_AnologData(PSS_LY);
		X2 = PS2_AnologData(PSS_RX);
		Y2 = PS2_AnologData(PSS_RY);
				
	    if (Y1 < 5 && X1 > 80 && X1 < 180) 
		{
		    g_CarState = enRUN;
		}
		else if (Y1 > 230 && X1 > 80 && X1 < 180) 
	    {
			g_CarState = enBACK;			  
		}
		else if (X1 < 5 && Y1 > 80 && Y1 < 180) 
		{
			g_CarState = enLEFT;  
		}
	    else if (Y1 > 80 && Y1 < 180 && X1 > 230)
		{
			g_CarState = enRIGHT;   
	    }
	    else if (Y1 <= 80 && X1 <= 80) 
		{
		    g_CarState = enUPLEFT;  
		}
		else if (Y1 <= 80 && X1 >= 180) 
		{
			g_CarState = enUPRIGHT;
		}
	    else if (X1 <= 80 && Y1 >= 180) 
		{
			g_CarState = enDOWNLEFT;	
		}
	    else if (Y1 >= 180 && X1 >= 180) 
		{
			g_CarState = enDOWNRIGHT;			  
		}
	    else
		{
			g_CarState = enSTOP;     
	    }

	    if (X2 < 5 && Y2 > 110 && Y2 < 150) 
	    {
		    g_ServoState = 1;        
		}
		else if (Y2 > 110 && Y2 < 150 && X2 > 230)
		{
		    g_ServoState = 2;        
		}
		else
		{
		    g_ServoState = 0;        
		}
	  }
	
     switch (g_ServoState)
     {
       case 0: if (flag != 0) {
               flag = 0;
               servo_appointed_detection(90);
               } break;
       case 1: if (flag != 1) {
               flag = 1;
               servo_appointed_detection(180);
               } break;
       case 2: if (flag != 2) {
               flag = 2;
               servo_appointed_detection(0);
               } break;
       default: break;
     }
	 
     switch (g_CarState)
     {
      case enSTOP: brake(); break;
      case enRUN: run(); break;
      case enLEFT: left(); break;
      case enRIGHT: right(); break;
      case enBACK: back(); break;
      case enTLEFT: spin_left(); break;
      case enTRIGHT: spin_right(); break;
	  case enUPLEFT: upleft();break;
	  case enUPRIGHT: upright();break;
	  case enDOWNLEFT:downleft();break;
	  case enDOWNRIGHT:downright();break;
      default: brake(); break;
     }
	 
     //The following delays are necessary, mainly to avoid frequent restarts caused by too frequent send handle commands.
     delay(50);	
   }
} 
