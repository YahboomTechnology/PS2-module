
#include <PS2X_lib.h>  //The PS2 library file needs to be added to the Arduino IDE library file first

/*PS2 pin definition*/
#define PS2_DAT_PIN        A3  //MOS
#define PS2_CMD_PIN        A2  //MIS
#define PS2_SEL_PIN        A4  //CS
#define PS2_CLK_PIN        A1  //SCK

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   true  
#define rumble      true  

PS2X ps2x;                

int error = 0;
byte type = 0;
byte vibrate = 0;

/*Car running status enumeration*/
enum {
  enSTOP = 1,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enUPLEFT,
  enUPRIGHT,
  enDOWNLEFT,
  enDOWNRIGHT
} enCarState;

/*Motor pin definition*/
int Left_motor_go = 8;    //Left motor forward(AIN1)
int Left_motor_back = 7;  //Left motor back(AIN2)
int Right_motor_go = 2;   //Right motor forward(BIN1)
int Right_motor_back = 4; //Right motor back(BIN2)
int Left_motor_pwm = 6;  
int Right_motor_pwm = 5;  

int buzzer = A0;          //Buzzer connect to Pin_A0 of Arduino UNO

int CarSpeedControl = 150;

int g_CarState = enSTOP;  
int g_ServoState = 0;     

int ServoPin = 3;

int LED_R = 11;           
int LED_G = 10;           
int LED_B = 9;           

/**
* Function       setup
* @author        Danny
* @date          2017.07.25
* @brief         Initialization configure
* @param[in]     void
* @retval        void
* @par History   
*/
void setup()
{
  Serial.begin(9600);

  //Initialize motor drive IO as output mode
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);

  //Initialize buzzer drive IO as output mode
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);

  pinMode(ServoPin, OUTPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);

  delay(300);
 /*Initialize PS2*/
  error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, pressures, rumble);
  if (error == 0)
  {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures)
    {
      Serial.println("true ");
    }
    else
    {
      Serial.println("false");
    }
    Serial.print("rumble = ");
    if (rumble)
    {
      Serial.println("true)");
    }
    else
    {
      Serial.println("false");
    }
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
  }
}

/**
* Function       run
* @author        Danny
* @date          2017.07.25
* @brief         run
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void run()
{
  digitalWrite(Left_motor_go, HIGH);  
  digitalWrite(Left_motor_back, LOW);  
  analogWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       brake
* @author        Danny
* @date          2017.07.25
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
* @date          2017.07.25
* @brief         turn left(left wheel stop,right wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void left()
{
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, LOW);   
  analogWrite(Left_motor_pwm, 0);

  digitalWrite(Right_motor_go, HIGH); 
  digitalWrite(Right_motor_back, LOW); 
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       upleft
* @author        Danny
* @date          2017.07.25
* @brief         The car moves along the left front  (the left wheel advances, the right wheel advances, the difference between the two)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void upleft()
{
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  analogWrite(Left_motor_pwm, CarSpeedControl - 30);     //The left motor speed is set to 120 (0-255)

  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  analogWrite(Right_motor_pwm, CarSpeedControl + 30);   //The right motor speed is set to 180 (0-255)
}

/**
* Function       downleft
* @author        Danny
* @date          2017.07.25
* @brief         The car moves along the left back (the left wheel back, the right wheel back, the difference between the two)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void downleft()
{
  digitalWrite(Left_motor_go, LOW);         
  digitalWrite(Left_motor_back, HIGH);     
  analogWrite(Left_motor_pwm, CarSpeedControl - 30);//The left motor speed is set to 120 (0-255)

  digitalWrite(Right_motor_go, LOW);        
  digitalWrite(Right_motor_back, HIGH);      
  analogWrite(Right_motor_pwm, CarSpeedControl + 30);//The right motor speed is set to 180 (0-255)
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.07.25
* @brief         turn left in place(left wheel back,right wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_left()
{
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  analogWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       right
* @author        Danny
* @date          2017.07.25
* @brief         right(left wheel advance,right wheel back)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void right()
{
  digitalWrite(Left_motor_go, HIGH);   
  digitalWrite(Left_motor_back, LOW);   
  analogWrite(Left_motor_pwm, CarSpeedControl);     //The left motor speed is set to 200 (0-255)

  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, LOW);  
  analogWrite(Right_motor_pwm, 0);      //The left motor speed is set to 0 (0-255)
}

/**
* Function       upright
* @author        Danny
* @date          2017.07.25
* @brief         The car moves along the upper right (the left wheel advances, the right wheel advances, the two have differential)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void upright()
{
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);  
  analogWrite(Left_motor_pwm, CarSpeedControl + 30);     //The left motor speed is set to 180 (0-255)

  digitalWrite(Right_motor_go, HIGH);   
  digitalWrite(Right_motor_back, LOW);  
  analogWrite(Right_motor_pwm, CarSpeedControl - 30);    //The right motor speed is set to 120 (0-255)
}
/**
* Function       downright
* @author        Danny
* @date          2017.07.25
* @brief         The car backs along the lower right (the left wheel retreats, the right wheel retreats, and the two have a differential)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void downright()
{
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  analogWrite(Left_motor_pwm, CarSpeedControl + 30);     //The left motor speed is set to 180 (0-255)

  digitalWrite(Right_motor_go, LOW);  
  digitalWrite(Right_motor_back, HIGH);
  analogWrite(Right_motor_pwm, CarSpeedControl - 30);   //The right motor speed is set to 120 (0-255)
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.07.25
* @brief         turn right in place(left wheel advance,right wheel back)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_right()
{
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  analogWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       back
* @author        Danny
* @date          2017.07.25
* @brief         back
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void back()
{
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  analogWrite(Left_motor_pwm, CarSpeedControl);

  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH);
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       whistle
* @author        Danny
* @date          2017.07.25
* @brief          whistle
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
}

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.07.26
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
* Function       servo_appointed_detection
* @author        Danny
* @date          2017.07.25
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
* @date          2017.07.25
* @brief         Color_LED light the specified color
* @param[in1]    v_iRed:（0-255）
* @param[in2]    v_iGreen:（0-255）
* @param[in3]    v_iBlue:（0-255）
* @param[out]    void
* @retval        void
* @par History   
*/
void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
  analogWrite(LED_R, v_iRed);
  analogWrite(LED_G, v_iGreen);
  analogWrite(LED_B, v_iBlue);
  return;
}

/**
* Function       PS2_control
* @author        Danny
* @date          2017.07.25
* @brief         PS2 controll Car
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void PS2_control(void)
{
  int X1, Y1, X2, Y2;
  if (error == 1)           //skip loop if no controller found
    return;
  if (type != 1)            //skip loop if no controller found
    return;

  //DualShock Controller
  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

  //Press the START button on the handle
  if (ps2x.Button(PSB_START))        //will be TRUE as long as button is pressed
    Serial.println("Start is being held");

  //Press the SELECT button on the handle
  if (ps2x.Button(PSB_SELECT))
    Serial.println("Select is being held");

  //Press the UP button on the handle
  if (ps2x.Button(PSB_PAD_UP))      //will be TRUE as long as button is pressed
  {
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    g_CarState = enRUN;
  }
  //Press the RIGHT button on the handle
  else if (ps2x.Button(PSB_PAD_RIGHT))
  {
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    g_CarState = enRIGHT;
  }
  //Press the LEFT button on the handle
  else if (ps2x.Button(PSB_PAD_LEFT))
  {
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    g_CarState = enLEFT;
  }
  //Press the DOWN button on the handle
  else if (ps2x.Button(PSB_PAD_DOWN))
  {
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    g_CarState = enBACK;
  }
  else
  {
    g_CarState = enSTOP;
  }

  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
  vibrate = ps2x.Analog(PSAB_CROSS);

  if (ps2x.Button(PSB_L3))
  {
    g_CarState = enSTOP;
    Serial.println("L3 pressed");
  }

  if (ps2x.Button(PSB_R3))
  {
    servo_appointed_detection(90);
    Serial.println("R3 pressed");
  }

  if (ps2x.Button(PSB_L2))
  {
    Serial.println("L2 pressed");
    CarSpeedControl += 50;
    if (CarSpeedControl > 255)
    {
      CarSpeedControl = 255;
    }
  }

  if (ps2x.Button(PSB_R2))
  {
    Serial.println("R2 pressed");
    CarSpeedControl -= 50;
    if (CarSpeedControl < 50)
    {
      CarSpeedControl = 100;
    }
  }

  if (ps2x.Button(PSB_SQUARE))
  {
    Serial.println("Square pressed");
    color_led_pwm(255, 0, 0);
    delay(500);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }

  if (ps2x.Button(PSB_TRIANGLE))
  {
    Serial.println("Triangle pressed");
    color_led_pwm(0, 255, 0);
    delay(500);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }

  if (ps2x.Button(PSB_CIRCLE))
  {
    Serial.println("Circle pressed");
    color_led_pwm(0, 0, 255);
    delay(500);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }

  if (ps2x.NewButtonState(PSB_CROSS))    //will be TRUE if button was JUST pressed OR released
  {
    Serial.println("X just changed");
    whistle();
  }

  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
  {
    //print stick values if either is TRUE
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);

    //读取摇杆的数据
    Y1 = ps2x.Analog(PSS_LY);
    X1 = ps2x.Analog(PSS_LX);
    Y2 = ps2x.Analog(PSS_RY);
    X2 = ps2x.Analog(PSS_RX);

    if (Y1 < 5 && X1 > 80 && X1 < 180)         //UP
    {
      g_CarState = enRUN;
    }
    else if (Y1 > 230 && X1 > 80 && X1 < 180) //DOWN
    {
      g_CarState = enBACK;
    }
    else if (X1 < 5 && Y1 > 80 && Y1 < 180)   //LEFT
    {
      g_CarState = enLEFT;
    }
    else if (Y1 > 80 && Y1 < 180 && X1 > 230) //RIGHT
    {
      g_CarState = enRIGHT;
    }
    else if (Y1 <= 80 && X1 <= 80)           //upper left
    {
      g_CarState = enUPLEFT;
    }
    else if (Y1 <= 80 && X1 >= 180)          //upper right
    {
      g_CarState = enUPRIGHT;
    }
    else if (X1 <= 80 && Y1 >= 180)          //left lower
    {
      g_CarState = enDOWNLEFT;
    }
    else if (Y1 >= 180 && X1 >= 180)        //right lower
    {
      g_CarState = enDOWNRIGHT;
    }
    else                                         //stop
    {
      g_CarState = enSTOP;
    }

    if (X2 < 5 && Y2 > 110 && Y2 < 150)         //left
    {
      g_ServoState = 1;
    }
    else if (Y2 > 110 && Y2 < 150 && X2 > 230) //right
    {
      g_ServoState = 2;
    }
    else                                   
    {
      g_ServoState = 0;
    }
  }
}


/**
* Function       loop
* @author        Danny
* @date          2017.07.25
* @brief         The loop takes the instructions from the handle and executes the corresponding actions
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void loop()
{
  int flag = 0;
  PS2_control();
  switch (g_CarState)
  {
    case enSTOP: brake(); break;
    case enRUN: run();  break;
    case enLEFT: left();  break;
    case enRIGHT: right(); break;
    case enBACK: back(); break;
    case enUPLEFT: upleft();  break;
    case enUPRIGHT: upright(); break;
    case enDOWNLEFT: downleft();  break;
    case enDOWNRIGHT: downright(); break;
    default: break;
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
 //The following delays are necessary, mainly to avoid frequent restarts caused by too frequent send handle commands.
  delay(50);
}
