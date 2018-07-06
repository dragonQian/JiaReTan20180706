/************************************
**Program ID:WT2_XWJ
**Cpu ID: Atmega128A
**Software Ver:0.2
**Hardware Ver:0.2
**program by zhengkai
***lasting Update:20180528
***************************************/
#include <OneWire.h>
#include <DallasTemperature.h>
#include <max6675.h>
//#include <DS3231.h>
#include <Wire.h>
/*
DS3231 Clock;
bool Century=false;
bool h12;
bool PM;
byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;
byte year, month, date, DoW, hour, minute, second;
*/
#define ONE_WIRE_BUS 5    //define 18B20 DP
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// arrays to hold device address
DeviceAddress Device0, Device1, Device2;

#define thermoCS 2         //define thermo cs out pin
#define thermoDO  3        //define thermo data out pin
#define thermoCLK 4        //define thermo clk out pin
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

#define FAN_LED 40    //define FAN_LED
#define ON_OFF_LED 39    //define ON_OFF_LED

#define B24     28    //define 24C(BOTTON)
#define B32     29    //define 32C(BOTTON)
#define B38     30    //define 38C(BOTTON)
#define B43     31    //define 43C(BOTTON)
#define FAN_HIGH 32    //define FAN_HIGH(BOTTON)
#define FAN_LOW  33    //define FAN_LOW(BOTTON)
#define ON_OFF   34    //define ON_OFF(BOTTON)
#define CHG      35    //define CHG(BOTTON)

//#define DS18B20_PIN 5 //define DS18B20 OP
#define BUZZ      6   //define hall in
#define MCU_STATE 7   //define hall in

#define HALL 22    //define hall in
#define PWM  14    //define pwm output
#define MOC1 13    //define moc1 output
#define MOC2 12    //define moc2 output
#define REL1 24    //define REL1 output
#define REL2 25    //define REL2 output

#define dataPin 42    //define 595_data
#define latchPin 43    //define 595_latch
#define clockPin 44    //define 595_clk


boolean L24_onoff = HIGH; //define power ON_OFF_LED state
boolean L32_onoff = LOW; //define power ON_OFF_LED state
boolean L38_onoff = LOW; //define power ON_OFF_LED state
boolean L43_onoff = LOW; //define power ON_OFF_LED state


boolean Pwr_onoff = LOW; //define power ON_OFF_LED state
boolean Speed = HIGH; //define FAN_LED state
int x; //define temp set state
float temp;//define temp real state
int onduty;//define duty state
boolean overtemp ; //define overtemp state

#define MIN_RATE_HEAT 0// rang 0 - 15624
#define MAX_RATE_HEAT 15624 // Did not consider the diff of value like 0 or 15624 //if you don't want to heat ,set the COM1A = 0
 
/*-------------------------------------------------------------*PID_control_begin*-----------------------------------------------------------------*/
//define the struct PID param
typedef struct PID_PARAM_ {
  float kP;
  float kI;
  float kD;
  float Imax;
}heat_PID_param;
//defien the PID struct
typedef struct PID_STRUCT_ {
 float _SetPoint;
 float _LastErr;
 float _PreErr; 
 float _SumErr;
}heat_PID_struct;

//Define a value record the PID parameters;
  
//



/**
* Function description: Initialize the parameters of PID
* 
* 
* CreateDate : 2018/6/29 by qwl
* LastChangeDate: 2018/6/29 by qwl
*
*/

/*
typedef struct PID_PARAM_ {
  float kP;
  float kI;
  float kD;
  float Imax;
}PID_PARAM;

typedef struct PID_ {
  float integrator;// integrator value
  float last_input;// last inpute for derivative
  float lastderivative;// last derivative for low-pass filter
  float output;
  float derivative;
}PID;

*/
/**
*  Function description: Initial the value of PID parameters 
*  Return null;
*  
*/
void PID_Init(PID_PARAM_* PID_param, PID_STRUCT_* PID_struct)
{
  PID_param->kP = 2;
  PID_param->kI = 0.08;
  PID_param->kD = 0.045;
  PID_param->Imax = 20;
  
  PID_struct->_SetPoint = 0;
  PID_struct->_LastErr = 0;
  PID_struct->_PreErr = 0;
  PID_struct->_SumErr = 0;
  return;
}

/**
*  Function description: input the target temperature,return the value of heat duty ratio 
*  input 1:unsigned long 
*  input 2:unsigned long
*  return the value of heat frequence
*/
float PID_Calc(PID_PARAM_* PID_param, PID_STRUCT_* PID_struct, float NowPoint)
{
  float dErr,Err;
  Err = PID_struct->_SetPoint - NowPoint;
  PID_struct->_SumErr += Err;
  dErr = PID_struct->_LastErr - PID_struct->_PreErr;
  
  PID_struct->_PreErr = PID_struct->_LastErr;
  PID_struct->_LastErr = Err;
  
  if((PID_param->kI * PID_struct->_SumErr) > PID_param->Imax)
  {
    PID_struct->_SumErr = PID_param->Imax / PID_param->kI;
  }
  else if ((PID_param->kI * PID_struct->_SumErr) < (-PID_param->Imax))
  {
    PID_struct->_SumErr = (-PID_param->Imax) / PID_param->kI; 
  }
  return (PID_param->kP * Err + PID_param->kI * PID_struct->_SumErr + PID_param->kD * dErr);
}
/*-------------------------------------------------------------*PID_control_end*-----------------------------------------------------------------*/
//

void setup() {
  // put your setup code here, to run once:
  /*
  Wire.begin();
        //以下部分是初始化时间，每次板子通电之后都会初始化成这个时间，只是测试用，以后可以删除。
        Clock.setSecond(50);//Set the second 
        Clock.setMinute(29);//Set the minute 设置分钟
        Clock.setHour(16);  //Set the hour 设置小时
        Clock.setDoW(5);    //Set the day of the week 设置星期几
        Clock.setDate(20);  //Set the date of the month 设置月份
        Clock.setMonth(4);  //Set the month of the year 设置一年中的月份
        Clock.setYear(18);  //Set the year (Last two digits of the year) 设置年份(在今年的最后两位数——比如2013年最后的13)
 // Start the serial interface
 */
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT); //让三个脚都是输出状态

  pinMode(FAN_LED, OUTPUT);
  pinMode(ON_OFF_LED, OUTPUT);

  pinMode(B24, INPUT);
  pinMode(B32, INPUT);
  pinMode(B38, INPUT);
  pinMode(B43, INPUT);

  pinMode(FAN_HIGH, INPUT);
  pinMode(FAN_LOW, INPUT);
  pinMode(ON_OFF, INPUT);
  pinMode(CHG, INPUT);

  pinMode(MCU_STATE, OUTPUT);
  pinMode(BUZZ, OUTPUT);

  pinMode(HALL, INPUT);
  pinMode(PWM,  OUTPUT);
  pinMode(MOC1, OUTPUT);
  pinMode(MOC2, OUTPUT);
  pinMode(REL1, OUTPUT);
  pinMode(REL2, OUTPUT);

  digitalWrite(REL1, LOW);
  digitalWrite(REL2, LOW);
  digitalWrite(MOC1, LOW);
  digitalWrite(MOC2, LOW);

  digitalWrite(ON_OFF_LED, !Pwr_onoff);
  //x = 16;//wt2
  x =224;//wt3
  // start serial port
  // start serial port
  Serial.begin(9600);
  Serial1.begin(9600);
  
  Serial.println("serial0 is begining now");
  //Serial1.print("serial1 is begining now");
  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  if (!sensors.getAddress(Device0, 0)) Serial.println("Unable to find address for Device0");

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(Device0, 11);

  Serial.print("Device0 Resolution: ");
  Serial.print(sensors.getResolution(Device0), DEC);
  Serial.println();

  Serial.print("init L43 is ");
  Serial.println(L43_onoff);
  Serial.print("init L38 is ");
  Serial.println(L38_onoff);
  Serial.print("init L32 is ");
  Serial.println(L32_onoff);
  Serial.print("init L24 is ");
  Serial.println(L24_onoff);
  Serial.print("init overtemp is ");
  Serial.println(overtemp);
  Serial.println("");
  Serial.println("");
  
  //TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10) ; before configuration
  //TCCR1B = _BV(WGM12) | _BV(CS11);
  
  PID_Init(&heat_PID_param, &heat_PID_struct);
  
  TCCR0 =  _BV(WGM01) | _BV(WGM00) | _BV(COM01) | _BV(CS01) | _BV(CS00);  //prescaler 32 top: 256 Fast PWM
  OCR0 = 0;
  TCCR1A = _BV(COM1A1) | _BV(WGM11); //Fast PWM top:ICRn  
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) |  _BV(CS10); //prescaler 1024  
  ICL1 = MIN_RATE_HEAT; // 1hz
  OCR1A = 0;
}
/*
void ReadDS3231()
{
  int second,minute,hour,date,month,year,temperature; 
  second=Clock.getSecond();
  minute=Clock.getMinute();
  hour=Clock.getHour(h12, PM);
  date=Clock.getDate();
  month=Clock.getMonth(Century);
  year=Clock.getYear();
  
  //temperature=Clock.getTemperature();
  
  Serial.print("20");
  Serial.print(year,DEC);
  Serial.print('-');
  Serial.print(month,DEC);
  Serial.print('-');
  Serial.print(date,DEC);
  Serial.print(' ');
  Serial.print(hour,DEC);
  Serial.print(':');
  Serial.print(minute,DEC);
  Serial.print(':');
  Serial.print(second,DEC);
  Serial.print('\n');
  //Serial.print("Temperature=");  //这里是显示温度
  //Serial.print(temperature); 
  Serial.print('\n');
  //delay(1000);
}
*/
//*****************************************temperature_request**********************************************************
void temp_request()
{
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();   //发送温度测量请求命令
  //Serial.print("Temperature for the device 1 (index 0) is: ");
  //Serial.print("DS18B20 is: ");
  //Serial.println(sensors.getTempCByIndex(0));  //获取0号传感器温度数据并发送
  temp = sensors.getTempCByIndex(0);
  //Serial.print("********TEMPERATURE is ");
  Serial.println(temp);
  Serial1.println(temp);
  //Serial.println("********");
  /*
    Serial.print("Thermo is  ");
    Serial.println(thermocouple.readCelsius(), 1);
    //delay(10);
  */
}
//**********************************************************************************************************

//****************************************** buzzing *******************************************************
void buzz()
{
  digitalWrite(BUZZ, HIGH); //buzzing
  delay(25); //然后延时一段时间，
  digitalWrite(BUZZ, LOW); //buzzing
  //delay(10); //然后延时一段时间，
}
//***********************************************************************************************************

//****************************************** mcu_state ******************************************************
void mcu_state()
{
  digitalWrite(MCU_STATE, HIGH);
  delay(25);
  digitalWrite(MCU_STATE, LOW);
  //delay(25);//MCU_STATE
}
//**********************************************************************************************************

//*************************************** on_off ***********************************************************
/* 启动按键控制程序 testing ok*/
void on_off()
{
  if (digitalRead(ON_OFF) == LOW) //由于本例检测低电平
  {
    //Serial.println("Power_Key is online");
    buzz();
    if (digitalRead(ON_OFF) == LOW) //然后检测仍然是低电平，意为按下按键
    {
      Serial.println("Power_Key is press down");
      digitalWrite(ON_OFF_LED, Pwr_onoff); //写入当前ON_OFF_LED状态onoff，
      //Serial.println(Pwr_onoff);
      Pwr_onoff = (!Pwr_onoff); //然后ON_OFF_LED状态反转，以便下一次使用。
      //Serial.println("POWER_state is");
      //Serial.println(Pwr_onoff);
      //Serial1.println("POWER_state is");
      //Serial1.println(Pwr_onoff);
      delay(5);  //延时一段时间，防止按钮突然断开再按下(10ms 为临时替代按键检测时长，硅胶按键为5没啥)
      while (digitalRead(ON_OFF) == LOW) //判断按钮状态，如果仍然按下的话，等待松开。防止一直按住导致GREEN_ON_OFF_LED输出端连续反转
      {
        delay(1);
      }
    }
  }

  if (Pwr_onoff == 0)
  {
    digitalWrite(REL1, LOW);
    digitalWrite(REL2, LOW);
    //digitalWrite(MOC1, LOW);
    //digitalWrite(MOC2, LOW);
  }
  else
  {
    digitalWrite(REL1, HIGH);
    digitalWrite(REL2, HIGH);
    //digitalWrite(MOC1, HIGH);
    //digitalWrite(MOC2, HIGH);
  }
}
//**********************************************************************************************************

//************************************** fan_set **********************************************************
void fan_set()
{
  if (digitalRead(FAN_HIGH) == LOW) //由于本例检测低电平
  {
    Serial.println("FAN_HIGH is press down");
    buzz();
    if (digitalRead(FAN_HIGH) == LOW) //然后检测仍然是低电平，意为按下按键
    {
      Serial.println("FAN is HIGH state");
      digitalWrite(FAN_LED, Speed); //写入当前ON_OFF_LED状态onoff，
      Speed = (!Speed); //然后Speed状态反转，以便下一次使用。
      //Serial.println("FAN_LED is ");
      //Serial.println(Speed);
      delay(5);  //延时一段时间，防止按钮突然断开再按下(10ms 为临时替代按键检测时长，硅胶按键为5没啥)
      while (digitalRead(FAN_HIGH) == LOW) //判断按钮状态，如果仍然按下的话，等待松开。防止一直按住导致GREEN_ON_OFF_LED输出端连续反转
      {
        delay(1);
      }
    }
  }

  // FAN_LOW按键控制程序 testing ok
  if (digitalRead(FAN_LOW) == LOW) //由于本例检测低电平
  {
    Serial.println("FAN_LOW is press down");
    buzz();
    if (digitalRead(FAN_LOW) == LOW) //然后检测仍然是低电平，意为按下按键
    {
      Serial.println("FAN is LOW state");
      digitalWrite(FAN_LED, Speed); //写入当前ON_OFF_LED状态onoff，
      Speed = (!Speed); //然后Speed状态反转，以便下一次使用。
      //Serial.println("FAN_LED is ");
      //Serial.println(Speed);
      delay(5);  //延时一段时间，防止按钮突然断开再按下(10ms 为临时替代按键检测时长，硅胶按键为5没啥)
      while (digitalRead(FAN_LOW) == LOW) //判断按钮状态，如果仍然按下的话，等待松开。防止一直按住导致GREEN_ON_OFF_LED输出端连续反转
      {
        delay(1);
      }
    }
  }

}
//**********************************************************************************************************
//*************************************** temp_set *********************************************************
void temp_set()
{
  /* 温度按键控制程序 testing ok*/
  if (digitalRead(B43) == LOW) //由于本例检测低电平
  {
    Serial.println("B43_Key is online");
    buzz();
    if (digitalRead(B43) == LOW) //然后检测仍然是低电平，意为按下按键
    {
      //Serial.println("B43 is press down");
      //x = 128;//wt2
      x = 112;//wt3
      heat_PID_struct._SetPoint = 43.0f;
      L43_onoff = 1;
      L38_onoff = 0;
      L32_onoff = 0;
      L24_onoff = 0;
      delay(5);  //延时一段时间，防止按钮突然断开再按下。
      Serial.print("L43 is ");
      Serial.println(L43_onoff);
      Serial.print("L38 is ");
      Serial.println(L38_onoff);
      Serial.print("L32 is ");
      Serial.println(L32_onoff);
      Serial.print("L24 is ");
      Serial.println(L24_onoff);
      while (digitalRead(B43) == LOW) //判断按钮状态，如果仍然按下的话，等待松开。防止一直按住导致GREEN_ON_OFF_LED输出端连续反转
      {
        delay(1);
      }
    }
  }

  if (digitalRead(B38) == LOW) //由于本例检测低电平
  {
    Serial.println("B38_Key is online");
    buzz();
    if (digitalRead(B38) == LOW) //然后检测仍然是低电平，意为按下按键
    {
      //Serial.println("B38 is press down");
      //x = 64;//wt2
      x = 176;//wt3
      heat_PID_struct._SetPoint = 38.0f;
      L43_onoff = 0;
      L38_onoff = 1;
      L32_onoff = 0;
      L24_onoff = 0;
      delay(5);  //延时一段时间，防止按钮突然断开再按下。
      Serial.print("L43 is ");
      Serial.println(L43_onoff);
      Serial.print("L38 is ");
      Serial.println(L38_onoff);
      Serial.print("L32 is ");
      Serial.println(L32_onoff);
      Serial.print("L24 is ");
      Serial.println(L24_onoff);
      while (digitalRead(B38) == LOW) //判断按钮状态，如果仍然按下的话，等待松开。防止一直按住导致GREEN_ON_OFF_LED输出端连续反转
      {
        delay(1);
      }
    }
  }

  if (digitalRead(B32) == LOW) //由于本例检测低电平
  {
    Serial.println("B32_Key is online");
    buzz();
    if (digitalRead(B32) == LOW) //然后检测仍然是低电平，意为按下按键
    {
      Serial.println("B32 is press down");
      //x = 32;//wt2
      x = 208;//wt3
      heat_PID_struct._SetPoint = 32.0f;
      L43_onoff = 0;
      L38_onoff = 0;
      L32_onoff = 1;
      L24_onoff = 0;
      delay(5);  //延时一段时间，防止按钮突然断开再按下。
      Serial.print("L43 is ");
      Serial.println(L43_onoff);
      Serial.print("L38 is ");
      Serial.println(L38_onoff);
      Serial.print("L32 is ");
      Serial.println(L32_onoff);
      Serial.print("L24 is ");
      Serial.println(L24_onoff);
      while (digitalRead(B32) == LOW) //判断按钮状态，如果仍然按下的话，等待松开。防止一直按住导致GREEN_ON_OFF_LED输出端连续反转
      {
        delay(1);
      }
    }
  }

  if (digitalRead(B24) == LOW) //由于本例检测低电平
  {
    Serial.println("B24_Key is online");
    buzz();
    if (digitalRead(B24) == LOW) //然后检测仍然是低电平，意为按下按键
    {
      Serial.println("B24 is press down");
      //x = 16;//wt2
      x = 224;//wt3
      heat_PID_struct._SetPoint = 0.0f; // don't heat 
      L43_onoff = 0;
      L38_onoff = 0;
      L32_onoff = 0;
      L24_onoff = 1;
      delay(5);  //延时一段时间，防止按钮突然断开再按下。
      Serial.print("L43 is ");
      Serial.println(L43_onoff);
      Serial.print("L38 is ");
      Serial.println(L38_onoff);
      Serial.print("L32 is ");
      Serial.println(L32_onoff);
      Serial.print("L24 is ");
      Serial.println(L24_onoff);
      while (digitalRead(B24) == LOW) //判断按钮状态，如果仍然按下的话，等待松开。防止一直按住导致GREEN_ON_OFF_LED输出端连续反转
      {
        delay(1);
      }
    }
  }

  //Serial.println (x);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, x);  //显示数x
  digitalWrite(latchPin, HIGH);
  delay(5);
}
//*****************************************************************temp_ctrl****************************************************************
void temp_ctrl()
{
    //temp; // the current temperature;
   //heat_PID_param;

   //heat_PID_struct;
  float output = 0.0f;
   if (L43_onoff == 1)//43 C
   {
     heat_PID_struct._SetPoint = 43.0f;
   }
   else if(L38_onoff == 1) //38 C
   {
     heat_PID_struct._SetPoint = 38.0f;
   }
   else if(L32_onoff == 1) // 32 C
   {
     heat_PID_struct._SetPoint = 32.0f;
   }
   else if(L24_onoff == 1) // 24 C
   {
     heat_PID_struct._SetPoint = 24.0f;
   }
   output = PID_Calc(&heat_PID_param, &heat_PID_struct, temp);
   //limit the output
   if(output > MAX_RATE_HEAT)
   {
     output = MAX_RATE_HEAT;
   }
   else if(output < MIN_RATE_HEAT)
   {
     output = MIN_RATE_HEAT;
   }
   output = constrain(output, MIN_RATE_HEAT, MAX_RATE_HEAT);
   OCR1A = (unsigned int)output;
}
/*
void temp_ctrl()
{
  if (L43_onoff == 1)
  {
     if (temp >= 42.5)
    {
      digitalWrite(MOC1, LOW);
      digitalWrite(MOC2, LOW);
      overtemp = HIGH;
      //Serial.println(" over temp is  ");
      //Serial.println(overtemp);
    }
    else
    {
      digitalWrite(MOC1, HIGH);
      digitalWrite(MOC2, HIGH);
      overtemp = LOW;
      //Serial.print(" over temp is ");
      //Serial.println(overtemp);
    }
  }
  if (L38_onoff == 1)
  {
    if (temp >= 37.5)
    {
      digitalWrite(MOC1, LOW);
      digitalWrite(MOC2, LOW);
      overtemp = HIGH;
      //Serial.println(" over temp is  ");
      //Serial.println(overtemp);
    }
    else
    {
      digitalWrite(MOC1, HIGH);
      digitalWrite(MOC2, HIGH);
      overtemp = LOW;
      //Serial.print(" over temp is ");
      //Serial.println(overtemp);
    }
  }
  if (L32_onoff == 1)
  {
    if (temp >= 31.5)
    {
       digitalWrite(MOC1, LOW);
      digitalWrite(MOC2, LOW);
      overtemp = HIGH;
      //Serial.println(" over temp is  ");
      //Serial.println(overtemp);
    }
    else
    {
      digitalWrite(MOC1, HIGH);
      digitalWrite(MOC2, HIGH);
      overtemp = LOW;
      //Serial.print(" over temp is ");
      //Serial.println(overtemp);
    }
  }
  if (L24_onoff == 1)
  {
    if (temp >= 24)
    {
      digitalWrite(MOC1, LOW);
      digitalWrite(MOC2, LOW);
      overtemp = HIGH;
      //Serial.println(" over temp is  ");
      //Serial.println(overtemp);
    }
    else
    {
      digitalWrite(MOC1, LOW);
      digitalWrite(MOC2, LOW);
      overtemp = LOW;
      //Serial.print(" over temp is ");
      //Serial.println(overtemp);
    }
  }
}
*/

void onduty_ctrl()
{
  if (Speed == 1)
  {
    //onduty = 200; //38.7% fan is high state 200
    onduty = 64;
    /*
    if (overtemp == 1)
    {
      onduty = onduty * 0.95;
    }
    else if(overtemp == 0)
    {
      onduty =onduty*1.05;
      }
      */
    //Serial.println("fan is high state");
  }
  else if (Speed ==0)
  {
    //onduty = 570; //fan is low state
    onduty = 128;
    /*
    if (overtemp == 1)
    {
      onduty = onduty * 0.95;
    }
    else if(overtemp == 0)
    {
      onduty =onduty*1.05;
      }
      */
    //Serial.println("fan is low state");
  } 
   //OCR1B = onduty; //change the value of prescaler to 1024 ,so the onduty have to multiply 128
   OCR0 = onduty;
}



//**********************************************************************************************************

//*************************************** main loop ********************************************************
void loop() {

  mcu_state();
  on_off();
  fan_set();
  temp_set();
  temp_request();
  temp_ctrl();//control the heat 
  onduty_ctrl();//control the fan
  //ReadDS3231();
  //
  //Serial.print("onduty is ");
  //Serial.println(onduty);
   /*
    if (Speed == 1)
    {
    
    OCR1B = 395; //61.5% fan is high state
    Serial.println("fan is high state 395");
    }
    else
    {
    
    OCR1B = 602; //41% fan is low state
    Serial.println("fan is low state 602");
    }
  */

}//大括号


