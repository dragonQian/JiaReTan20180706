/************************************
**Program ID:WT2_SWJ
**Cpu ID: Atmega328P
**Software Ver:0.3
**Hardware Ver:0.3
**program by zhengkai
**lasting Update:20180525
**ADD LOGO
***************************************/

/*-----------------------------------Head file include ----------------------------begin*/
#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8glib.h>
/*-----------------------------------Head file include ----------------------------end*/
/*------------------------------------screen logo----------------------------------begin*/
static unsigned char u8g_logo_bits[] U8G_PROGMEM = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3F,0x70,0x00,0x00,
0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3F,0x70,0xF8,0x1F,
0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3F,0x70,0xF8,0x1F,
0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3F,0x70,0x18,0x18,
0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x19,0x18,
0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xF9,0x1F,
0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7C,0xF8,0x1F,
0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x18,0x18,
0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x18,0x18,
0x00,0x0F,0xC0,0x3F,0x80,0xC7,0x7F,0xE0,0xF1,0x0B,0x00,0xFF,0x01,0x70,0xF8,0x1F,
0x00,0x0F,0xF0,0xFF,0x80,0xEF,0xFF,0xE0,0xFB,0x1F,0x00,0xFF,0x07,0x70,0xF8,0x1F,
0x00,0x0F,0xF8,0xFF,0x81,0xFF,0xFF,0xE1,0xFF,0x3F,0xC0,0xFF,0x0F,0xF8,0x01,0x00,
0x00,0x0F,0xFC,0xFF,0x83,0xFF,0xFF,0xE3,0xFF,0x7F,0xE0,0xFF,0x1F,0xFE,0xFC,0x7F,
0x00,0x0F,0x7E,0xF8,0x87,0xFF,0xFF,0xE3,0x0F,0x7E,0xF0,0xC3,0x1F,0x1E,0xFE,0x7F,
0x00,0x0F,0x3F,0xF0,0x87,0x7F,0xF8,0xE7,0x07,0xFC,0xF8,0x81,0x3F,0x00,0xFE,0x7F,
0x00,0x0F,0x1F,0xE0,0x87,0x3F,0xF0,0xE7,0x07,0xFC,0xF8,0x00,0x3F,0x00,0x00,0x00,
0x00,0x0F,0x0F,0xC0,0x87,0x3F,0xE0,0xE7,0x03,0xF8,0x79,0x00,0x3E,0xC0,0x00,0x00,
0x00,0x0F,0x0E,0xC0,0x87,0x1F,0xC0,0xE7,0x01,0xF0,0x79,0x00,0x3E,0xC0,0x80,0x03,
0x00,0x0F,0x00,0xFC,0x87,0x0F,0x80,0xE7,0x01,0xF0,0x01,0xC0,0x3F,0xE0,0xC1,0x01,
0x00,0x0F,0xC0,0xFF,0x87,0x0F,0x80,0xE7,0x01,0xF0,0x01,0xFC,0x3F,0xF8,0xE7,0x00,
0x00,0x0F,0xFC,0xFF,0x87,0x0F,0x80,0xE7,0x01,0xF0,0xC1,0xFF,0x3F,0xF8,0xF7,0x3F,
0x00,0x0F,0xFF,0xFF,0x87,0x0F,0x80,0xE7,0x01,0xF0,0xF1,0xFF,0x3F,0xD8,0xF6,0x3F,
0x00,0x0F,0xFF,0xCF,0x87,0x0F,0x80,0xE7,0x01,0xF0,0xF9,0x7F,0x3E,0xD8,0x76,0x38,
0x00,0x8F,0x7F,0xC0,0x87,0x0F,0x80,0xE7,0x01,0xF0,0xF9,0x03,0x3E,0xD8,0x76,0x38,
0x00,0x8F,0x1F,0xC0,0x87,0x1F,0xC0,0xE7,0x03,0xF8,0xF9,0x00,0x3E,0xD8,0xF6,0x3F,
0x00,0x8F,0x0F,0xC0,0x87,0x1F,0xC0,0xE7,0x03,0xF8,0x79,0x00,0x3E,0xD8,0xF6,0x3F,
0x00,0x8F,0x0F,0xC0,0x87,0x3F,0xE0,0xE7,0x07,0x7C,0x78,0x00,0x3F,0xD8,0xF6,0x3F,
0x00,0x8F,0x0F,0xE0,0x87,0x7F,0xF0,0xE7,0x0F,0x7E,0x78,0x00,0x3F,0xD8,0x76,0x38,
0x00,0x8F,0x1F,0xF8,0x87,0xFF,0xFF,0xE3,0xFF,0x3F,0xF8,0xC0,0x3F,0xD8,0x76,0x38,
0x00,0x8F,0x3F,0xFC,0x87,0xFF,0xFF,0xE1,0xFF,0x1F,0xF8,0xE1,0x3F,0xD8,0x76,0x38,
0x00,0x0F,0xFF,0xFF,0x8F,0xFF,0xFF,0xE0,0xFF,0x0F,0xF0,0xFF,0x7F,0xC0,0xF0,0x3F,
0x00,0x0F,0xFE,0xBF,0x8F,0xCF,0x7F,0xE0,0xF9,0x07,0xE0,0xFF,0x7C,0xC0,0xF0,0x3F,
0x00,0x0F,0xFC,0xDF,0x8F,0x8F,0x3F,0xE0,0xF1,0x03,0xC0,0x3F,0x7C,0xC0,0xF0,0x3F,
0x00,0x0F,0xF0,0x07,0x80,0x0F,0x00,0xE0,0x01,0x00,0x80,0x3F,0x7C,0xC0,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x3F,0x10,0x87,0xE1,0x7F,0xCE,0x79,0x8E,0x9F,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0x1C,0x87,0xCC,0x7F,0xC6,0x1C,0x26,0x87,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x7F,0x4E,0x93,0xFC,0x7F,0xC6,0x0C,0x27,0xC3,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x7F,0x4E,0xC3,0xF8,0x7F,0x52,0x2E,0x33,0xCB,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x7F,0x4E,0xC9,0xF1,0x7F,0x52,0x3E,0x33,0xCF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x7F,0x4E,0xC9,0x63,0xC0,0x18,0x3F,0x33,0xCF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x7F,0x4E,0xCC,0x67,0xC0,0x18,0x9F,0x93,0xE7,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x3F,0x67,0x64,0xE6,0xFF,0x9C,0x9F,0x93,0xE7,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x3F,0x67,0xE6,0xF0,0xFF,0x9C,0x9F,0xC7,0xE7,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};
/*------------------------------------screen logo----------------------------------end*/
/*------------------------------------global value---------------------------------begin*/
#define FOSC_this 16000000
#define RAUD_this 9600
#define MYUBRR FOSC_this/16/RAUD_this-1

#define High_temp 7         //define high_temp mark
#define Error 6             //define error mark

#define STATUS_FLAG 28
#define ALARM_POINT 55

float serial_temp = 0;
float alarm_temp = 0;

bool receive_complete_flag = false;
bool receive_OK = false;

unsigned int rx_num = 0;

volatile unsigned char REC_DATA[24] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                              
volatile unsigned char SED_DATA[24] = {0xaa,0xbb,0x00,0x00,0x00,0x00,0x00,0x00,
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

union swt_float
{
    char _ch[4];
    float _f;
}swt_tmp;

enum PROC_LOOP_STATUS
{
      BEGIN=1, REC_MES, GET_ALARM, SED_MES, SHOW_RESULT, END_LOOP
}main_loop;

#define ONE_WIRE_BUS 14                 //define a one_wire port
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress Device0, Device1, Device2;//arrays to hold device address

//define the oled orginal state
#define OLED_RST 12
U8GLIB_NHD27OLED_2X_BW u8g(13, 11, 10, 9);  // SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9

/*------------------------------------global value-----------------------------------end*/

/*------------------------------------------*function define*----------------------begin*/
void draw_temp(float temperature);
void Initial_Serial(void);

void m_putchar(unsigned char x);
void send_dat(unsigned char *pd,unsigned char len);

/*------------------------------------------*function define*------------------------end*/
/*------------------------------------------*initial start*------------------------*/
void setup()
{  
  // start serial port
  pinMode(Error, OUTPUT);
  pinMode(High_temp, OUTPUT);
  digitalWrite(High_temp, HIGH);
  digitalWrite(Error, HIGH); 
  //pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(100);
  digitalWrite(OLED_RST, HIGH);
  
  //display init
  u8g.begin();
  //show the Tappa logo
  u8g.firstPage();
  do{
    u8g.drawXBMP( 0, 0, 128, 64, u8g_logo_bits);
  } while( u8g.nextPage() ); 
  //delay 2s to show logo 
  delay(2000);
  //
  Initial_Serial();
  //Serial.begin(9600);
  //init the ds18b20
  //Serial.println("OLED is Starting......");
  //Serial.print("Locating devices...");
  //sensors.begin();
  //Serial.print("Found ");
  //Serial.print(sensors.getDeviceCount(), DEC);
  //Serial.println(" devices.");


  //report parasite power requirements
  //Serial.print("Parasite power is: ");
  //if (sensors.isParasitePowerMode()) Serial.println("ON");
  //else Serial.println("OFF");
  //if (!sensors.getAddress(Device0, 0)) Serial.println("Unable to find address for Device0");
  //sensors.setResolution(Device0, 11);
  //Serial.print("Device0 Resolution: ");
  //Serial.print(sensors.getResolution(Device0), DEC);
  //Serial.println();
  main_loop = BEGIN;
  
}

void loop()
{
  unsigned char CRC_ACC=0,i;
  float get_temp = 0;
  //--------------------------------------------------while for receive 24 byte
  do
  {
    if(receive_complete_flag){
      for(i=2;i<23;i++)//macro
        CRC_ACC += REC_DATA[i];
      receive_complete_flag = false;
      }
    if(CRC_ACC == REC_DATA[23]){//macro
      receive_OK = true;
      swt_tmp._ch = &REC_DATA[2];
      serial_temp = swt_tmp._f;
      }
      else{
        receive_OK = false;
      }  
    }while(!receive_OK);
    main_loop = REC_MES;
  //----------------------------------------------------while for the alarm flag or the value of temperature
  UCSR0B &=~_BV(RXCIE0);//close the interrupt of Recieve;                 
  do
  {
     sensors.begin();
     delay(10);
     sensors.setResolution(Device0, 11);
     sensors.requestTemperatures();
     get_temp = sensors.getTempCByIndex(0);
     alarm_temp = get_temp;
  }while(get_temp <=0 || get_temp > 100);
  UCSR0B |= _BV(RXCIE0);//open the interrupt of Recieve;
  main_loop = GET_ALARM;
  //sensors.requestTemperatures();   //发送温度测量请求命令
  //get_temp = sensors.getTempCByIndex(0);
  //Serial.print("DS18B20 is: ");
  //Serial.println(get_temp-3);  //获取0号传感器温度数据并发送
  
  //-------------------------------------------------------while for sending the 24 byte to the downboard 
 
 for(i=0;i<24;i++){
    SED_DATA[i] = REC_DATA[i];//填充报警信息
  }
  if(alarm_temp > ALARM_POINT)
    SED_DATA[13] = 0x01;
  else{
    SED_DATA[13] = 0x00;
    }
  SED_DATA[23] += SED_DATA[13];
  send_dat(SED_DATA,24);
  while(!(UCSR0A&(1<<TXC0)));
  UCSR0A |= _BV(TXC0);
  
  main_loop = SED_MES;
  //-----------------------------------------------while for the show the temperature from the sensor whese position set wind port 
  
  u8g.firstPage();
  do
  {
    draw_temp(serial_temp);
    //Serial.print("LCD is working: ");
  }while(u8g.nextPage());

  main_loop = SHOW_RESULT;
}

void Initial_Serial(void){
  //setting the baud rate
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  //setting frame format ...  Set frame format: 8data, 1stop bit 
  UCSR0C = 3<<UCSZ00;
  //enabling the Transmitter or the Receiver depending on the usage
  UCSR0B = (1<<RXEN0)|(1<<TXEN0) | (1<<RXCIE0);
  //set the port for serial PD0 = RXD  PD1 = TXD 
  DDRD &=~_BV(PD0);
  DDRD |= _BV(PD1);
  PORTD |= _BV(PD0);
  
  //example code 
  /*
  #define FOSC 1843200 // Clock Speed
  #define BAUD 9600
  #define MYUBRR FOSC/16/BAUD-1
  void main( void )
  {
  ...
  USART_Init(MYUBRR)
  ...
  }
  void USART_Init( unsigned int ubrr)
  {
  //Set baud rate 
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  Enable receiver and transmitter 
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  // Set frame format: 8data, 2stop bit 
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
  }
  */
  }

void draw_temp(float temperature)
{  
  //*************** thermo temp sense *********************
  u8g.setFont(u8g_font_helvB12r);
  if (temperature > STATUS_FLAG)
  {
  u8g.drawStr(0, 60, "System Working");
  }
  else if(temperature <= STATUS_FLAG)
  {
  u8g.drawStr(0, 60, "System Standby");
  }
  u8g.setPrintPos(88, 15);
  u8g.print("o");
  u8g.setFont(u8g_font_fub30r);//字体设置
  u8g.setPrintPos(5, 40);
  u8g.print(temperature, 1);
  u8g.setPrintPos(95, 40);
  u8g.print("C");
  delay(5);
}
ISR(USART_RX_vect){
  unsigned char temp;
  UCSR0B &=~_BV(RXCIE0);
  temp=UDR0;

  if((temp == 0xaa) && (rx_num == 0)){
    REC_DATA[rx_num++] = temp;
    return;
    }
    else if((temp == 0xbb) && (rx_num == 1)){
      REC_DATA[rx_num++] = temp;
      return;
    }
    else {
      if((REC_DATA[0] == 0xaa) && (REC_DATA[1] == 0xbb))
         REC_DATA[rx_num++] = temp;
      else
         rx_num = 0;
    }
    if(rx_num >= 24){
      receive_complete_flag = true;
      rx_num = 0;
    }
    UCSR0B |= _BV(RXCIE0);//
  }

void m_putchar(unsigned char x){
  while(!(UCSR0A&(1<<UDRE0)));
  UDR0 = x;
  while(!(UCSR0A&(1<<TXC0)));
  UCSR0A |= _BV(TXC0);
  }

void send_dat(unsigned char *pd,unsigned char len){
  unsigned char i;
  for(i=0;i<len;i++){
    m_putchar(*pd);
    pd++;
    }
  }


  

