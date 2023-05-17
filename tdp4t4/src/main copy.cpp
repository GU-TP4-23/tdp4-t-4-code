//--------Team 4 SLAVE/MASTER code
#include <Arduino.h>
#include "Z_Control_Functions.h"
#include "hardware/i2c.h"
#include <Wire.h>
#include "coordinates.h"

#define LED1 6
#define LED2 7
#define G_LED_SLAVE LED1
#define R_LED_MASTER LED2
#define CURRENT_EN 20
#define AN_R 26
#define COMP_INT 18
#define ADC_BITS 10


#define TEAM1 51
#define TEAM2 52
#define TEAM3 53
#define TEAM4 54
#define XY_MOTION 55
#define Z_MOTION 56

#define I2C_PORT i2c0

byte Z_Offset = 100;
byte MT1 = 3;

int MASTERswitchSTATE;  //the switch only turns the MCU into a master, slave is activated automatically
int MasterInit = 0;      //Team4 is initially slave
int switch_pin = 1;  //switch is GPIO 1

float bias_current = 87.5*pow(10,-6);
bool mesure = false;

volatile bool c_trig = false;
volatile int trig_time = 0;

float r_mesure(){
    digitalWrite(CURRENT_EN, HIGH);
    int a_total = 0;
    int samples = 200;
    for (int i=0; i<samples; i++){
        a_total += analogRead(AN_R);
        delay(1);
    }
    float a_in = (a_total*1.0)/(1.0*samples);
    float v = 3.3-3.3*(a_in/pow(2,ADC_BITS));
    float r = v/bias_current;
    Serial.print(a_in);
    Serial.print("  |  ");
    Serial.println(r);
    digitalWrite(CURRENT_EN, LOW);
    return r;
}

void c_interupt(){
    if (c_trig == true){
        trig_time = micros();
        c_trig = false;

    }
}

float c_mesure(){
    digitalWrite(CURRENT_EN, HIGH);
    float dv = 3.3-1.5;
    int start_trig = micros();
    while(analogRead(AN_R) > 2.5/pow(2,ADC_BITS)*3.3){
    }
    while(analogRead(AN_R) < 1.5/pow(2,ADC_BITS)*3.3){
    }
    int trig_time = micros();
    int tdelta = (trig_time-start_trig);
    float capacitance = 1000 * 0.4838 * bias_current*(tdelta/dv);
    digitalWrite(CURRENT_EN, LOW);
    delay(10000);
    return capacitance;
}

void receiveEvent(int howMany)
  {   // howMANY is always equal to no. of bytes received
    int Slave4Receives = Wire.read();       
    //reads handover byte from master3
    if (Slave4Receives == 3 )
      {          //if team4 as slave receives a 3 from other team acting as master
      Serial.println(3);
      MasterInit = 1;                 //Now team4 can be initialised as master instead of slave
      Serial.println("Received Byte from Master");
      setup(); 
      }
    else 
      {
      MasterInit = 0;
      }  
  }



void readSwitch()  
  {
    int sVal = digitalRead(switch_pin);
    if (sVal == 1) 
      {
        MASTERswitchSTATE = 1;
      }
    else
      {
        MASTERswitchSTATE = 0;
      }
  } 


void setup(){
    attachInterrupt(digitalPinToInterrupt(switch_pin),readSwitch,RISING);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(CURRENT_EN, OUTPUT);
    pinMode(AN_R, INPUT);
    pinMode(COMP_INT, INPUT);
    Serial.begin();
    //attachInterrupt(digitalPinToInterrupt(COMP_INT),c_interupt,FALLING);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);      //sets up pull-up resistors on I2C pins, essential for I2C
    gpio_pull_up(5);
    if (MasterInit == 1)
      {     //Master Setup: Team4 is configured to Master on I2C0 channel
        Wire.end();  
        Wire.begin(); 
        Serial.begin(9600);
      }   
    else if(MasterInit == 0)
      {            //Slave Setup: Team4 is configured to Slave on I2C0 channel
        Wire.end();
        Wire.begin(TEAM4);
        Serial.begin(9600); 
        Wire.onReceive(receiveEvent);
      }   

}

void loop(){
    digitalWrite(LED2, HIGH);
    digitalWrite(LED1, HIGH);
    digitalWrite(CURRENT_EN, LOW);
    Serial.println("started");
    delay(1000);
    Serial.println("delay_finnished");
    float c = c_mesure();
    delay(100);
    Serial.println(c);
    digitalWrite(LED1, LOW);
    // digitalWrite(LED2, HIGH);
    // digitalWrite(LED1, HIGH);
   
    // if (MasterInit==1)
    //   {        
    //     Serial.println("Master! ");            //Prints to serial monitor
    //     if (MASTERswitchSTATE == 0) 
    //       {
    //         digitalWrite(LED_BUILTIN, HIGH);  //Board LED goes high for Master operation
    //         digitalWrite(G_LED_SLAVE, LOW);  //Slave LED goes LOW
    //         digitalWrite(R_LED_MASTER, LOW);  //Master state LED goes LOW 
    //         Serial.println("Flick Switch to change state and send byte");
    //         delay(2000);
    //         setup(); 
    //       }
    //     else if (MASTERswitchSTATE == 1)
    //       {
    //         Serial.println("Team4 Master is ready to transmit");
    //         digitalWrite(LED_BUILTIN, HIGH);  //Board LED goes high for Master operation
    //         digitalWrite(G_LED_SLAVE, LOW);  //Slave LED goes LOW
    //         digitalWrite(R_LED_MASTER, HIGH);  //Master state LED goes HIGH


    //         //ENTER YOUR TEAMS CODE HERE
    //         //MAX Z-Offset is 63000 um
    //         Z_Coord_Sender(0); //this is to reset to home point after team is finished
    //         Serial.println("Terminating transmission...");
    //         Serial.println("Transmission terminated.");
                
          
    //         //Master Handover to Team 1
    //         delay(2000);
    //         Wire.beginTransmission(TEAM1);  //This is sending handover to team1, Master4 writing to slave1
    //         Wire.write(MT1);        //WRITING MT1 BYTE = 3
    //         Wire.endTransmission(); 
    //         delay(100);
    //         Serial.println("GOOD: Team4 has sent MT1 byte to team 1 ");
    //         delay(1000);
    //         MasterInit = 0;
    //         MASTERswitchSTATE = 0; //this is for 2nd time around, so that switch does automatically turn on
    //         setup();
    //       }
    //   }
    // // TEAM 4 has been initialised as SLAVE
    // else if (MasterInit == 0) 
    //   {
    //     Serial.println("SLAVE! ");  
    //     if (MASTERswitchSTATE == 1)
    //       {
    //         Serial.println("Team4 is now a slave ");
    //         digitalWrite(LED_BUILTIN, LOW);  //Board LED goes LOW for Slave operation
    //         digitalWrite(G_LED_SLAVE, HIGH);  //Slave LED goes HIGH
    //         digitalWrite(R_LED_MASTER, LOW);  //Master state LED goes LOW
    //         delay(2000);
    //         setup(); 
    //       }
    //     else if (MASTERswitchSTATE == 0) 
    //       {
    //         Serial.println("Team4 is now a slave ");
    //         digitalWrite(LED_BUILTIN, LOW);  //Board LED goes LOW for Slave operation
    //         digitalWrite(G_LED_SLAVE, HIGH);  //Slave LED goes HIGH
    //         digitalWrite(R_LED_MASTER, LOW);  //Master state LED goes LOW
    //         delay(2000);
    //         setup();
    //       }
    //   }
    
}