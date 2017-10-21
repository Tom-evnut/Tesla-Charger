/*
Tesla Gen 2 Charger Phase 1 driver experimental code v1
2017
D.Maguire
Tweaks by T de Bree

Notes:
Serial control
s to toggle charge on or off
v followed by the desired voltage to change setpoint Whole number only
c followed by the desired current to change setpoint Whole number only
*/

#include <due_can.h>  
#include <due_wire.h> 
#include <DueTimer.h>  
#include <Wire_EEPROM.h> 


#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

 // Useful macros for setting and resetting bits
//#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))



//*********GENERAL VARIABLE   DATA ******************

uint16_t voltset = 0;
uint16_t curset = 0;
int  setting = 1;
int incomingByte = 0;
int state =0;

CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.


//setup bytes to contain CAN data
//bytes for 0x045c/////////////////////
byte test52;  //0x45c byte 2 test 2
byte test53;  //0x45c byte 3 test 3
/////////////////////////////////////////

//bytes for 0x042c/////////////////////
byte test20;  //0x42c byte 0 test 4
byte test21;  //0x42c byte 1 test 5
byte test24;  //0x42c byte 4 test 8


/////////////////////////////////////////




void setup() 
  {


    
    Serial.begin(9600);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

    Timer3.attachInterrupt(Charger_msgs).start(100000); // charger messages every 100ms

    

    

 
 // Initialize CAN ports 
      pinMode(48,OUTPUT);
      if (Can1.begin(500000,48)) 
        {
          Serial.println("Using CAN1 - initialization completed.\n");
           
        }
           else Serial.println("CAN1 initialization (sync) ERROR\n");


    // Initialize CAN0
     pinMode(50,OUTPUT);
     if (Can0.begin(500000,50)) 
        {
          Serial.println("Using CAN0 - initialization completed.\n");
        }
        else Serial.println("CAN0 initialization (sync) ERROR\n");


/////////Setup initial state of 2 variable CAN messages///////////////////////////

     test52=0x15;
     test53=0x0e;

     test20=0x42;
     test21=0x60;
     test24=0x64;
///////////////////////////////////////////////////////////////////////////////////


  
}
   



void loop() {
  // put your main code here, to run repeatedly:

 if (Serial.available() > 0)
  {
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case 118://v for voltage setting in whole numbers
         if (Serial.available() > 0)
         {
          voltset = (Serial.parseInt()*100);
          setting = 1;
         }
        break;
        
       case 115://s for start AND stop
         if (Serial.available() > 0)
         {
          state = !state;
          setting = 1;
         }
        break;
      case 99: //c for current setting in whole numbers
         if (Serial.available() > 0)
         {
          curset = (Serial.parseInt()*2000);
          setting = 1;
         }
        break;

      default: 
      // if nothing else matches, do the default
      // default is optional
        break; 
      
    }
  }
 
  if (setting == 1) //display if any setting changed
    {
    Serial.println();
    if (state == 1)
    {
      Serial.print("Charger On   ");
    }
    else
    {
      Serial.print("Charger Off   ");
    }
    Serial.print("Set voltage : ");
    Serial.print(voltset*0.01,0);  
    Serial.print("V | Set current : ");
    Serial.print(curset*0.0005,0);
    Serial.print(" A");
     setting = 0;
    }

switch (state)
  {
    case 0: //Charger off
       test52=0x15;
       test53=0x0e;
       test21=0x60;
       test24=0x64;
      break;
  
    case 1://Charger on
       test52=0x14;
       test53=0x2E;
       test21=0xc4;
       test24=0xfe;
      break;
  
    default:
        // if nothing else matches, do the default
      break;
  }

}

void Charger_msgs()
{
        outframe.id = 0x045c;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=highByte(volset);  //Voltage setpoint
        outframe.data.bytes[1]=lowByte(volset);//Voltage setpoint
        outframe.data.bytes[2]=test52;
        outframe.data.bytes[3]=test53;
        outframe.data.bytes[4]=0x00;
        outframe.data.bytes[5]=0x00;
        outframe.data.bytes[6]=0x90;
        outframe.data.bytes[7]=0x8c;
        Can1.sendFrame(outframe); 

        outframe.id = 0x042c;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=test20; 
        outframe.data.bytes[1]=test21; 
        outframe.data.bytes[2]=lowByte(curset); //Current setpoint
        outframe.data.bytes[3]=highByte(curset); //Current setpoint 
        outframe.data.bytes[4]=test24;
        outframe.data.bytes[5]=0x00;  
        outframe.data.bytes[6]=0x00;
        outframe.data.bytes[7]=0x00;
        Can1.sendFrame(outframe); 


///////////Static Frame every 100ms///////////////////////////////////////////////////////////////////
        outframe.id = 0x368;            // Set our transmission address ID
        outframe.length = 8;            // Data payload 8 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=1;                 //No request
        outframe.data.bytes[0]=0x03;  
        outframe.data.bytes[1]=0x49;
        outframe.data.bytes[2]=0x29;
        outframe.data.bytes[3]=0x11;
        outframe.data.bytes[4]=0x00;
        outframe.data.bytes[5]=0x0c;
        outframe.data.bytes[6]=0x40;
        outframe.data.bytes[7]=0xff;
       

                      

        Can1.sendFrame(outframe); 
////////////////////////////////////////////////////////////////////////////////////////////////////////

}



























