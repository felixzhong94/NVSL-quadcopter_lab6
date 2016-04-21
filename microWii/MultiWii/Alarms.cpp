#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Sensors.h"
#include "Alarms.h"

void toggleResource(uint8_t resource, uint8_t activate);

/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
    }
    delay(60); //wait 60 ms
  }
}

/********************************************************************/
/****                   Global Resource Handling                 ****/
/********************************************************************/
 
  void toggleResource(uint8_t resource, uint8_t activate){
	  if (activate == 1) {LEDPIN_ON;}
	  else LEDPIN_OFF;
      return;
  }

