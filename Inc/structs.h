#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "calibration.h"


typedef struct{
    } GPIOStruct;

typedef struct{
    }COMStruct;


/* Global Structs */
extern ControllerStruct controller;
extern ObserverStruct observer;
extern COMStruct com;
extern FSMStruct state;
extern EncoderStruct comm_encoder;
extern DRVStruct drv;
extern PreferenceWriter prefs;
extern CalStruct comm_encoder_cal;


#endif
