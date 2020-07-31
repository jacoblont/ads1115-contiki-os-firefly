/*
 * Copyright (c) 2015, Zolertia <http://www.zolertia.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * This file 'ads1115.c' is based on sht25.c (zoul platform)
 * Author: Jacob Lont (github: jacoblont)
 * Date 16-03-2020
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup zoul-ads1115-sensor
 * @{
 *
 * \file
 *         ads1115 16-bits I2C ADC file
 * \author
 *         Jacob Lont <contiki@jacoblont.nl>
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "contiki.h"
#include "dev/i2c.h"
#include "ads1115.h"
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
static uint8_t enabled;

/*---------------------------------------------------------------------------*/
// Based on the SHT25 code // JL: This is needed as part of the SENSORS_SENSOR struct.
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return enabled;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int //  Based on the SHT25 code
ads1115_read_reg(uint8_t reg, uint8_t* buf, uint8_t num, uint8_t i2cAddress) //JL: register, buffer-pointer, num?? number of bytes? Yes.
{
  if((buf == NULL) || (num <= 0)) {
    printf("ADS1115: ads1115_read_reg buf-num ERROR \n");
    return ADS1115_ERROR;
  }

  i2c_master_enable(); // needed here? It is also never disabled? Shouldnt this enable be done in configure?
  if(i2c_single_send(i2cAddress, reg) == I2C_MASTER_ERR_NONE) {
    if(i2c_burst_receive(i2cAddress, buf, num) == I2C_MASTER_ERR_NONE) {
//      printf("ADS1115: ads1115_read_reg SUCCES \n");
      return ADS1115_SUCCESS;
    }
  }
  printf("ADS1115: ads1115_read_reg returns ERROR \n");
  return ADS1115_ERROR;
}


/*---------------------------------------------------------------------------*/
// Function used to write to a register
static int
ads1115_write_reg(uint8_t i2cAddress, uint8_t* buf, uint8_t num)
{
  //printf("ADS1115: ads1115_write_reg started\n");
  if((buf == NULL) || (num <= 0)) {
    printf("ADS1115: invalid write reg values\n");
    return ADS1115_ERROR;
  }
  if(i2c_burst_send(i2cAddress, buf, num) == I2C_MASTER_ERR_NONE) {
    //printf("ADS1115: I2C burst send succeeded\n");
    return ADS1115_SUCCESS;
  }
  // Should be the addresses:
  printf("buf is: %p, ", buf);
  printf("*buf+1 is: %p\n", (buf+1));
  printf("*buf+2 is: %p\n", (buf+2));
  // Should be the bytes:
  printf("*buf is: %u\n", *buf);
  printf("*buf++ is: %d\n", *(buf+1));
  printf("*buf++ is: %d\n", *(buf+2));
  printf("ADS1115: ads1115_write_reg returns ERROR\n");
  return ADS1115_ERROR;
}


/**************************************************************************/
// Read 16-bits from the specified destination register
static uint16_t readRegister(uint8_t reg) {
  uint8_t i2cresult[2]; // 2 byte result array
  ads1115_read_reg(reg, &i2cresult[0], (uint8_t)2, ADS1115_ADDRESS);
   
  //printf("readRegister gives:  %u \n",(i2cresult[0] << 8) | i2cresult[1]);
  return ((i2cresult[0] << 8) | i2cresult[1]);
}




/**************************************************************************/
//  Gets a single-ended ADC reading from the specified channel
uint16_t readADC_SingleEnded(uint8_t channel) {
  channel = channel-1; // channel is base 0, but is send in as base 1.
  if (channel > 3) {
    return 0;
  }

  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      //ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
      ADS1015_REG_CONFIG_MODE_CONTIN;   // Set to continuous conversion

  // Set PGA/voltage range
  config |= CHOSEN_GAIN;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  //printf("Config is: %u\n", config);
  /*   In WriteRegister, the first byte that would be send, was the byte with the register address.
    We need to make a variable like: 8bits_REG_Pointer, 16 bits config for ads1115_write_reg.
    -> Create a 3 byte array to send using i2c_burst_send within ads1115_write_reg.
  */
  //printf("Going to call ads1115_write_reg\n");
  // Create the array and send it
  uint8_t sendbuffer[3]; // Create a 3 byte array
  sendbuffer[0] = ADS1015_REG_POINTER_CONFIG;
  sendbuffer[1] = (config >> 8) & 0xFF; // The first byte of the 2-byte config variable
  sendbuffer[2] = config & 0xFF; // The second byte of the config variable
  uint8_t *ptr;
  ptr = sendbuffer; // Points to 0th element of the sendbuffer arr
  
  if(ads1115_write_reg(ADS1115_ADDRESS, ptr, (uint8_t)3) != ADS1115_SUCCESS){
    return (uint16_t)0; // ERROR = -1, while uint16_t > 0. Now we can also see that something is wrong.
  } 
  
  // Wait for the conversion to complete
  //printf("ADS1115: Starting the rtimer\n");
  // Wait for the conversion to complete (this (/100) should do +-11 ms, I need +-9 ms, so its fine)
  rtimer_clock_t t0;
  t0 = RTIMER_NOW();
  while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + RTIMER_SECOND / 100));
  //printf("ADS1115: rtimer done\n");

  // Read the conversion results
  return readRegister(ADS1015_REG_POINTER_CONVERT); 
}



/*---------------------------------------------------------------------------*/
// Function to request readings from the ADS1115
uint16_t ads1115_value(int type)
{
  if(type == ADS1115_VAL_S1 || type == ADS1115_VAL_S2 ){
    //printf("ADS1115: function ads1115_value started\n");

    if(!enabled) {
      printf("ADS1115: sensor is not configured\n");
      return ADS1115_ERROR;
    }

    //printf("ADS1115: calling readADC_SingleEnded from ads1115_value\n");
    uint16_t val;
    val = readADC_SingleEnded(type);
    return val;
  }
  
  // Else
  printf("ADS1115: ads1115_configuration option not supported\n");
  return ADS1115_ERROR;
}



/**************************************************************************/
// Function to configure the ADS1115 for contiki SENSORS_SENSOR structure
// JL: This is needed as part of the contiki SENSORS_SENSOR struct.

static int
configure(int type, int value_arg)
{
  switch(type) {
  case (ADS1115_ACTIVE):
    if(value_arg){
      i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);
      enabled = 1;
      //printf("ADS1115: configure function succeeded\n");
      return ADS1115_SUCCESS;
    }
  default:
    PRINTF("ADS1115: configure option not supported\n");
    return ADS1115_ERROR;
  }
}

/*---------------------------------------------------------------------------*/
// JL: This is needed as part of the contiki SENSORS_SENSOR struct.
// To get the actual sensor values, use the function 'ads1115_value'
// The original content of this function in this case comes from
// https://github.com/contiki-os/contiki/blob/master/dev/sht11/sht11-sensor.c
static int
value(int type)
{
  return type;
  //return 0;

}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(ads1115, ADS1115_SENSOR, value, configure, status); //JL Todo check what these arguments are for. Some of them seem to be functions I commented.
/*---------------------------------------------------------------------------*/
/** @} */


/***       COMMENTED OUT THINGS ARE BELOW THIS LINE       ***/

