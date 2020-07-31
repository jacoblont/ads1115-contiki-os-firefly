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
 */
/**
 * \addtogroup zoul-examples
 * @{
 *
 * \defgroup
 *
 * Demonstrates the use of the ADS1115 ADC using I2C.
 * Based on the use of the SHT25 digital temperature and humidity sensor
 * And based on the Adafruit ADS1x15 Arduino Library on Github
 * @{
 *
 * \file
 *         A quick program for testing the ADS1115 16-bits I2C ADC
 * \author
 *         Jacob Lont <contiki@jacoblont.nl>
 *         Original SHT file: Antonio Lignan <alinan@zolertia.com>
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "contiki.h"
#include "ads1115.h"
/*---------------------------------------------------------------------------*/
PROCESS(remote_ads1115_process, "ADS1115 test");
AUTOSTART_PROCESSES(&remote_ads1115_process);
/*---------------------------------------------------------------------------*/
static struct etimer et;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(remote_ads1115_process, ev, data)
{
  PROCESS_BEGIN();
  SENSORS_ACTIVATE(ads1115);
  
  // Configure the ADS1115 code
  if(ads1115.configure(ADS1115_ACTIVE, 1) != ADS1115_SUCCESS){
    printf("ADS1115: configuration failed\n");
  }
  else{
    printf("ADS1115: configuration succeeded\n");
  }


  /* Let it spin and read sensor data */
  while(1) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    
    int16_t adc0, adc1; // Initialize (/reset) the sensor values
    
    // get the sensor values
    adc0 = ads1115_value(ADS1115_VAL_S1);
    adc1 = ads1115_value(ADS1115_VAL_S2);

    // Print the values
    printf("Sensor1: %u , ", adc0);
    printf("Sensor2: %u \n", adc1); 
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */