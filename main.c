/* **********************************************************************
 * AVR-GCC source code for Freifunk-Open-MPP-Solar-Tracker
 * Copyright (C) 2017  by Corinna 'Elektra' Aichele 
 * 
 * This file is part of the Open-Hardware and Open-Software project 
 * Freifunk-Open-MPP-Solar-Tracker.
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This source code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this source file. If not, see http://www.gnu.org/licenses/. 
 *************************************************************************/


#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
// #include <util/atomic.h>
//#include <string.h>

/* UART settings */

#ifndef F_CPU
#warning "F_CPU not defined in Makefile. Using 3.686400 MHz"
#define F_CPU 3686400UL 
#endif

/* Define baud rate for serial port */
#define BAUD 9600UL

/* Calculate UART BAUD register setting */
#define UBRR_SETTING ((F_CPU+BAUD*8)/(BAUD*16)-1)

// MCU, firmware revision and controller type
#define FIRM_REV "ATmega8_A_1"

#define UART_MAXSTRLEN 10

    
    /* All Voltages are in mV */
    // v_out_max = Charge end voltage in mV at 25 degrees Celsius for sealed VRLA 12 V battery (Kung-Long, Panasonic and the like)
    // Manufacturers recommend 14.7 V to 14.9 V charge end voltage for cyclic use, 13.6 V to 13.8 V for standby use
    // As a solar system is a mix between cyclic and standby use, 14.15 V - 14.3 V charge end voltage is a good compromise.
    
    uint16_t v_out_max = 14200; 
    
    // v_load_off = Low voltage disconnect voltage in mV. Battery wear depends on depth-of-discharge. 11.7 Volt is moderate, discharging will stop at approximately 15% charge. For extended battery life, v_load_off can be increased to 12.1 V (~50% charge). Lead acid batteries have no memory effect.

    uint16_t v_load_off = 11700; 
    
    // v_load_on = Low voltage disconnect enable voltage in mV. v_load_off and v_load_on are building a hysteresis so the system doesn't turn the load on and off repeatedly. 
    
    uint16_t v_load_on = 12300; 
    
    
    uint16_t v_mpp_estimate;
    uint8_t step = 0x1;
    uint16_t ticks;
    double ptc_resistance;
    double resistor_voltage;
    double temperature;
    double temp_deviation;
    uint16_t v_out_max_temp;
    uint16_t old_mpp_pwm_val = 0x0;
    uint16_t count;
    uint16_t v_out_adcval;
    uint16_t solar_in_adcval;
    uint16_t v_in_value;
    uint16_t v_in_idle_store = 0x100;
    uint16_t v_out_value;
    char vo[] = "";
    volatile uint8_t watchdog_ticks = 0;
    uint16_t watchdog_ticks_main;
    uint16_t watchdog_minutes = 0;
    uint16_t watchdog_reboot_timer = 1440;
    uint8_t lvd_flag = 0;
    volatile uint8_t uart_rx_str_complete = 0; 
    volatile uint8_t uart_rx_str_count = 0;
    volatile char uart_rx_string[UART_MAXSTRLEN + 1];
    char receive_rx_serial_char;
    char config_string[8] = "0";
    uint16_t new_config_value = 0;
    uint16_t watchdog_timer_eeprom EEMEM;
    uint16_t lvd_on_eeprom EEMEM;
    uint16_t lvd_off_eeprom EEMEM;
    char Char_Minimum_Vmpp[6] = "0";
    char Char_V_in_idle[6] = "0";
    //char Char_MPP_estimate[6] = "0" ;
    uint16_t lvd_on = 0;
    uint16_t lvd_off = 0;
    uint8_t trigger_eeprom_reload = 0;
    uint8_t disable_serial_messages = 1;
    uint8_t enable_min_mppt_message = 0;

    
/* ADC init */
void ADC_Init(void)
{
  // Reference: Use Vcc as AVcc
  ADMUX = (1<<REFS0);    
  
  /* Bit ADFR ("free running") in ADCSRA is zero 
   * by default which means single conversion */
  
  // Enable frequency divider
  ADCSRA = (1<<ADPS1) | (1<<ADPS0);
  // set ADC enable
  ADCSRA |= (1<<ADEN);                             

  /* After activating the ADC, a warm-up readout is 
   * recommended to increase accuracy */
  
  // run ADC readout
  ADCSRA |= (1<<ADSC);
  // wait until finished
  while (ADCSRA & (1<<ADSC) ) {             
  }
}



/* ADC read, single conversion */
uint16_t ADC_Read( uint8_t channel )
{ 
  
  /* Select ADC channel */
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
  // single conversion
  ADCSRA |= (1<<ADSC);
  // wait for conversion until finished
  while (ADCSRA & (1<<ADSC) ) { 
  }
  
  return ADCW;
  
}



/* Multiple ADC readouts, calculate average  */
uint16_t ADC_Read_Avg( uint8_t channel, uint8_t nsamples )
    {
    
    uint32_t sum = 0;

    for (uint8_t i = 0; i < nsamples; ++i ) {
    sum += ADC_Read( channel );
            }

    return (uint16_t)( sum / nsamples );
    
    }


/* UART send single character */
int uart_putc(char c)
    {
        // wait until transmit is possible 
        while (!(UCSRA & (1<<UDRE)))  
        {
        }                             
	// send character
        UDR = c;
        return 0;
        
        }

    
/* UART send string */
void uart_puts(char *s)
    {
        while (*s)
        {   //transmit as long as  *s != '\0' 
            uart_putc(*s);
            s++;
        }
    }


    
/* Low voltage disconnect */ 
void low_voltage_disconnect (uint16_t voltage)
    {
	
        // V_out greater v_load_on enables load. lvd_flag keeps load disabled.
	    if (voltage > v_load_on && lvd_flag == 0) 
        { 
        PORTD = (1<<PD7) ;
        disable_serial_messages = 1;
        }
        
        // Below v_load_off, disable load      
        if (voltage < v_load_off) 
        {
        PORTD = (0<<PD7);
        }
        
                    
    }
 
      
/* Measure battery voltage */
 uint16_t measure_v_out(void) {
	v_out_adcval = ADC_Read_Avg(0, 8); 
	v_out_value = (17.57 * v_out_adcval);
        return v_out_value;
}

/* Measure solar input voltage */
uint16_t measure_v_in(void) {
        // Read ADC channel 2, calculate average from 10 readings
        solar_in_adcval = ADC_Read_Avg(2, 8);  
        v_in_value = (29.13 * solar_in_adcval);
        return v_in_value;
    
}

void reboot_timer (void) {
    
            // cli();
             watchdog_ticks_main = watchdog_ticks_main + watchdog_ticks;
             watchdog_ticks = 0; 
            // sei();
    
            if (watchdog_ticks_main >= 850) {
              
           // while (watchdog_ticks_main >= 850) {
                
            watchdog_ticks_main = (watchdog_ticks_main - 850);
            watchdog_minutes++;
            
            
                if (watchdog_minutes >= watchdog_reboot_timer) {
                    
                    PORTD = (0<<PD7);
                    watchdog_minutes = 0;
                    
                    if (lvd_flag == 0) {
                    _delay_ms(5000);
                    }
                    
                    lvd_flag = 0;
                    PORTD = (1<<PD7);
                    disable_serial_messages = 1;
                    }
        }
}

void read_temp_sens (void)
    {
        /* Read voltage of ptc resistance in voltage divider
         * Suggested PTC temperature sensor model:  KTY 81-210
         * R2 = PTC resistance at 25 degrees Celsius 2000 Ohm +- 20 Ohm
         * R1 = 1300 Ohm 
         * R2 = R1 / ((3300mV / resistor_voltage) - 1) 
         * all values in mV */
        
        
        resistor_voltage = (ADC_Read_Avg(3, 8)) * 3.22265 ;
        
        if (resistor_voltage > 3200) {
            
            if (disable_serial_messages == 0) {
            uart_puts ("No temperature sensor \r\n");
            }
        temperature = 0;
        v_out_max_temp = v_out_max;
        return;
        }
        

         ptc_resistance = 1300 /  ((3300 / resistor_voltage) - 1);

       
        // KTY 81-210 is not very accurate.
        // Best accuracy at 40 degrees Celsius
         
        temperature = -30 + ((ptc_resistance - 1247) / 14.15);
        
        /* Calculate and adjust charge end voltage depending on 
        battery temperature for voltage regulated lead acid battery chemistry 
        Correction factor 5 mV per cell for one degree Celsius 
        12 V lead acid type has 6 cells */
        
        if (temperature > 25.00) {
        v_out_max_temp = v_out_max - ((temperature - 25.00) * 30);
        }
        
        if (temperature > 42.00) {
        v_out_max_temp = 13100;
        }
        
        if (temperature < 25.00) {
        v_out_max_temp = v_out_max + ((25.00 - temperature) * 30);
        }
        
        
        
        
    }

                
                
void serialdatareport (void)
    {
        
        // Let the world know whether load is enabled or disabled (if it has power to read the data ;)

        if (PORTD == (1<<PD7))
        {
        uart_puts("Load enabled\r\n");
        }
        else 
        {
        uart_puts("Load disabled\r\n");
        }
        
        read_temp_sens();
        
        uart_puts("Temperature ");
        dtostrf (temperature, 1, 1, vo);
        uart_puts(vo);
        uart_puts (" degrees Celsius\r\nTemperature adjusted charge end: ");
        itoa(v_out_max_temp, vo, 10 );
        uart_puts(vo);
        uart_puts (" mV\r\n");
        
        
        measure_v_in();
        
        uart_puts("V_in ");
        itoa( v_in_value, vo, 10 ); 
        uart_puts( vo );
        uart_puts(" mV\r\nV_in_idle ");
        uart_puts(Char_V_in_idle);
        uart_puts(" mV\r\n");
        measure_v_out();
        uart_puts("V_out "); 
        itoa( v_out_value, vo, 10 ); 
        uart_puts( vo );
        uart_puts(" mV\r\n");
        
        if (enable_min_mppt_message == 1) {
        uart_puts("V_mppt_min ");
        uart_puts(Char_Minimum_Vmpp);
        uart_puts(" mV\r\n");
        }
        
        uart_puts ("Firmware: ");
        uart_puts (FIRM_REV);
        uart_puts ("\r\n");
       
        uart_puts ("\n");
        uart_puts ("Commands:\r\nP=Poweroff (min)\r\nN=Load oN (mV): ");
        itoa((v_load_on), vo, 10 );
        uart_puts (vo);
        uart_puts ("\r\nF=Load oFF (mV): ");
        itoa((v_load_off), vo, 10 ); 
        uart_puts (vo);
        uart_puts ("\r\nW=Watchdog (min): ");
        itoa(watchdog_reboot_timer, vo, 10 );
        uart_puts (vo); 
        uart_puts ("\r\n"); 
        
        itoa((watchdog_reboot_timer - watchdog_minutes), vo, 10 );

        if (PORTD == (1<<PD7)) 
        {
        uart_puts ("Minutes until load off: ");
        uart_puts (vo); 
        }
        else if (lvd_flag == 1)
        {
        uart_puts ("Minutes until load on: ");
        uart_puts (vo); 
        }
        uart_puts ("\r\n\n");
        
       
    }
        
void serial_config (void) {
    
  if (uart_rx_str_count > 7) {
       
      uart_rx_str_complete = 0;
      uart_rx_str_count = 0;
      uart_rx_string[0] = '\0';
  }
      
 
  if (uart_rx_str_complete == 1) {
      
       uint8_t digit = 0;

       while (digit < uart_rx_str_count) 
       {        
           config_string[digit] = uart_rx_string[(digit +2)]; 
           digit++ ;
        }

       
       new_config_value = atol(config_string);

    
      if (uart_rx_string[0] == 'P' && uart_rx_string[1] == '=' && uart_rx_str_count > 2 && new_config_value >= 1 && watchdog_reboot_timer > new_config_value )
     
      {   
          watchdog_minutes = watchdog_reboot_timer - new_config_value;
          PORTD = (0<<PD7);
          lvd_flag = 1;
      }

       
     
      if (uart_rx_string[0] == 'N' && uart_rx_string[1] == '=' && uart_rx_str_count > 2 && new_config_value >= 12000 && new_config_value < 13800)
          
      { eeprom_write_word (&lvd_on_eeprom, new_config_value);
        trigger_eeprom_reload = 1;
        }
      
      if (uart_rx_string[0] == 'F' && uart_rx_string[1] == '=' && uart_rx_str_count > 2 && new_config_value >= 10000 && new_config_value < 13000 )
          
      { eeprom_write_word (&lvd_off_eeprom, new_config_value); 
        trigger_eeprom_reload = 1;
        }  
      
      if (uart_rx_string[0] == 'W' && uart_rx_string[1] == '=' && uart_rx_str_count > 2 && new_config_value >= 60 && new_config_value < 42200 )
      { eeprom_write_word (&watchdog_timer_eeprom, new_config_value);
        trigger_eeprom_reload = 1;
           
        } 

      
      uart_rx_str_complete = 0;
      
      uart_rx_str_count = 0;
      
      uart_rx_string[0] = '\0';
     
    }
}



void apply_eeprom_settings (void) {
    
    
    
    uint16_t watch_temp = eeprom_read_word (&watchdog_timer_eeprom); 
    
    if (watchdog_reboot_timer !=  watch_temp && watch_temp > 9 && watch_temp != -1) {
       watchdog_reboot_timer = watch_temp;
    }
    
    uint16_t load_on_temp = eeprom_read_word (&lvd_on_eeprom); 
    
    if (v_load_on !=  load_on_temp && load_on_temp > 12000 && load_on_temp < 14000 ) {
        v_load_on = load_on_temp;
    }
    
    uint16_t load_off_temp = eeprom_read_word (&lvd_off_eeprom);
    
    if (v_load_off != load_off_temp && load_off_temp > 10000 && load_off_temp < 13000 ) {
        v_load_off = load_off_temp;
    }
    
    trigger_eeprom_reload = 0;
     
}


 void charge_end_limit (void) 
      {
	 /* Reduce charging current at V_out_max_temp */
	 
         uint8_t disable_serial_messages_counter = 0;
         
         measure_v_out();
	 
	 if (v_out_value > (v_out_max_temp + 20)) {
             
             OCR1A = old_mpp_pwm_val;
             count = 0;
             if (disable_serial_messages == 0) {
             uart_puts ("At charge end\r\n");
             }
               

             while(count < 65535) {
                 measure_v_out();
                 if (v_out_value > v_out_max_temp && OCR1A < 0x3FF ){
                    //uart_puts ("PWM UP by 1\r\n");
                    OCR1A += 0x1;}
                  
                if(v_out_value < v_out_max_temp && OCR1A > 0x1) {
                   //uart_puts ("PWM DOWN by 1\r\n"); 
                   OCR1A -= 0x1;}
                   
                   if (count == 10000 || count == 20000 || count == 30000 || count == 40000 || count == 50000 || count == 60000 ) {
                       
                       reboot_timer();
                       
                       if (disable_serial_messages_counter >= 1) {
                           disable_serial_messages = 0;
                           disable_serial_messages_counter = 0; 
                       }
                           
                       if (disable_serial_messages == 1) {
                           disable_serial_messages_counter++;
                       }
                       
                       if (disable_serial_messages == 0) {
                       serialdatareport();
                       }
                   }
                       
                  count++;
                  _delay_ms(1);
             }
            
      }
}

       


/*********************************************************************
 ***********************   Main  *************************************
 ********************************************************************/

int main(void)
{
    
    // Set up GPIOs PD7 and PB1 as output ports
    DDRD = (1<<PD7);
    DDRB = (1<<PB1);
    
    
    // Enable ADC
    ADC_Init();
    
    // Set up PWM1 to control refence voltage of OP-AMP
    TCCR1A|=(1<<COM1A1)|(0<<COM1A0)|(0<<WGM13)|(1<<WGM12)|(1<<WGM11)|(1<<WGM10);
    TCCR1B|=(1<<CS10);
    ICR1=0x3ff;
    OCR1A = 0xf0; 
 
    // Set prescaler of Timer/Counter2 to 1024
    TCCR2 |= ( 1<<CS02 )| ( 1<<CS01)| ( 1<<CS00 ); 
    
    
    /* Use timer0 for watchdog  
       Sleep (idle) time with 3.684 MHz clock is 70.6ms */
    
    // Enable Timer/Counter0 to generate interrupts for watchdog timer clock
    
       TIFR |= (1<<TOV0);
    
	// Set counter0 overflow interrupt
       TIMSK |= (1<<TOIE0);
       //TIMSK |= (1<<TOIE0);
    
    // Use counter0, set prescaler to 1024
       TCCR0 |= ( 1<<CS02 )| ( 0<<CS01)| ( 1<<CS00 );
    
    
    // Enable UART 
        UBRRH = UBRR_SETTING >> 8; 
        UBRRL = UBRR_SETTING & 0xFF;
        
        
        UCSRB |= (1<<RXCIE) | (1<<RXEN) | (1<<TXEN);
        //UCSRB |= (1<<TXEN);
        
	// select asynchronous mode 8N1 
       UCSRC |= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // asynchronous mode 8N1 
       
       apply_eeprom_settings();
       
       _delay_ms(800);
    
    // Enable interrupts
       sei();  
       
    
        _delay_ms(5);
        
    v_out_max_temp = v_out_max;


/* Main program loop */

while (1) {
        
        
        
        measure_v_out();
        low_voltage_disconnect(v_out_value);
        charge_end_limit();
        reboot_timer();
		 
		 
        /* Check if there is actually power from the solar panel.
         * If not, sleep for a while */
        /* First, measure solar input voltage */
        measure_v_in();
		
        /* If solar power is coming in and 
        * we didn't reach maximum output, 
        * run the MPP routine */ 
		 
        if ((v_out_value < v_in_value) && (v_out_value < (v_out_max_temp - 20))) {
		  
        /* Measure solar panel open circuit voltage 
         * and calculate MPP point */
		  
          
          OCR1A = 0x3FF;
          _delay_ms(400);
          measure_v_in();
          
          
          itoa(v_in_value, Char_V_in_idle, 10 );

          v_mpp_estimate = v_in_value / 1.24;
          //itoa( v_mpp_estimate, Char_MPP_estimate, 10 ); 
		  
          /* Set MPP to lowest possible point (PWM output of AVR = 0) */
          OCR1A = 0x0;
          
          charge_end_limit();
          _delay_ms(200);
          measure_v_in();
          
          enable_min_mppt_message = 1;
         
          itoa( v_in_value, Char_Minimum_Vmpp, 10 );
         
          
         /* Ramp up PWM until reaching the
          * calculated MPP input voltage */
        
          measure_v_in();
          measure_v_out();
		  
          while ((v_in_value < v_mpp_estimate) && (v_out_value < (v_out_max_temp - 150)))  {
            OCR1A += step;
            _delay_ms(10);
            measure_v_in();
            measure_v_out(); 
            }
		    
         ticks = 0;
         while (ticks != 100) {
         set_sleep_mode(SLEEP_MODE_IDLE);
         sleep_enable();
         sleep_mode();
         ticks ++; 
         charge_end_limit();
         reboot_timer();
            }
                        
         } 

   
    if (v_out_value > v_in_value) {
       
       //uart_puts("No solar energy.");
       
       itoa(0, Char_V_in_idle, 10 );
       enable_min_mppt_message = 0; 
       ticks = 0;
       while (ticks < 250) {
       set_sleep_mode(SLEEP_MODE_IDLE);
       sleep_enable();
       sleep_mode();
       ticks ++;
       reboot_timer();
       
                    }
        }
       
       
        charge_end_limit();
        serial_config();
        disable_serial_messages = 0;
        _delay_ms(5000);
        serialdatareport();
        

        reboot_timer();
        
        if (trigger_eeprom_reload == 1) {
            apply_eeprom_settings(); }

    }


   return 0; // never reached 
}


/***********************************
 * Interrupt service routines ******
 * ********************************/
        
        

/* ISR TIMER0 overflow routine */

ISR( TIMER0_OVF_vect )
      {
          
       watchdog_ticks++;
          

      } 

      

ISR(USART_RXC_vect) {
    
  // read data from buffer 
    receive_rx_serial_char = UDR;
     
    if( uart_rx_str_complete == 0 ) {
 
    if( receive_rx_serial_char != '\n' &&
        receive_rx_serial_char != '\r' &&
        uart_rx_str_count < UART_MAXSTRLEN ) {
        uart_rx_string[uart_rx_str_count] = receive_rx_serial_char;
        uart_rx_str_count++; }
        
    else {
      
      uart_rx_string[uart_rx_str_count] = '\0';
      uart_rx_str_complete = 1;
        
      }
    }
 }

    
