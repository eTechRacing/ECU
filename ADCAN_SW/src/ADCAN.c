/* ***************************************************************************
**  File Name    : ADCAN.c
**  Version      : 2.0
**  Description  : ADC conversion and CAN communication using Microchip MCP2561 with 
                   multiple analog sensors (refer to Sensorics 2018-2019 design) 
**  Author       : Marta Basquens. e-Tech Racing 2018-2019 ©
**  Target       : ADCAN PCB: PIC18F2685
**  Compiler     : PIC C Compiler 
**  Programmer   : PICKit3, Firmware Suite Version 01.25.20
**  Last Updated : 31 Jan 2019
** ***************************************************************************/

#include "ADCAN.h"
#include "ADCAN_vars.c"
#include "ADCAN_funs.c"

void main()
{
   Initial_Config();  
   delay_ms(5);

   enable_interrupts(INT_TIMER2);
   enable_interrupts(GLOBAL);
   /* Setup timer 2
   * On a 20 Mhz clock, this will trigger a timer2 interrupt every 1.0 ms
   * For time.h to work properly, Timer2 must overflow every millisecond
   * OverflowTime = 4 * (1/OscFrequency) * Prescale * Period * Postscale 
   * For 20 Mhz: .001 seconds  = 4 * (1/20000000 seconds) * 5 * 250 * 4
   */   
   setup_timer_2(T2_DIV_BY_4,250,5);

   delay_ms(3);

   enable_interrupts(int_canrx0);  

   while (true)
   {
      output_high(LED1); //led de comunicació del PIC

      if (RXB0CON.RXFUL == 1)
      {
         RXB0CON.RXFUL = 0;      // IF YOU DON'T NEED TO HANDLE THE MESSAGES
         //CANFLAG0=1;             // IF YOU WANT TO HANDLE THE MESSAGES
      }
      if (RXB1CON.RXFUL == 1)
      {
         RXB1CON.RXFUL = 0;      // IF YOU DON'T WANT TO HANDLE THE MESSAGES
         //CANFLAG1=1;             // IF YOU WANT TO HANDLE THE MESSAGES
      }


      // CAN SINCRONISM
      if (CAN_flag == 1)
      {
         CAN_flag = 0;
         contCAN++;

         if ( not_first_message == 1 )
         {  
            // SEND ALIVE
            if (contCAN >= 4)
            {
               send_alive(0xB0);
            }  

            send_data (0x85,8,APPS2,APPS1,SteeringSensor,BrakeSensor);
            send_data (0x86,4,SUSPFL,SUSPFR,0,0);

         } // END if (not_first_message == 1)         

         not_first_message = 1;

      } // END CAN_flag


      if ( read_ADC_flag>0 ) //every 1ms
      {    

         enable_interrupts(INT_TIMER2);
         read_ADC_flag = 0;


         /*APPS1_sum = get_adc_data(0,APPS1_sum);
         APPS2_sum = get_adc_data(1,APPS2_sum);
         BrakeS_sum = get_adc_data(2,BrakeS_sum);
         Steering_sum = get_adc_data(3,Steering_sum);
         SpringFR_sum = get_adc_data(10,SpringFR_sum);
         SpringFL_sum = get_adc_data(8,SpringFL_sum);*/


         // APP1
         set_adc_channel(0);
         delay_us(20);
         SUMA = read_adc();
         APPS1_sum += SUMA;

         // APP2
         set_adc_channel(1);
         delay_us(20);
         SUMA = read_adc();
         APPS2_sum += SUMA;

         // Brake           
         set_adc_channel(2);
         delay_us(20);
         SUMA = read_adc();
         BrakeS_sum += SUMA;

         // Steering
         set_adc_channel(3);
         delay_us(20);
         SUMA = read_adc();        
         Steering_sum += SUMA;

         // Spring FR
         set_adc_channel(10);
         delay_us(20);
         SUMA = read_adc();
         SpringFR_sum += SUMA;

         // Spring FL 
         set_adc_channel(8);
         delay_us(20);
         SUMA = read_adc();
         SpringFL_sum += SUMA;

         output_high(LED2); //led conversió ADC 

         if ( mean_filter_flag < 8 )
         {
            enable_interrupts(INT_TIMER2);
         }
         else //every 8 ms
         {
            enable_interrupts(INT_TIMER2);
            mean_filter_flag = 0;

   	      /*
   	      APPS1 = mean_filter(APPS1_sum); 
   	      APPS2 = mean_filter(APPS2_sum);
   	      BrakeSensor = mean_filter(BrakeS_sum);
   	      SteeringSensor = mean_filter(Steering_sum);
   	      SUSPFR = mean_filter(SpringFR_sum);
   	      SUSPFL = mean_filter(SpringFL_sum);
   			*/

            //filtre per cada senyal   
            APPS1_sum = APPS1_sum >> 3;           
            APPS1 = ((APPS1_sum<<8)&0xFF00)+((APPS1_sum>>8)&0x00FF);            
                  
   			APPS2_sum = APPS2_sum >> 3;            
   			APPS2 = ((APPS2_sum<<8)&0xFF00)+((APPS2_sum>>8)&0x00FF);               

   			BrakeS_sum = BrakeS_sum >> 3;
   			BrakeSensor = ((BrakeS_sum<<8)&0xFF00)+((BrakeS_sum>>8)&0x00FF);            

   			Steering_sum = Steering_sum >> 3;           
   			SteeringSensor = ((Steering_sum<<8)&0xFF00)+((Steering_sum>>8)&0x00FF);  
   			       
   			SpringFR_sum = SpringFR_sum >> 3;
   			SUSPFR = ((SpringFR_sum<<8)&0xFF00)+((SpringFR_sum>>8)&0x00FF);            

   			SpringFL_sum = SpringFL_sum >> 3;
   			SUSPFL = ((SpringFL_sum<<8)&0xFF00)+((SpringFL_sum>>8)&0x00FF);
   			
   			reset_variable(APPS1_sum, APPS2_sum, BrakeS_sum, Steering_sum, SpringFR_sum, SpringFL_sum, 0,0);
   			
         }  
      } // END if (read_adc_flag >0)

   } // END while(1)

} // END Main
