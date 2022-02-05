/**************************************************************************/
/*!
    @brief  Declaration of all the interrupts and functions used in ADCAN.c
*/
/**************************************************************************/

// INTERRUPT: CAN
#int_canrx0
void canrx0_int()
{
   // message identificator
   cobid = ((unsigned int16)RXB0SIDH << 3) | ((RXB0SIDL & 0xE0) >> 5); 
   // message length
   lengthCAN = (unsigned int8)RXB0DLC & 0xF;
   
   // sincronism
   if(cobid==0x80){ 
      CAN_flag = 1;
   }
 
   RXB0CON.RXFUL = 0;
} 

//  INTERRUPT: TIMER 2
#int_TIMER2
void  TIMER2_isr(void)
{
   disable_interrupts(INT_TIMER2); 
   read_ADC_flag = 1;
   mean_filter_flag++;
}

/**************************************************************************/
/*!
    @brief  Sets the initial configuration for the CAN protocol, disables
            all the interrupts present in the microcontroller as well as
            sets active the internal ADC
*/
/**************************************************************************/

void Initial_Config(void){ //configuració inicial del micro: 
   
   // configuration: CAN
   disable_interrupts(GLOBAL); // desabilitar els interruptors per si quedava algo obert del micro
   can_init(); //llibreria integrada de can
   
   can_set_mode(CAN_OP_CONFIG);
  
     //brp = 0 => 1000 kBaud
     //brp = 1 =>  500 kBaud
     //brp = 2 =>  333 kBaud
     //brp = 3 =>  250 kBaud
   
   //MIRAR DATASHEET del micro per saber q es cada cosa
   BRGCON1.brp=1;          // 5+2+2+1 = 10 
   BRGCON1.sjw=0;          // 1 TQ 
   BRGCON2.prseg=1;        // 2 TQ
   BRGCON2.seg1ph=4;       // 5 TQ
   BRGCON2.sam=FALSE; 
   BRGCON2.seg2phts=TRUE;  
   BRGCON3.seg2ph=1;       // 2 TQ
   BRGCON3.wakfil=FALSE;
   CIOCON = 0x20;

   can_set_mode(CAN_OP_NORMAL);
   
   // configuration: ADC
   setup_adc_ports(ALL_ANALOG|VSS_VDD);
   setup_adc(ADC_CLOCK_DIV_32|ADC_TAD_MUL_0);
   
   // configuration: WATCHDOG
   setup_wdt(WDT_OFF); //està off ja que només fa falta per programes llargs. Va plantejar més problemes activtat q desactivat   
}

/**************************************************************************/
/*!
    @brief  writes the alive message for the corresponding PCB to the CAN
            line
*/
/**************************************************************************/

void send_alive(unsigned int16 _cobid) //envia un senyal per dir q el node no esta penjat
{
   contCAN = 0;
   data = _alive;
   can_putd(_cobid,&data,1,0,0,0);
   _alive++; 
        
   delay_ms(2); 
   
   output_high(CAN); //led alive
}

/**************************************************************************/
/*!
    @brief  writes the sensor data to the CAN line (4 recordings at a time)
*/
/**************************************************************************/
//lo de _extra_data_n es equivalent a: _data_n, es una manera d'anomenar una data més
// enviar data -> cobid - longitud - data
void send_data (unsigned int16 _cobid, int _len, int16 _data_1, int16 _data_2, int16 _data_3, int16 _data_4)
{
   &data_bis = &data+32; 
   data_bis = make32(_data_3,_data_4);            
   data = make32(_data_1,_data_2);            
   can_putd(_cobid,&data,_len,0,0,0);
               
   delay_ms(2);
}

/**************************************************************************/
/*!
    @brief  applies a mean filter (8 measurements) to the specified sensor
*/
/**************************************************************************/
/* Com els sensors sempre tenen una petita desviació a part del filtre analogic també es posa un filtre
en el software, així els valors no varien tant */

/*int mean_filter (int32 _sensor)
{
   //_sensor = _sensor >> 3;
   sensor_mean = (((_sensor>>3)<<8)&0xFF00)+(((_sensor>>3)>>8)&0x00FF); 
   return sensor_mean;
   //sensor_mean=0;
}
*/
 int mean_filter (int32 _sensor)
 {
   _sensor = _sensor >> 3;
   int16 _sensor_mean = ((_sensor<<8)&0xFF00)+((_sensor>>8)&0x00FF); 
   return _sensor_mean;
}

/**************************************************************************/
/*!
    @brief  resets all the variables used in the mean_filter() calculation
*/
/**************************************************************************/

void reset_variable (int32 _sensor1, int32 _sensor2, int32 _sensor3, int32 _sensor4, int32 _sensor5, int32 _sensor6, int32 _sensor7, int32 _sensor8)
{
   _sensor1 = 0;
   _sensor2 = 0;
   _sensor3 = 0;
   _sensor4 = 0;
   _sensor5 = 0;
   _sensor6 = 0;
   _sensor7 = 0;
   _sensor8 = 0;
}

/**************************************************************************/
/*!
    @brief  read the adc port data and accumulates its value to the previous
            read sensor data in order to perform the mean_filter() calculation
*/
/**************************************************************************/

int get_adc_data (int _adc_channel, int32 _sensor1) //converteix dades analogiques a digitals, funció pròdpia del micro
{
   set_adc_channel(_adc_channel);
   delay_us(20);
   SUMA = read_adc();
   int32 _sensor2 = _sensor1 + SUMA;
   
   return _sensor2;
}

/*
int get_adc_data (unsigned int16 _adc_channel, int32 _sensor)
{
   set_adc_channel(_adc_channel);
   delay_us(20);
   SUMA = read_adc();
   _sensor += SUMA;
   
   return _sensor;
}*/
