/**************************************************************************/
/*       Declaracio de totes les funcions utilitzades en ADCAN.c         */
/**************************************************************************/

// INTERRUPT: CAN
#int_canrx0
void canrx0_int()
{
   cobid = ((unsigned int16)RXB0SIDH << 3) | ((RXB0SIDL & 0xE0) >> 5); // identificador de missatge
   lengthCAN = (unsigned int8)RXB0DLC & 0xF; //llargada del missatge
   
   // CAN sincronism
   if(cobid==0x80)
   { 
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

/************************************************************************************/
/*  Configuracio inicial pel protocol de CAN: desabilita interruptors i activa ADC  */
/************************************************************************************/

void Initial_Config(void){ //configuració inicial del micro 
   
   // configuracio CAN
   disable_interrupts(GLOBAL); // desabilitar els interruptors per si quedava algo obert del micro
   can_init(); //llibreria integrada de can
   
   can_set_mode(CAN_OP_CONFIG);
  
   BRGCON1.brp=1;     // 500 kBaud
   BRGCON1.sjw=0;         
   BRGCON2.prseg=1;        
   BRGCON2.seg1ph=4;       
   BRGCON2.sam=FALSE; 
   BRGCON2.seg2phts=TRUE;  
   BRGCON3.seg2ph=1;       
   BRGCON3.wakfil=FALSE;
   CIOCON = 0x20;

   can_set_mode(CAN_OP_NORMAL);
   
   // configuracio ADC
   setup_adc_ports(ALL_ANALOG|VSS_VDD);
   setup_adc(ADC_CLOCK_DIV_32|ADC_TAD_MUL_0);
}

/**************************************************************************/
/*                Envia per CAN el missatge alive del ADCAN                         */
/**************************************************************************/

void send_alive(unsigned int16 _cobid) //envia senyal per indicar que el node no esta penjat
{
   contCAN = 0;
   data = _alive;
   can_putd(_cobid,&data,1,0,0,0);
   _alive++; 
        
   delay_ms(2); 
   
   output_high(CAN); //led alive
}

/*************************************************************************/
/*          Envia les dades dels sensors per CAN (4 dades/envio)              */
/*************************************************************************/

void send_data (unsigned int16 _cobid, int _len, int16 _data_1, int16 _data_2, int16 _data_3, int16 _data_4)
{
   &data_bis = &data+32; 
   data_bis = make32(_data_3,_data_4);            
   data = make32(_data_1,_data_2);            
   can_putd(_cobid,&data,_len,0,0,0);
               
   delay_ms(2);
}
