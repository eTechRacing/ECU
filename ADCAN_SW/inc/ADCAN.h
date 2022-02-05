#include <18F2685.h> //libreria microcontrolador
#device adc=10 //modulo conversor análogo digital de 8 a 10 bits selecionables por el programador.

// Libreria CAN
#define CAN_USE_EXTENDED_ID FALSE //utilizamos la trama de CAN standard (FALSE). La extendida seria TRUE.
#include <can-18xxx8.c>

// REGISTRES LEDs: indica el nombre de los seguientes pines, que concuerdan con el esquematico
#define CAN       PIN_C3 // pin 14
#define LED1      PIN_C1 // pin 12
#define LED2      PIN_C2 // pin 13
/* segun datasheet: PIN_A0 a PIN_A5 => pin2 a pin7 respectivamente;
					PIN_B0 a PIN_B7 => pin21 a pin28 respectivamente;
					PIN_C0 a PIN_C7 => pin11 a pin18 respectivamente;*/

//configura fusibles de configuración
#FUSES NOWDT                    //No Watch Dog Timer; (El Watch Dog timer evita que los microprocesadores se queden colgados)
#FUSES WDT128                   //Watch Dog Timer uses 1:128 Postscale
#FUSES HS                       //High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD)
#FUSES NOPROTECT                //Code not protected from reading
#FUSES NOBROWNOUT               //No brownout reset
//#FUSES BORV20                   //Brownout reset at 2.0V
#FUSES NOPUT                    //No Power Up Timer
#FUSES NOCPD                    //No EE protection
#FUSES STVREN                   //Stack full/underflow will cause reset
#FUSES NODEBUG                  //No Debug mode for ICD
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NOWRT                    //Program memory not write protected
#FUSES NOWRTD                   //Data EEPROM not write protected
#FUSES IESO                     //Internal External Switch Over mode enabled
#FUSES FCMEN                    //Fail-safe clock monitor enabled
#FUSES PBADEN                   //PORTB pins are configured as analog input channels on RESET
#FUSES BBSIZ4K                  //4K words Boot Block size
#FUSES NOWRTC                   //configuration not registers write protected
#FUSES NOWRTB                   //Boot block not write protected
#FUSES NOEBTR                   //Memory not protected from table reads
#FUSES NOEBTRB                  //Boot block not protected from table reads
#FUSES NOCPB                    //No Boot Block code protection
#FUSES LPT1OSC                  //Timer1 configured for low-power operation
#FUSES NOMCLR                   //Master Clear pin used for I/O
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)

#use delay(clock=20000000)    // velocidad del reloj a la que queremos trabajar, en este caso f=20MHz
#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8) 
/* posibilita la comunicación del PIC con otro dispositivo utilizando el protocolo de comunicación serie RS232,
   nos permite formatear la salida de esos datos de la forma que nosotros queramos.
   - BAUD: para establecer la velocidad en baudios a la que queremos que se transmitan los datos por el puerto serie, 9600 es lo normal.
   - BITS: núm de bits que utilizaremos en la transmisión, estándar es 8 o 9. Para comunicación con microcontroladores 8 son suficientes.
   - PARITY: nos permite utilizar un bit de paridad para la comprobación de errores, está opción la dejamos a No=>N.
   - XMIT: configura por qual patilla del PIC saldrán los datos, se tendrá que cambiar a nuestras necesidades.
   - RCV: configura por qual patilla del PIC se recibirán los datos, se tendrá que cambiar a nuestras necesidades. */


