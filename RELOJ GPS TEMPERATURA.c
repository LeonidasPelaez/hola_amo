//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                SISTEMA GPS - GSM                   $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//------------------------------------------------------
//          librerias y archivos necesarios 
//------------------------------------------------------
#include "18F4525.h"          // Microcontrolador con el que se va a trabajar
//#device *=16 HIGH_INTS=TRUE

#use delay( clock = 32 Mhz )

//#device WRITE_EEPROM = NOINT
#include "string.h"  // libreria para manejo de datos
#include "stdlib.h"
#include "math.h"

#include "1wire.c" 
#include "ds1820.c" 
//------------------------------------------------------
//          activar o desactivar fuses del PIC
//------------------------------------------------------
/*
//#Fuses INTRC_HP   // activar clock interno
#FUSES INTRC_IO    // activar clock interno HS Oscilator 16-25MHz
//#FUSES PLLEN       // 4X PLL enabled 
#Fuses PUT         // activar POWER UP TIMER 65.6 ms             
#FUSES NOMCLR      // desactivar Master Clear pin used for I/O
#Fuses PROTECT     // activar proteccion d e codigo
#FUSES NOWDT       // No Watch Dog Timer
//#FUSES WDT128      // Watch Dog Timer uses 1:128 Postscale
#FUSES NOXINST     // Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES NOFCMEN     // Fail-safe clock monitor disabled
#FUSES NOIESO      // Internal External Switch Over mode disabled
#FUSES NOBROWNOUT  // No brownout reset
//#FUSES NOWDT_SW      // No Watch Dog Timer, enabled in Software
#FUSES NOSTVREN    // Stack full/underflow will not cause reset
//#FUSES BBSIZ1K     // 1K words Boot Block size
//#FUSES BORV30    //
//#FUSES SOSC_DIG    // modo digital de pines rc0 rc1 y no como reloj secundario
*/

#FUSES INTRC_IO          //para cristal de 4MHz
#FUSES NOPROTECT   //lectura habilitada del codigo del pic
#fuses NOBROWNOUT  // Fusibles
#FUSES NOWDT       //No Watch Dog Timer
#fuses BORV27
#FUSES PUT         //power up timer habilitado
#FUSES NOCPD       //No PROTECTION
#FUSES NOSTVREN
#fuses NODEBUG     // No usar pines de depuración
#FUSES NOLVP       //No Watch Dog Timer
#FUSES NOWRTD
#fuses NOIESO
#fuses NOFCMEN
#FUSES NOPBADEN
#FUSES NOWRTC
#FUSES NOEBTR
#FUSES NOEBTRB
#FUSES NOCPB
#FUSES NOLPT1OSC
#FUSES NOMCLR        //No Watch Dog Timer 
#FUSES NOXINST

//------------------------------------------------------
//          configurar puerto serial RS232 
//------------------------------------------------------
//#use delay( clock = 32 Mhz )
//#use rs232( baud = 19200, parity=N, xmit = PIN_C6, rcv = PIN_C7, bits = 8, stream = modemGSM ) // RDA1
  #use rs232( baud = 9600, parity=N, xmit = PIN_C6, rcv = PIN_C7, bits = 8, stream = GPS )      // RDA2
//------------------------------------------------------
//          Definir pines de LCD y PIC 
//------------------------------------------------------
/*
#define _FLEX_LCD           // Aqui se pueden redefinir los pines de LCD 
   #define LCD_DB4 PIN_D4
   #define LCD_DB5 PIN_D5
   #define LCD_DB6 PIN_D6
   #define LCD_DB7 PIN_D7
   
   #define LCD_RS  PIN_D1
   #define LCD_RW  PIN_D2
   #define LCD_E   PIN_D3
#include "flex_lcd.c"          //  Libreria para el manejo del lcd
*/
//------------------------------------------------------
//          Definir puertos del PIC 
//------------------------------------------------------
#use fast_io( A )
#use fast_io( B )
#use fast_io( C )
#use fast_io( D )
#use fast_io( E )
//--------------------------------------------------------
//          Definiciones de pines
//--------------------------------------------------------
#define LUZ_LCD         LATA5
#define LED_PILOTO      LATA0
#define LED_Toggle      LED_PILOTO = !LED_PILOTO
#define BUZZER          LATA1
#define SISTEMA_ON_OFF  LATA2

#define ENA      LATB6
#define DATA     LATB7
#define STB      LATB5
#define CLOCK    LATB4

#define PUNTO_AUX    LATB2
//--------------------------------------------------------
//          Definir variables constantes
//--------------------------------------------------------
unsigned int8 const   Lin_adds[ 8 ] = {
                                         0x48,
                                         0x49,
                                         0x4a,
                                         0x4b,
                                         0x4c,
                                         0x4d,
                                         0x4e,
                                         0x4f
                                      };  

unsigned int8 const   Lin_data[ 8 ] = {       
                                         0b00010000,
                                         0b00011000,
                                         0b00011100,
                                         0b00011110,
                                         0b00011100,
                                         0b00011000,
                                         0b00010000,
                                         0b00000000
                                    };                      

unsigned int8  const   lenbuff = 100;
unsigned int8  const   Tiempo_Muestra_Reporte = 30;
unsigned int8  const   PUNTERO_ASCII = 29;

unsigned int8  const   Vel_Local_Min = 4;
unsigned int8  const   Vel_Servr_Min = 7;

unsigned int8  const   Time_Envio_Tcp = 2;
unsigned int16 const   Temporizador_Envio_Periodico = 1200; // 1200 segundos == 20 minutos
//--------------------------------------------------------
//          Definir variables globales 
//--------------------------------------------------------
unsigned int8   Hora_Inicio_Trabajo   = 0;
unsigned int8   Minuto_Inicio_Trabajo = 0;

unsigned int8   Hora_Final_Trabajo    = 0; 
unsigned int8   Minuto_Final_Trabajo  = 0;

unsigned int16  Radio_Pto_Cero        = 0;
unsigned int8   Radio_Pto_Ctrl        = 0;

float           Latitud_Cero_1_Float  = 0;
float           Longitud_Cero_1_Float = 0;

float           Latitud_Cero_2_Float  = 0;
float           Longitud_Cero_2_Float = 0;

unsigned int16  Tiempo_Inicio_Trabajo_Minutos = 0;      
unsigned int16  Tiempo_Final_Trabajo_Minutos  = 0;     

unsigned int8   Time_Envio  = 0;
unsigned int16  Time_Reset  = 0;
unsigned int16  Timer_Pausa = 0;
unsigned int16  Time_Pausa  = 0;
unsigned int16  Temporizador_Timer_INT = 0;
unsigned int16  Temporizador_Periodico = 1080;

char            IDE[ 8 ]     = {"BEI01"}; //ID Empresa y Buss
char            IDE_AUX[ 8 ] = {0}; //ID Empresa y Buss

unsigned int8   bool_mostrar_mensaje = 0,
                Estado_Configuracion,
                Estado_Puntos_Tiempos,
                Resultado_Estados,
                Estado_Activacion_Bus,
                pto_nro,
                Trabajar_Bool,
                Unidad,
                Decena,
                Numero,
                hora_decimal_leo,
                minuto_decimal_leo,
                segundo_decimal_leo,
                yuly,
                vel_8b = 0, 
                Velocidad_Bus, 
                Uni_Seg, 

                Signo_Lat,    // signo de latitud     + == lat. norte    - == latitud sur
                Signo_Lon,    // signo de longitud    + == long. este    - == long. oeste  

                Var_partida,
                FUNCIONA,
                //PUNTO,
                Tipo,

                segundos = 0,
                minutos = 0,
                segundero = 0,
                segundo_base,
                Temporizador_Buzzer = 0,

                intervalo = 1,
                diferencia_segundos,
                Tiempo_extra = 0,

                hora_decimal,
                minuto_decimal,
                segundo_decimal,                                 
                i,            // variable usado intensamente en bucles for
                j,

                Lim_Inf,         // limite inferior y superior
                Lim_Sup,         // se cargan con numero de posicion de cadena
                Size_String,     // tamaÃ±o de cadena a analizar
                Orden_Data,      // verifica que los datos se carguen en la cadena correcta
                xbuff = 0x00,    // puntero adjunto para cargar datos 
                bufferIndice = 0,
                Num_String,      // indica el numero de cadena donde se cargara los datos
                flagcommand = 0, // habilita o deshabilita el cargado de datos en las cadenas
                rcvchar2 = 0,                
                rcvchar1 = 0,

                Fmt_12h, 
                Bool_Paso_Pto_Partida = 0,
                Bool_Paso_Pto_Final   = 0,

                Var_1_8b,      // variable 1 auxiliar de 8 BITS para operaciones matematicas
                Var_2_8b,      // variable 2 auxiliar de 8 BITS para operaciones matematicas
                Var_3_8b,

                Bool_Mensaje,
                Timer_Mensaje = 0, 
                Time_Mensaje, 

                Num_Char_Mensaje = 0,
                Timer_Mensaje_1  = 0,
                Timer_Mensaje_2  = 0,
                Bucle_Mensaje    = 0,
                Num_Mensaje      = 1,
                Counter_8_bits   = 0,
                Flecha           = 0,

                Punto_a_Reenviar    = 0,
                Bool_Reenvia_Puntos = 0,              
               
                Temporizador_envio = 0,

                Hora_Partida_Pro,
                Minuto_Partida_Pro,

                Hora_Picada,
                Minuto_Picada,                
                
                Segundos_Mas = 0, 
                Minutos_Mas = 0,
    
                bool_Reporte_Time = 0, 
    
                //Minutos_Reporte,
                Segundos_Reporte,  
                contador_reporte_tempo,

                Signo_Reporte=0,
                Hora_Picada_Ant,
                Minuto_Picada_Ant,
                bool_llamada = 0,
                Temporizador_LLamada = 0,
                Direccion_Datos_Puntos_Control, 
                Direccion_Horas_Picada,
                Apagar_Sistema = 0,
                Ind_Fono = 0,
                Indice_Txt,
                Fun_Interrupt = 0, 
                Proceso_Int = 0, 
                S_Envio_TCP,
                flag_Envio_TCP,
                Estado_Reset = 0,

                DECENA_HORA_PARTIDA = 0,                 
                UNIDAD_HORA_PARTIDA = 0,

                DECENA_MINUTO_PARTIDA= 0, 
                UNIDAD_MINUTO_PARTIDA = 0,
                Recuperar_Tiempo_Reset = 0;

unsigned int16  r_flags = 0,
                Minutos_Reporte;

unsigned int8   flag_udatabus = 0;
unsigned int8   flag_upuntosc = 0;

unsigned int16  Minutero_Control,
                Temporizador_Reset, 
              
                Ref_Coordn,

                Var_1_16b,     // variable 1 auxiliar de 16 BITS para operaciones matematicas
                Var_2_16b,     // variable 2 auxiliar de 16 BITS para operaciones matematicas
                Var_3_16b,     // variable 3 auxiliar de 16 BITS para operaciones matematicas  

                Tiempo_Mas,
                Var_Tiempo_16b,
                Time_Reporte_16_bits, 
                Ind_Eeprom,
                Ind_String;

signed int32    Tiempo_Picada_Segundos,
                Tiempo_GPS_Segundos,
                Tiempo_Picada_Segundos_Ant,
                Var_1_32_bits,
                Var_2_32_bits,
                Var_3_32_bits;

unsigned int32  Var_1_32_bits_Postv;
                         
float           X_1,
                X_2,
                Y_1,
                Y_2,
                X,             // distancia en metros
                Y,
                X_Lat,
                Y_Long,
                Direccion_Float,
                pto_latitud=0,
                pto_longitud=0;
               
unsigned int8   string_0[ lenbuff ],   // hora latitud longitud 
                //string_1[ lenbuff ], // satelites
                //string_2[ lenbuff ], // hora latitud longitud
                string_3[ lenbuff ],   // velocidad

                Punto_ok[ 6 ]  = {"Pok"},
                horaok[ 6 ]    = {"cok"},
                Ideok[ 6 ]     = {"lok"},

                Hr_12h[ 10 ],
                Hora[ 10 ],
                Velocidad[ 20 ],       // almacena la velocidad
                Latitud[ 20 ],
                Longitud[ 20 ],
                Latitud_Respaldo[ 20 ],
                Longitud_Respaldo[ 20 ],
                POSICION[ 20 ],                   // guarda numero de posicion de las comas en una cadena
                
                HORA_PARTIDA[ 10 ] = { 0 },
                VEL_GSM[ 5 ]         = { 0 }, 
                C00RDENADAS[ 20 ]    = { 0 },     // almacena coordenadas y velocidad para envio al  servidor
                Puntos_Control[ 10 ] = { 0 },     // almacena ID y adjunta numero de punto para enviar al servidor

                Txt_pto_Ctrl[ 4 ],                // almacena temporalmente texto de paradero hasta escribir en eeprom
 
                Tiempos_Tramo[ 40 ][ 3 ] = { 0 }, // 120 B capacidad para almacenar hasta 40 tiempos   ( tiempos cortos, timepos largos, tiempos feriados )               
                
                Texto_Tramo[ 40 ][ 3 ]   = { 0 }, // 120 B

                Mensaje_String_1[ 20 ] = { 0 },
                Mensaje_String_2[ 20 ] = { 0 },

                Direccion_S[ 10 ]   =  { 0 },

                Puerto_Number[ 10 ] = { 0 },
                Horas_Picada_Prog[ 80 ]  = { 0 }, 
                Horas_Picada_Real[ 120 ] = { 0 }, 

                //CONFIG_GPS_9600[ 32 ]  = { "$PSRF100,1,9600,8,1,0*0D\r\n"  },
                //CONFIG_GPS_19200[ 32 ] = { "$PSRF100,1,19200,8,1,0*38\r\n" }, 

                operadorAPN[ 15 ] = {"movistar.pe"},
                //oprUSR[ 20 ]      = {"movistar@datos"},
                //oprPSS[ 15 ]      = {"movistar"},    
                servidor[25]      = {"www.hiledperu.com"},//{"hiled.no-ip.org"},
                puerto[10]        = { 0 },
                Puerto_Prueba[8]  = { "9027" },
                Ide_Prueba[8]     = { "TF96"};
    
float           Coordenadas_Puntos[ 40 ][ 2 ]   = { 0 };   // 320 B capacidad para almacenar hasta 40 puntos de control



//=====================================
//=====================================

  
float Temp_1_Float;
float Sum_Temp_1= 0;

//byte
int8
    Dia_Actual = 0,
    Dia_Anterior = 0,

    Hora_Actual = 50,
    hORA_Anterior = 0,

    Minuto_Actual = 0,
    Minuto_Anterior = 100,
    Estado_SD = 0,

    CONTADOR = 0,

    PUNTO = 0,
    TEMPERATURA = 0,
    //j,
    DEC_HORA,
    UNI_HORA,

    DEC_MIN,
    UNI_MIN,

    BIN_UNI_MIN,
    BIN_DEC_MIN,
    BIN_UNI_HORA,
    Binario,
    DE_MILL,
    UN_MILL,
    CENTENA,
    //DECENA,
    //UNIDAD,
    DEC_TEMP,
    UNI_TEMP,    
    BIN_UNI_TEMP,
    BIN_DEC_TEMP, 
    hora_lima_24,
    hora_lima_12,
    minuto_lima,
    cont_hora = 0,
    cont_temp = 0,
    Temp_Int8;

//bool
int1 
    h12,
    PM,
    toggle = 0;

int    ii=0;
int32  jj;     
int32  Registro_32_bits;


float           temperature_float;
unsigned int8   Temperatura_Int8;
 unsigned char Temperatura_String[ 10 ] = { 0 };

//--------------------------------------------------------
//            Definir Funciones Globales 
//--------------------------------------------------------
void setup( void );
void Inicializar_Buffers( void );
void Mostrar_Gps_Data( void );
void Procesar_Usart_Data( void );
void CALCULAR_DISTANCIA( void );

void CRONOMETRO( void );
void Hora_LLegada( void );

void Comprobar_Inicio_Trabajo( void ) ; 
void Enviar_Puntos( void ); 

void Back_Up_Hora_Partida_Eeprom( void );
void Back_Up_Hora_Llegada_Eeprom( void );
void Back_Up_Cronometro_Eeprom( void );
void Back_Up_Busqueda_Puntos_Eeprom( void );

void Read_Back_Up_Hora_Partida_Eeprom( void );
void Read_Back_Up_Hora_Llegada_Eeprom( void );
void Read_Back_Up_Cronometro_Eeprom( void );
void Read_Back_Up_Busqueda_Puntos_Eeprom( void );

void Mostrar_Inicio_Normal( void );
void Mostrar_Puntos_Tiempos_Texto( void );
void Leer_Mostrar_Config( void );
void Leer_Config( void );
void Default_Config( void );

void Buscar_Ptos_Partida( void );
void Buscar_Ptos_Final( void );

void Calculo_Tiempos_Varios( void );
void Set_Var_Pto_Ctrl( void );
void Calculo_Reporte_Final( void );

void Read_Data_Eeprom_Ptos_Ctrl( void );

void Read_Back_Up_Data( void );
void Back_Up_Data( void );

void Crear_Horas_Picada_Programada( void );
void Inicializar_Variables_Sistema( void );

void Muestra_Display_Hora( void );
void Muestra_Display_Temperatura( void );
int8 Num_to_Binario( int8 );
 

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$                rda2_isr()                  $$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#int_RDA 
void rda_isr()// interrupcion de usart para recepcion de datos del GPS
{   
    rcvchar2 = 0;  
    rcvchar2 = fgetc( GPS ); 
    Procesar_Usart_Data( );
}

//--------------------------------------------------------------------------------------------------------
//$$$$$$$$$$$$$$$                               #int_RTCC HIGH                             $$$$$$$$$$$$$$$
//--------------------------------------------------------------------------------------------------------
#int_RTCC            // Para velocidad de reloj de 64 Mhz
void  RTCC_isr( void )      // entra a esta rutina cada 3.091 ms
{
    Temporizador_Buzzer++; 
    
    if( Temporizador_Buzzer > 61 )
      { 
          Temporizador_Buzzer = 0; 
          PUNTO_AUX = !PUNTO_AUX; 
      } 
}
//--------------------------------------------------------------------------------------------------------
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                PROGRAMA - PRINCIPAL                $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void main( )
{
   setup( );
   
   while(TRUE)
        {  
            // esta rutina se ejecuta si y solo   
            // si el GPS esta activado 
                                   
                  if( flagcommand == 1 ) 
                    { 
                        enable_interrupts( INT_RTCC );  
                        Mostrar_Gps_Data( ); 
                        Muestra_Display_Hora(  );
                        Inicializar_Buffers( ); 
                        
                        
                        cont_hora++;
                        if( cont_hora > 5 )
                          {
                              cont_hora = 0;
                              PUNTO_AUX = 0;
                              disable_interrupts( INT_RTCC ); 
                              
                              for( cont_temp = 0; cont_temp < 3; cont_temp++ )
                                 {
                                     
                                     temperature_float = ds1820_read(); 
                                     Temp_Int8 = (int8)temperature_float;
                                     Muestra_Display_Temperatura(  );
                                 }
                          }
                        
                       flagcommand = 0; 
                    } 
        }
} 
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                       FUNCIONES                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//--------------------------------------------------------------------------------------------------------
//$$$$$$$$$$$$$$$                                   setup( )                               $$$$$$$$$$$$$$$
//--------------------------------------------------------------------------------------------------------
void setup( )
{
    setup_oscillator(OSC_32MHZ);
    
    LATA = 0X00; 
    LATB = 0X00; 
    LATC = 0b00100000;  //0X00; 
    LATD = 0X00; 
    LATE = 0X00;     
    //---------------------------------------------------------------------------  
    // configuracion iniciail de estado de
    // perifericos y temporizadores 
    disable_interrupts( global );       // Desabilitar todas las interrupciones
    disable_interrupts( int_timer0 );   // Desabilitar interrupciones del timer0
    disable_interrupts( int_timer1 );   // Desabilitar interrupciones del timer1
    disable_interrupts( int_timer2 );   // Desabilitar interrupciones del timer2
    disable_interrupts( int_timer3 );   // Desabilitar interrupciones del timer3
    //disable_interrupts( int_timer4 );   // Desabilitar interrupciones del timer4
    disable_interrupts( int_rda );      // Desabilitar todas las interrupciones
    //disable_interrupts( int_rda2 );     // Desabilitar todas las interrupciones
    disable_interrupts( int_ext );      // Desabilitar interrupcion externa INT0
    disable_interrupts( int_ext1 );     // Desabilitar interrupcion externa INT1
    disable_interrupts( int_ext2 );     // Desabilitar interrupcion externa INT2
    setup_adc_ports   ( NO_ANALOGS );   // Desabilitar entradas analogicas
    setup_adc         ( ADC_OFF) ;      // Desabilitar el periferico ADC
    //setup_spi         ( FALSE );        // Desabilitar el perferico SPI
    //setup_psp         ( PSP_DISABLED ); // Desabilitar el perferico PSP
    setup_comparator  ( NC_NC_NC_NC );  // Desabilitar el perferico COMPARADOR
    port_b_pullups    ( false );        // Resistencias PULL_UP activadas PORT_B
    setup_uart        ( false );        // periferico USART desactivaso
    delay_us          ( 10 );           // Espera a que se activen. 
    //---------------------------------------------------------------------------- 
    set_tris_a( 0b00000000 );           
    set_tris_b( 0b00000000 );
    set_tris_c( 0b10100000 );
    set_tris_d( 0b00000000 );
    set_tris_e( 0b00000000 );
    
    SISTEMA_ON_OFF = 0;
    LED_PILOTO     = 0;      
    BUZZER         = 0;
    LUZ_LCD        = 1; 
    
    ENA = 0;

    delay_ms( 50 );
    Inicializar_Buffers( );
    Num_String    = 0;
    flagcommand   = 0; 
    Orden_Data    = 0;
    
    setup_uart( true ); 
    enable_interrupts( INT_RDA ); 
    setup_timer_0( T0_8_BIT | RTCC_INTERNAL | RTCC_DIV_256 );  // 13.056ms
    //enable_interrupts( INT_RTCC ); 
    enable_interrupts( GLOBAL ); 

    STB = 0;
    DATA = 0;    
    for( ii = 24; ii != 0; ii-- )
       { 
          CLOCK = 0;                 
          CLOCK = 1; 
       }
    STB = 1;   

    DATA = 1;
    
    for( ii = 24; ii != 0; ii-- )
       { 
           STB = 0;   
           CLOCK = 0;
           CLOCK = 1; 
           STB = 1;
           //DATA = 0;  
           delay_ms( 130 );
       } 
}

//--------------------------------------------------------------------------------------------------------
//$$$$$$$$$$$$$$$                             Procesar_Data( )                             $$$$$$$$$$$$$$$
//--------------------------------------------------------------------------------------------------------
// ordenas los datos recibidos por la USART en sus respectivas 
// cadenas segun el protocolo NMEA
void Procesar_Usart_Data( ) 
{
 
    if( flagcommand == 1 ) goto salir_leo;

    switch( Num_String )
          {
              case 0: switch( Orden_Data )// hora latitud longitud
                            {
                                case 0: if( rcvchar2 == '$') Orden_Data = 1; else{ Orden_Data = 0;} break;
                                case 1: if( rcvchar2 == 'G') Orden_Data = 2; else Orden_Data = 0; break;         
                                case 2: if( rcvchar2 == 'P') Orden_Data = 3; else Orden_Data = 0; break;
                                case 3: if( rcvchar2 == 'G') Orden_Data = 4; else Orden_Data = 0; break; 
                                case 4: if( rcvchar2 == 'G') Orden_Data = 5; else Orden_Data = 0; break;         
                                case 5: if( rcvchar2 == 'A') Orden_Data = 6; else Orden_Data = 0; break;
                                case 6: String_0[ xbuff ] = rcvchar2;
                                        xbuff++;
                                        if( xbuff >= lenbuff ) xbuff = 0;

                                        if( rcvchar2 == 10 || rcvchar2 == 13 )
                                          { 
                                              Num_String = 1; 
                                              xbuff      = 0; 
                                              Orden_Data = 0;   
                                          }                                        
                                        break;  
                            } break;                          
                      
              case 1: switch( Orden_Data )//velocidad
                            {
                                case 0: if( rcvchar2 == '$') Orden_Data = 1; else Orden_Data = 0; break;
                                case 1: if( rcvchar2 == 'G') Orden_Data = 2; else Orden_Data = 0; break;         
                                case 2: if( rcvchar2 == 'P') Orden_Data = 3; else Orden_Data = 0; break;
                                case 3: if( rcvchar2 == 'V') Orden_Data = 4; else Orden_Data = 0; break; 
                                case 4: if( rcvchar2 == 'T') Orden_Data = 5; else Orden_Data = 0; break;         
                                case 5: if( rcvchar2 == 'G') Orden_Data = 6; else Orden_Data = 0; break;
                                case 6: String_3[ xbuff ] = rcvchar2;
                                        xbuff++; 
                                        if( xbuff >= lenbuff ) xbuff = 0;

                                        if( rcvchar2 == 10 || rcvchar2 == 13 )
                                          { 
                                              Num_String  = 0; 
                                              xbuff       = 0; 
                                              Orden_Data  = 0; 
                                              flagcommand = 1;
                                          }
                                        break;  
                            } break;                                  
          }           
    salir_leo: return; 
}
//--------------------------------------------------------------------------------------------------------
//$$$$$$$$$$$$$$$                              Mostrar_Data( )                             $$$$$$$$$$$$$$$
//--------------------------------------------------------------------------------------------------------
void Mostrar_Gps_Data( )   
{    
    LED_PILOTO = 0;    
    //----------------------------------------------------------------------------------------
    // en este code obtenemos coordenadas 
    // y hora y las mostramos en LCD
    for( i = 0; i < 20; i++ ) POSICION[ i ] = 0;
    Size_String = strlen(String_0);  
    // GPGGA de aqui obtenemos la hora, latitud y longitud
    for( i = 0, j=0; i< Size_String && j < 12; i++ )
       { 
             if( String_0[i] == ',')
                 { 
                    POSICION[ j ] = i;  
                    j++; 
                 } 
       } 

    Lim_Inf = POSICION[ 0 ] + 1; 
    Lim_Sup = POSICION[ 1 ];    
    for( i = Lim_Inf; i< Lim_Sup; i++ )
       { 
             Hora[ i - Lim_Inf ] = String_0[ i ]; 
       } 

    Decena =  Hora[ 4 ];                    //  SEGUNDOS  GPS
    Decena -= 48; 
    Unidad =  Hora[ 5 ]; 
    Unidad -= 48;         
    segundo_decimal = Decena*10 + Unidad;   //  MINUTOS GPS
    segundo_decimal_leo = segundo_decimal; 
    Decena =  Hora[ 2 ]; 
    Decena -= 48;
    Unidad =  Hora[ 3 ]; 
    Unidad -= 48; 
    minuto_decimal = Decena*10 + Unidad; 
    minuto_decimal_leo = minuto_decimal;
    minuto_lima = minuto_decimal_leo;
    Decena =  Hora[ 0 ];                    //  HORA  GPS
    Decena -= 48; 
    Unidad =  Hora[ 1 ]; 
    Unidad -= 48;   
    Numero =  Decena*10 + Unidad;
    
    
    if( Numero < 5 ){ hora_lima_24 = Numero + 19; }  // formateamos solo la hora a nuestro meridiano
      else{ hora_lima_24 = Numero - 5; }
                            
    if( hora_lima_24 > 12 ) hora_lima_12 = hora_lima_24 - 12;
    else if( hora_lima_24 == 0 ) hora_lima_12 = 12;
    else hora_lima_12 = hora_lima_24; 
                          
    
    if( Numero < 5 ){ Numero = 19 + Numero; }  // formateamos solo la hora a nuestro meridiano
      else{ Numero -= 5; } // 5 horas de diferencia con el MGT
    
    hora_decimal     = Numero; 
    hora_decimal_leo = hora_decimal;

    Decena =  Numero/10; 
    Decena += 48; 
    Unidad =  Numero%10; 
    Unidad += 48; // y luego lo cargamos en su posicion 
    
    Hora[ 0 ] = Decena; // formateamos la hora para mostrar en el LCD
    Hora[ 1 ] = Unidad; //
    Hora[ 7 ] = Hora[ 5 ]; 
    Hora[ 6 ] = Hora[ 4 ]; 
    Hora[ 5 ] = ':'; 
    Hora[ 4 ] = Hora[ 3 ]; 
    Hora[ 3 ] = Hora[ 2 ]; 
    Hora[ 2 ] = ':'; 
    Hora[ 8 ] = 0; // NULL STRING para mandar a LCD

    Hr_12h[ 0 ] = Hora[ 0 ]; 
    Hr_12h[ 1 ] = Hora[ 1 ]; 
    Hr_12h[ 2 ] = Hora[ 2 ]; 
    Hr_12h[ 3 ] = Hora[ 3 ];
    Hr_12h[ 4 ] = Hora[ 4 ];                 
    Hr_12h[ 5 ] = Hora[ 5 ];
    Hr_12h[ 6 ] = Hora[ 6 ];
    Hr_12h[ 7 ] = Hora[ 7 ];
    Hr_12h[ 8 ] = Hora[ 8 ];

    Fmt_12h     = ( Hr_12h[ 0 ] - 48 )*10 + ( Hr_12h[ 1 ] - 48 );

    if( Fmt_12h > 12 ) 
      {
          Fmt_12h = Fmt_12h -12;
          Hr_12h[ 0 ] = ( Fmt_12h / 10 ) + 48;          
          Hr_12h[ 1 ] = ( Fmt_12h % 10 ) + 48;

          if( Hr_12h[ 0 ] == '0' ) Hr_12h[ 0 ] = ' ';
      }  
    
    //---------------------------------------------------------------------------- 
    // de este code obtenemos coordenadas en tiempo  
    // real del GPS y la formateamos TIPO GOOGLE MAPS
    Lim_Inf = POSICION[ 1 ] + 1;                
    Lim_Sup = POSICION[ 2 ];                    
    Signo_Lat = String_0[ Lim_Sup + 1 ];       
    for( i = Lim_Inf; i< Lim_Sup; i++ )         
       { 
           Latitud[ i - Lim_Inf ] = String_0[i];
           Latitud_Respaldo[ i - Lim_Inf ] = String_0[i];
       }   
    
    Lim_Inf = POSICION[ 3 ] + 1;
    Lim_Sup = POSICION[ 4 ];
    Signo_Lon = String_0[ Lim_Sup + 1 ];    
    for( i = Lim_Inf; i < Lim_Sup; i++ )
       { 
           Longitud[ i - Lim_Inf ] = String_0[ i ];
           Longitud_Respaldo[ i - Lim_Inf ] = String_0[ i ];
       }   
    
    if( Signo_Lat == 'S') Signo_Lat = '-';
      else Signo_Lat = '+';
      
    if( Signo_Lon == 'W') Signo_Lon = '-';
      else Signo_Lon = '+';
    
    Ref_Coordn = (Latitud[ 0 ] - 48)*10 + (Latitud[ 1 ] - 48);
    Ref_Coordn = Ref_Coordn*100;

    X_2 = atof( Latitud );
    X_2 = X_2 - Ref_Coordn;
    X_2 = fabs(X_2);
    X_2 = X_2/60;
    Ref_Coordn = Ref_Coordn/100;
    X_2 = X_2 + Ref_Coordn;
    sprintf( Latitud,"%2.4f",X_2 );
    
    Ref_Coordn = (Longitud[ 1 ] - 48)*10 + (Longitud[ 2 ] - 48);
    Ref_Coordn = Ref_Coordn*100;

    Y_2 = atof( Longitud );
    Y_2 = Y_2 - Ref_Coordn;
    Y_2 = fabs(Y_2);
    Y_2 = Y_2/60;
    Ref_Coordn = Ref_Coordn/100;
    Y_2 = Y_2 + Ref_Coordn;
    sprintf( Longitud,"%2.4f",Y_2 ); 
    //--------------------------------------------------------------------------------------------------------
    // GPVTG de aqui obtenemos la velocidad y la mostramos en LCD
    // tambien se obtiene el curso de navegacion del vehiculo
    Size_String = strlen(String_3);    
    for( i = 0, j=0; i< Size_String && j < 8; i++ )
       { 
             if( String_3[i] == ',')
                 { 
                    POSICION[ j ] = i;  
                    j++; 
                 } 
       }
    // se obtine curso de navegacion
    Lim_Inf = POSICION[ 0 ] + 1; 
    Lim_Sup = POSICION[ 1 ];    
    for( i = Lim_Inf; i< Lim_Sup; i++ )
       {
           Direccion_S[ i - Lim_Inf ] = String_3[ i ]; 
       } 

    Direccion_S[ i - Lim_Inf ] = 0;  
    Direccion_Float = atof( Direccion_S );  

    if( Direccion_Float >= 0   && Direccion_Float <  45  ) Flecha = 0;
    if( Direccion_Float >= 45  && Direccion_Float <  90  ) Flecha = 1;
    if( Direccion_Float >= 90  && Direccion_Float <  135 ) Flecha = 2;
    if( Direccion_Float >= 135 && Direccion_Float <  180 ) Flecha = 3;
    if( Direccion_Float >= 180 && Direccion_Float <  225 ) Flecha = 4;
    if( Direccion_Float >= 225 && Direccion_Float <  270 ) Flecha = 5;
    if( Direccion_Float >= 270 && Direccion_Float <  315 ) Flecha = 6;
    if( Direccion_Float >= 315 && Direccion_Float <= 360 ) Flecha = 7;
   
    // se obtiene velocidad 
    Lim_Inf = POSICION[ 6 ] + 1;
    Lim_Sup = POSICION[ 7 ];
    for( i = Lim_Inf; i < Lim_Sup; i++ ) // Velocidad 
       {
           Velocidad[ i - Lim_Inf ] = String_3[i]; 
       }  

    X = atof( Velocidad );    
    vel_8b = ( int8 ) X; 

    Velocidad_Bus = vel_8b;
    if( Velocidad_Bus < Vel_Local_Min ) Velocidad_Bus = 0;
    //--------------------------------------------------------------------------------------------------------
    
      
           //   lcd_gotoxy( 1, 1 ); 
           //   printf( lcd_putc,"%s  ",Hr_12h );

           //   lcd_gotoxy(11,1); 
              /*if( bool_Reporte_Time == 1 )
                {
                   contador_reporte_tempo++;
                   if( contador_reporte_tempo >= Tiempo_Muestra_Reporte ) 
                     {
                          bool_Reporte_Time = 0;
                          contador_reporte_tempo = 0;
                     }

                   if( Signo_Reporte == 1 ) printf( lcd_putc,"-%Lu:%02d  ", Minutos_Reporte, Segundos_Reporte );  
                     else printf( lcd_putc,"+%Lu:%02d   ", Minutos_Reporte, Segundos_Reporte );                    
                }*/
         //      printf(lcd_putc,"%2dkm/h  ", Velocidad_Bus );  
   
}
//--------------------------------------------------------------------------------------------------------
//$$$$$$$$$$$$$$$                              Ini_Strings( )                              $$$$$$$$$$$$$$$
//--------------------------------------------------------------------------------------------------------
void Inicializar_Buffers( )
{ 
    xbuff = 0;   
    for( i = 0; i < 12; i++ ) Hora[ i ]  = 0;
    for( i = 0; i < 10; i++ ) Velocidad[ i ] = 0;
    for( i = 0; i < 15; i++ ) Latitud[ i ] = 0;
    for( i = 0; i < 15; i++ ) Longitud[ i ] = 0;
    for( i = 0; i < 15; i++ ) Latitud_Respaldo[ i ] = 0;
    for( i = 0; i < 15; i++ ) Longitud_Respaldo[ i ] = 0;     
}
//--------------------------------------------------------------------------------------------------------
//$$$$$$$$$$$$$$$                            CALCULAR_DISTANCIA                            $$$$$$$$$$$$$$$
//--------------------------------------------------------------------------------------------------------
void CALCULAR_DISTANCIA(void)
{
    X_1 = X_Lat;
    Y_1 = Y_Long;
    X_1 = X_1*111.111;              
    Y_1 = Y_1*108.2633;   

    Ref_Coordn = (Latitud_Respaldo[ 0 ] - 48)*10 + (Latitud_Respaldo[ 1 ] - 48);
    Ref_Coordn = Ref_Coordn*100;
    X_2 = atof( Latitud_Respaldo );           
    X_2 = X_2 - Ref_Coordn;
    X_2 = fabs(X_2);
    X_2 = X_2/60;
    Ref_Coordn = Ref_Coordn/100;
    X_2 = X_2 + Ref_Coordn;
    X_2 = X_2*111.111; 

    Ref_Coordn = (Longitud_Respaldo[ 1 ] - 48)*10 + (Longitud_Respaldo[ 2 ] - 48);
    Ref_Coordn = Ref_Coordn*100;

    Y_2 = atof( Longitud_Respaldo );              
    Y_2 = Y_2 - Ref_Coordn;
    Y_2 = fabs(Y_2);
    Y_2 = Y_2/60;
    Ref_Coordn = Ref_Coordn/100;
    Y_2 = Y_2 + Ref_Coordn;
    Y_2 = Y_2*108.2633; 

    X   = X_2 - X_1;
    X   = fabs(X);
    X   = X*X;              
    Y   = Y_2 - Y_1;
    Y   = fabs(Y);
    Y   = Y*Y;              
    X   = X + Y;
    X   = sqrt( X ); 
    X   = X*1000;     
}     

//--------------------------------------------------------------
//$$$$$$$$$$$                Read_Back_Up_Data       $$$$$$$$$$$
//--------------------------------------------------------------
 void Inicializar_Variables_Sistema( )
{
    Num_String    = 0;
    flagcommand   = 1; 
    Orden_Data    = 0; 
    Trabajar_Bool = 1;
    Tipo          = 0; 
    Var_partida   = 'N'; 
    Estado_Reset  = 0;
    Recuperar_Tiempo_Reset = 0;

    PUNTO = 0;
    Bool_Mensaje        = 0;
    Bool_Reenvia_Puntos = 0;
    Timer_Mensaje       = 0;
    Num_Char_Mensaje    = 0;

    Temporizador_Reset    = 0;   
    Estado_Activacion_Bus = 0;    // siempre mandar punto cero en inicio normal    
    FUNCIONA              = 0; 
} 

void Muestra_Display_Hora(  )
{
    DEC_HORA = hora_lima_12/10;
    UNI_HORA  = hora_lima_12%10;

    DEC_MIN = minuto_lima/10;
    UNI_MIN  = minuto_lima%10;
    
    BIN_UNI_MIN  = Num_to_Binario( UNI_MIN );
    BIN_DEC_MIN  = Num_to_Binario( DEC_MIN );
    if( PUNTO == 1 ) { BIN_DEC_MIN = BIN_DEC_MIN | 0b10000000; }
    
    BIN_UNI_HORA = Num_to_Binario( UNI_HORA );
    if( DEC_HORA == 1 ) { BIN_UNI_HORA = BIN_UNI_HORA | 0b10000000; }

    Registro_32_bits = BIN_UNI_HORA;    
    Registro_32_bits = Registro_32_bits << 8;
    Registro_32_bits = Registro_32_bits | BIN_DEC_MIN;
    Registro_32_bits = Registro_32_bits << 8;
    Registro_32_bits = Registro_32_bits | BIN_UNI_MIN;
    
    //-------------------------------------------------------
    STB = 0;    
    
    for( ii = 24; ii != 0; ii-- )
       {
           if( bit_test(Registro_32_bits,(ii-1))) DATA = 1;
             else DATA = 0;
           
           CLOCK = 0;           
           CLOCK = 1;           
       } 

    STB = 1;          
}

void Muestra_Display_Temperatura(  )
{
    UNI_HORA  = Temp_Int8/10;

    DEC_MIN = Temp_Int8%10;    
    UNI_MIN  = minuto_lima%10;
    
    BIN_UNI_MIN  = 0b01100011;
    BIN_DEC_MIN  = Num_to_Binario( DEC_MIN );
    
    BIN_UNI_HORA = Num_to_Binario( UNI_HORA );

    Registro_32_bits = BIN_UNI_HORA;    
    Registro_32_bits = Registro_32_bits << 8;
    Registro_32_bits = Registro_32_bits | BIN_DEC_MIN;
    Registro_32_bits = Registro_32_bits << 8;
    Registro_32_bits = Registro_32_bits | BIN_UNI_MIN;
    
    //-------------------------------------------------------
    STB = 0;    
    
    for( ii = 24; ii != 0; ii-- )
       {
           if( bit_test(Registro_32_bits,(ii-1))) DATA = 1;
             else DATA = 0;
           
           CLOCK = 0;
           CLOCK = 1;           
       } 

    STB = 1;          
}

int8 Num_to_Binario( int8 Numero )
{
    switch( Numero )
          {      
            //-gfedcba
              case 0: Binario = 0b00111111; break;
              case 1: Binario = 0b00000110; break;
              case 2: Binario = 0b01011011; break;
              case 3: Binario = 0b01001111; break;
              case 4: Binario = 0b01100110; break;
              case 5: Binario = 0b01101101; break;
              case 6: Binario = 0b01111101; break;
              case 7: Binario = 0b00000111; break;
              case 8: Binario = 0b01111111; break;  
              case 9: Binario = 0b01101111; break;                   
          }
     return(Binario);
}
