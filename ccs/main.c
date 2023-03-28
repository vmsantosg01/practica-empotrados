//*****************************************************************************
//
// Codigo de partida comunicacion TIVA-QT (Abril2021)
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
//  basada en la TIVA, en las que existe un intercambio de mensajes con un interfaz
//  grÃ¡fico (GUI) Qt.
//  La aplicacion se basa en un intercambio de mensajes con ordenes e informacion, a traves  de la
//  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
//  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
//   En el ejemplo basico de partida se implementara la recepcion de un mensaje
//  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
//  mensaje enviado desde la placa al GUI, para mostrar el estado de los botones.
//
//*****************************************************************************
#include<stdbool.h>
#include<stdint.h>
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADC
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de timers
#include "driverlib/pwm.h"          // NHR: Libreria PWM
#include <utils/uartstdioMod.h>     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
#include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
#include "timers.h"              // FreeRTOS: definiciones relacionadas con timers
#include "utils/cpu_usage.h"
#include "commands.h"
#include <serial2USBprotocol.h>
#include <usb_dev_serial.h>
#include "usb_messages_table.h"


//DEFINICION MACROS

#define DIV_RELOJ_PWM   2
#define PERIOD_PWM (SysCtlClockGet()*0.0025)/DIV_RELOJ_PWM //Periodo = 0.02 ms

// Definiciones de tareas
#define PWMTASKPRIO 1           // Prioridad para la tarea PWMTASK
#define PWMTASKSTACKSIZE 128    // Tamaño de pila para la tarea PWMTASK
#define VELTASKPRIO 1           // Prioridad para la tarea PWMTASK
#define VELTASKSTACKSIZE 128    // Tamaño de pila para la tarea PWMTASK

// Variables globales "main"
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;


SemaphoreHandle_t semaforo_freertos2;
TimerHandle_t xTimer;
QueueHandle_t cola_freertos_mot;


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1);
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
    static uint8_t ui8Count = 0;

    if (++ui8Count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        ui8Count = 0;
    }
    //return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
    SysCtlSleep();
}


//Esto se ejecuta cada vez que se produce un fallo de asignacio de heap
void vApplicationMallocFailedHook (void)
{
    while(1);
}



//**********************************************************************************************************************************************************
//
// _____________________________ TAREAS _________________________________________________________________________________________________________________
//
//**********************************************************************************************************************************************************

//****************************** Tarea canal USB *********************************//

static portTASK_FUNCTION( USBMessageProcessingTask, pvParameters ){

    uint8_t pui8Frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
    int32_t i32Numdatos;
    uint8_t ui8Message;
    void *ptrtoreceivedparam;
    uint32_t ui32Errors=0;

    /* The parameters are not used. */
    ( void ) pvParameters;

    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\n\nBienvenido a la aplicacion control de MiniRobot (curso 2022/23)!\n");
    UARTprintf("\nAutor: Victor Santos");

    for(;;)
    {
        //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
        i32Numdatos=receive_frame(pui8Frame,MAX_FRAME_SIZE); //Esta funcion es bloqueante
        if (i32Numdatos>0)
        {	//Si no hay error, proceso la trama que ha llegado.
            i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobaciï¿½n checksum
            if (i32Numdatos<0)
            {
                //Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
                ui32Errors++;
                // Procesamiento del error 
            }
            else
            {
                //El paquete esta bien, luego procedo a tratarlo.
                //Obtiene el valor del campo mensaje
                ui8Message=decode_message_type(pui8Frame);
                //Obtiene un puntero al campo de parametros y su tamanio.
                i32Numdatos=get_message_param_pointer(pui8Frame,i32Numdatos,&ptrtoreceivedparam);
                switch(ui8Message)
                {
                case MENSAJE_PING :
                    //A un mensaje de ping se responde con el propio mensaje
                    i32Numdatos=create_frame(pui8Frame,ui8Message,0,0,MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        send_frame(pui8Frame,i32Numdatos);
                    }else{
                        //Error de creacion de trama: determinar el error y abortar operacion
                        ui32Errors++;
                        // Procesamiento del error
                        switch(i32Numdatos){
                        case PROT_ERROR_NOMEM:
                            // Procesamiento del error NO MEMORY
                            break;
                        case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
                            //							// Procesamiento del error STUFFED_FRAME_TOO_LONG
                            break;
                        case PROT_ERROR_MESSAGE_TOO_LONG:
                            //							// Procesamiento del error MESSAGE TOO LONG
                            break;
                        }
                        case PROT_ERROR_INCORRECT_PARAM_SIZE:
                        {
                            // Procesamiento del error INCORRECT PARAM SIZE 
                        }
                        break;
                    }
                    break;

                case MENSAJE_SLITHERS:
                {
                    PARAM_MENSAJE_SLITHERS parametro;
                    int8_t vMotor[2];
                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0)
                    {
                       vMotor[0] = parametro.valor_motor_1;
                       vMotor[1] = parametro.valor_motor_2;
                       xQueueSend(cola_freertos_mot,vMotor,500);
                    }else{//Error
                        ui32Errors++; // Tratamiento del error
                    }
                }
                    break;

                default:
                {
                    PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                    parametro.message=ui8Message;
                    //El mensaje esta bien pero no esta implementado
                    i32Numdatos=create_frame(pui8Frame,MENSAJE_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        send_frame(pui8Frame,i32Numdatos);
                    }
                    break;
                }
                }// switch
            }
        }else{ // if (ui32Numdatos >0)
            //Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
            ui32Errors++;
            // Procesamiento del error
        }
    }
}

//*************************** TAREA PWM ************************************//

static portTASK_FUNCTION(PWMTask,pvParameters)
{
    //Initial configuration
    uint32_t ui32DutyCycle1,ui32DutyCycle2;
    uint32_t periodPWM = PERIOD_PWM; // Configuramos la frecuencia de 50Hz
    int8_t vMotor[2];

    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // Configuramos el generador PWM
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, periodPWM); // Establece periodo
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); // Habilita generador
//    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true); // Habilita salidas PWM (Dejemos que empicen a 0)
    //
    // Loop forever.
    //
    while(1)
    {
        if (xQueueReceive(cola_freertos_mot,vMotor,portMAX_DELAY)==pdTRUE){ // Cambiar portMAX_DELAY por 500 para apagado automatico 5 seg despues

            ui32DutyCycle1 = (abs(vMotor[0])/100.0) * PERIOD_PWM;   //Calculamos los ciclos de trabajo
            ui32DutyCycle2 = (abs(vMotor[1])/100.0) * PERIOD_PWM;

            if(ui32DutyCycle1!=0){
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle1 ); // Establece ciclo de trabajo 1
                PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita salidas PWMbit6
            }else{
                PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false); // deshabilita salida PWMbit6
            }

            if(ui32DutyCycle2!=0){
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle2); // Establece ciclo de trabajo 2
                PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita salidas PWMbit7
            }else{
                PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false); // deshabilita salida PWMbit7
            }

        }else{  //Por esta rama nunca llega excepto que la espera no sea portMAX_DELAY
            while(1);
        }

    }
}

//*************************** TAREA VELOCIDAD ************************************//

static portTASK_FUNCTION(VelTask,pvParameters)
{

    while(1)
    {
        xSemaphoreTake(semaforo_freertos2,portMAX_DELAY); //Paramos en el semaforo binario del RTOS
    }
}

void vTimerCallback( TimerHandle_t pxTimer )
{
    xSemaphoreGive(semaforo_freertos2);
}

//**********************************************************************************************************************************************************
//
// --------------------------------------- MAIN ------------------------------------------------------------------------------------------------------------
//
//**********************************************************************************************************************************************************
int main(void)
{

    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);	//Ponemos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)
    g_ui32SystemClock = SysCtlClockGet(); // Get the system clock speed.

    MAP_SysCtlPeripheralClockGating(true);  //Habilita el clock gating de los perifericos durante el bajo consumo

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

    //___________________________CONFIGURACIONES INICIALES PWM___________________________________//

    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);  // NHR: Configure PWM Clock divide system clock by 2

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // NHR: Se usara salidas puerto F para PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // NHR: Se usa el PWM1

    GPIOPinConfigure(GPIO_PF2_M1PWM6); // NHR: Configura PF2 como salida 6 del modulo PWM 1(ver pinmap.h)
    GPIOPinConfigure(GPIO_PF3_M1PWM7); // NHR: Configura PF3 como salida 7 del modulo PWM 1(ver pinmap.h)
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2); // NHR: PF2 y PF3 van a tener funcn PWM

    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);

    /**********************CREACION TAREAS******************************/

    // Inicializa el sistema de depuracion por terminal UART
    if (initCommandLine(256,tskIDLE_PRIORITY + 1) != pdTRUE)
    {
        while(1);
    }

    USBSerialInit(32,32);	//Inicializo el  sistema USB
    //
    // Crea las tareas
    //
    if(xTaskCreate(USBMessageProcessingTask,"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(PWMTask, "PWM", PWMTASKSTACKSIZE, NULL ,tskIDLE_PRIORITY + PWMTASKPRIO, NULL) != pdPASS))
    {
        while(1);
    }

    if((xTaskCreate(VelTask, "VelCalc", VELTASKSTACKSIZE, NULL ,tskIDLE_PRIORITY + VELTASKPRIO, NULL) != pdPASS))
    {
        while(1);
    }

    xTimer = xTimerCreate("TimerSW", 0.2 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback); // Creacion del timerSW cada 200ms
    if( NULL == xTimer )
             {
                while(1);
             }
    else{
        if( xTimerStart( xTimer, 0 ) != pdPASS ) //Inicializacion timerSW
        {
            while(1);
        }
    }

    //Creamos la cola
    cola_freertos_mot=xQueueCreate(16,sizeof(uint32_t));
    if (NULL==cola_freertos_mot)
    while(1); // Si hay problemas para crear la cola, se queda aquí.

    semaforo_freertos2=xSemaphoreCreateBinary();
    if ((semaforo_freertos2==NULL))
    {
        while (1);  //No hay memoria para los semaforo
    }
    //
    // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
    //
    vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

    //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
    while(1);
}
