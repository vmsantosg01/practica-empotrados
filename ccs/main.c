//*****************************************************************************
//
//  CODIGO PRACTICA 1 SISTEMAS EMPOTRADOS
//  AUTOR: VICTOR MANUEL SANTOS GARCIA
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
#include "config.h"
#include <stdbool.h>
#include "event_groups.h"        // FreeRTOS: definiciones relacionadas con grupos de eventos

#define DIV_RELOJ_PWM   2
#define PERIOD_PWM (SysCtlClockGet()*0.0025)/DIV_RELOJ_PWM //Periodo = 0.02 ms
#define POSICIONES_COLA 3
#define TIEMPOT1 0.2
#define BOTON3 GPIO_PIN_1



//Etiquetas de ayuda para que quede ms legible
#define LOWENERGY 0x0001
#define ENERGYOUT 0x0002
#define CAIDA 0x0004
#define BURNING 0x0008
#define MOT_ENCEND 0x0010
#define MOT_APAG 0x0020
#define BURNED 0x0040
#define SOLVED 0x0080

// Definiciones de tareas
#define PWMTASKPRIO 1           // Prioridad para la tarea PWMTASK
#define PWMTASKSTACKSIZE 128    // Tamano de pila para la tarea PWMTASK
#define VELTASKPRIO 1           // Prioridad para la tarea PWMTASK
#define VELTASKSTACKSIZE 128    // Tamano de pila para la tarea PWMTASK

// Variables globales "main"
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
uint32_t debug;


static EventGroupHandle_t FlagsEventosAlarm;
TaskHandle_t pwmtask_handler,vel_handler,alarmtask_handler,energytask_handler,buttontask_handler;
SemaphoreHandle_t semaforo_freertos2,USBSemaphoreMutex,semaforo_energy;
TimerHandle_t xTimer,xTimer2,xTimerMot,xTimerMot2,xTimerMot3;
QueueHandle_t cola_energy,cola_freertos_mot,cola_freertos_mot2,cola_freertos_bot,cola_velocity,cola_freertos_bot2,cola_gled,cola_ADC,cola_alarm_energy, cola_giroscopio;
static QueueSetHandle_t grupo_colas,grupo_colas_energy,grupo_colas_bot,grupo_colas_pwm;

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

    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
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
        {   //Si no hay error, proceso la trama que ha llegado.
            i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobacinn checksum
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
                        xSemaphoreTake(USBSemaphoreMutex,portMAX_DELAY);
                        send_frame(pui8Frame,i32Numdatos);
                        xSemaphoreGive(USBSemaphoreMutex);
                    }else{
                        //Error de creacion de trama: determinar el error y abortar operacion
                        ui32Errors++;
                        // Procesamiento del error
                        switch(i32Numdatos){
                        case PROT_ERROR_NOMEM:
                            // Procesamiento del error NO MEMORY
                            break;
                        case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
                            //                          // Procesamiento del error STUFFED_FRAME_TOO_LONG
                            break;
                        case PROT_ERROR_MESSAGE_TOO_LONG:
                            //                          // Procesamiento del error MESSAGE TOO LONG
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
                       xQueueSend(cola_freertos_mot2,vMotor,500);
                       xQueueSend(cola_energy,vMotor,500);
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
                        xSemaphoreTake(USBSemaphoreMutex,portMAX_DELAY);
                        send_frame(pui8Frame,i32Numdatos);
                        xSemaphoreGive(USBSemaphoreMutex);
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
    QueueSetMemberHandle_t  Activado;
    uint32_t ui32DutyCycle1,ui32DutyCycle2,ui32DutyCycle3;
    uint32_t periodPWM = PERIOD_PWM; // Configuramos la frecuencia de 50Hz
    int8_t vMotor[2];
    int32_t systemEnergy;

    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // Configuramos el generador PWM
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, periodPWM); // Establece periodo
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); // Habilita generador

    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // Configuramos el generador PWM
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, periodPWM); // Establece periodo
    PWMGenEnable(PWM1_BASE, PWM_GEN_1); // Habilita generador

    while(1)
    {

        Activado = xQueueSelectFromSet( grupo_colas_pwm, portMAX_DELAY);
        if (Activado==cola_freertos_mot)
        {
            xQueueReceive(cola_freertos_mot,vMotor,0);
            ui32DutyCycle1 = (abs(vMotor[0])/100.0) * PERIOD_PWM;   //Calculamos los ciclos de trabajo
            ui32DutyCycle2 = (abs(vMotor[1])/100.0) * PERIOD_PWM;

            if(ui32DutyCycle1!=0){
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2,ui32DutyCycle1 ); // Establece ciclo de trabajo 1
                PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true); // Habilita salidas PWMbit2
            }else{
                PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false); // deshabilita salida PWMbit2
            }

            if(ui32DutyCycle2!=0){
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,ui32DutyCycle2); // Establece ciclo de trabajo 2
                PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true); // Habilita salidas PWMbit3
            }else{
                PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false); // deshabilita salida PWMbit3
            }

        }else if(Activado==cola_gled){          //En principio esto enciende el led verde con la intensidad de systemEnergy/MAX_ENERGY en tanto por uno
            xQueueReceive(cola_gled,&systemEnergy,0);
            ui32DutyCycle3 = ((float)systemEnergy/MAX_ENERGY) * PERIOD_PWM;   //Calculamos los ciclos de trabajo

            if(ui32DutyCycle3!=0){
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle3 ); // Establece ciclo de trabajo 1
                PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true); // Habilita salidas PWMbit2
            }else{
                PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false); // deshabilita salida PWMbit2
            }

        }
    }
}

//*************************** TAREA VELOCIDAD ************************************//

static portTASK_FUNCTION(VelTask,pvParameters)
{
    QueueSetMemberHandle_t  Activado;
    PARAM_MENSAJE_DATOS_VELOCIDAD parametro;
    parametro.rVel = 0;
    parametro.rAngle = 0;
    parametro.travelDistance = 0;

    uint32_t total_distance = 0;

    int32_t i32Numdatos;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int8_t vMotor[2];

    while(1)
    {
        Activado = xQueueSelectFromSet( grupo_colas, portMAX_DELAY);
        if (Activado==cola_freertos_mot2)
        {
            xQueueReceive(cola_freertos_mot2,vMotor,0);
            xSemaphoreTake(semaforo_freertos2,0);
            parametro.rVel = (vMotor[0] + vMotor[1])/20.0;

            if(parametro.rVel != 0){
                xEventGroupSetBits(FlagsEventosAlarm, MOT_ENCEND);
            }else{
                xEventGroupSetBits(FlagsEventosAlarm, MOT_APAG);
            }
        }else if(Activado==semaforo_freertos2){
            xSemaphoreTake(semaforo_freertos2,0); //cierra el semaforo para que pueda volver a darse
        }

        parametro.rAngle += (int32_t)(TIEMPOT1 * (vMotor[0] - vMotor[1])/10.0) * (180.0/3.14); //Formula angulo (vizq - Vder/ dist)*tiempo*180/pi
        if(parametro.rAngle < 0){
            parametro.rAngle += 360.0;
        }else if(parametro.rAngle > 360){
            parametro.rAngle -= 360.0;
        }
        total_distance += (TIEMPOT1*abs(parametro.rVel));
        parametro.travelDistance = total_distance;

        i32Numdatos=create_frame(pui8Frame,MENSAJE_DATOS_VELOCIDAD,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
        if (i32Numdatos>=0)
        {
            xSemaphoreTake(USBSemaphoreMutex,portMAX_DELAY);
            send_frame(pui8Frame,i32Numdatos);
            xSemaphoreGive(USBSemaphoreMutex);
        }else{
            while(1);
        }
    }
}

//*************************** TAREA SENSORES ************************************//

static portTASK_FUNCTION( ButtonsTask, pvParameters )
{
    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
    PARAM_MENSAJE_BUTTONS parametro;
    int32_t i32Numdatos;
    int32_t i32Status;
    QueueSetMemberHandle_t  Activado;
    uint32_t valor_potenciometro,valor_giroscopioant,valor_giroscopio;
    bool first = true;
    bool pressed_left = false;
    bool pressed_right = false;
    bool pressed_mid = false;
    //
    // Loop forever.
    //
    while(1)
    {

        Activado = xQueueSelectFromSet( grupo_colas_bot, portMAX_DELAY);

        if (Activado==cola_freertos_bot)
        {
            parametro.fLeft = 0;
            parametro.fRight = 0;
            parametro.fMid = 0;
            xQueueReceive(cola_freertos_bot,&i32Status,0);

            if((i32Status & LEFT_BUTTON) == 0){ //Left button pressed
                parametro.fLeft = 1;
                xTimerReset( xTimerMot, 0 );
                xTimerStart( xTimerMot, 0 );
                pressed_left = true;

            }else if(pressed_left){
                xTimerReset( xTimerMot, 0 );
                xTimerStop( xTimerMot, 0 );
                xTimerChangePeriod( xTimerMot, 4 * configTICK_RATE_HZ, 0 );
                xEventGroupSetBits(FlagsEventosAlarm, SOLVED);
                pressed_left = false;
            }



            if((i32Status & RIGHT_BUTTON) == 0){    //Right button pressed
                parametro.fRight = 1;
                xTimerReset( xTimerMot2, 0 );
                xTimerStart( xTimerMot2, 0 );
                pressed_right = true;

            }else if(pressed_right){
                xTimerReset( xTimerMot2, 0 );
                xTimerStop( xTimerMot2, 0 );
                xTimerChangePeriod( xTimerMot2, 4 * configTICK_RATE_HZ, 0 );
                xEventGroupSetBits(FlagsEventosAlarm, SOLVED);
                pressed_right = false;
            }


            if((i32Status & MID_BUTTON) == 0){  //Mid button pressed
                parametro.fMid = 1;
                xTimerReset( xTimerMot3, 0 );
                xTimerStart( xTimerMot3, 0 );
                pressed_mid = true;

            }else if(pressed_mid){
                xTimerReset( xTimerMot3, 0 );
                xTimerStop( xTimerMot3, 0 );
                xTimerChangePeriod( xTimerMot3, 4 * configTICK_RATE_HZ, 0 );
                xEventGroupSetBits(FlagsEventosAlarm, SOLVED);
                pressed_mid = false;
            }
        }

        if(Activado==cola_ADC){
            xQueueReceive(cola_ADC,&valor_potenciometro,0);
            parametro.distADC = (valor_potenciometro * 70)/4095;
        }

        if(Activado==cola_giroscopio){
            xQueueReceive(cola_giroscopio,&valor_giroscopio,0);

            if(first){
                valor_giroscopioant = valor_giroscopio;
                first = false;
            }

            if(abs(valor_giroscopioant - valor_giroscopio) > 2048){
                xEventGroupSetBits(FlagsEventosAlarm, CAIDA);
            }
            valor_giroscopioant = valor_giroscopio;
        }

        i32Numdatos=create_frame(pui8Frame,MENSAJE_BUTTONS,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
        if (i32Numdatos>=0)
        {
            xSemaphoreTake(USBSemaphoreMutex,portMAX_DELAY);
            send_frame(pui8Frame,i32Numdatos);
            xSemaphoreGive(USBSemaphoreMutex);
        }


 //       parametro.button.fMid = 0;
    }
}

//*************************** TAREA ENERGIA ************************************//

static portTASK_FUNCTION( EnergyTask, pvParameters )
{
    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
    QueueSetMemberHandle_t  Activado;
    PARAM_MENSAJE_ENERGY parametro;
    int32_t i32Numdatos;
    int8_t vMotor[2];
    int32_t systemEnergy = 8000; //MAX_ENERGY;
    uint8_t mot1consumo = 0;
    uint8_t mot2consumo = 0;
    bool first_time = true;
    //
    // Loop forever.
    //
    while(1)
    {
        Activado = xQueueSelectFromSet( grupo_colas_energy, portMAX_DELAY);
        if (Activado==cola_energy)
        {
            xQueueReceive(cola_energy,vMotor,0);
            xSemaphoreTake(semaforo_energy,0);

            mot1consumo = abs(vMotor[0]);   //Calculamos los consumos
            mot2consumo = abs(vMotor[1]);

        }else if(Activado==semaforo_energy){
            xSemaphoreTake(semaforo_energy,0); //cierra el semaforo para que pueda volver a darse
        }

        systemEnergy -= (mot1consumo + mot2consumo);
        if(systemEnergy < 0){
            systemEnergy = 0;
            xEventGroupSetBits(FlagsEventosAlarm, ENERGYOUT);
        }else if(systemEnergy < 4000 && first_time){
            first_time = false;
            xEventGroupSetBits(FlagsEventosAlarm, LOWENERGY);
        }

        parametro.energy = systemEnergy;
        xQueueSend(cola_gled,&systemEnergy,500);

        i32Numdatos=create_frame(pui8Frame,MENSAJE_ENERGY,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
        if (i32Numdatos>=0)
        {
            xSemaphoreTake(USBSemaphoreMutex,portMAX_DELAY);
            send_frame(pui8Frame,i32Numdatos);
            xSemaphoreGive(USBSemaphoreMutex);
        }
    }
}

static portTASK_FUNCTION( AlarmTask, pvParameters )
{
    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
    PARAM_MENSAJE_ALARM parametro;
    int32_t i32Numdatos;
    EventBits_t uxBits;
    bool burning = false;

    bool motores_on = false;

    uint32_t ui32Color[3];

    ui32Color[RED] = 0xFFF0;   // Minimo: 0; Maxima 0xFFFF
    ui32Color[BLUE] = 0x0; // Minimo: 0; Maxima 0xFFFF
    ui32Color[GREEN] = 0x0;    // Minimo: 0; Maxima 0xFFFF

    parametro.lowEnergy = false;
    parametro.motorHot = false;
    parametro.robotFall = false;
    parametro.energybelow4k = false;

    //
    // Loop forever.
    //
    while(1)
    {
        uxBits = xEventGroupWaitBits(FlagsEventosAlarm,LOWENERGY|ENERGYOUT|CAIDA|BURNING|MOT_ENCEND|MOT_APAG|BURNED|SOLVED ,pdTRUE,pdFALSE,portMAX_DELAY);

        if ((uxBits & LOWENERGY) == LOWENERGY){
            RGBInit(0);
            RGBEnable();              // Habilita la generacion PWM para el encendido de los LEDs
            RGBSet(ui32Color,0.99);

            RGBBlinkRateSet(1.0f);
            parametro.energybelow4k = true;
        }

        else if ((uxBits & ENERGYOUT) == ENERGYOUT){
            RGBSet(ui32Color,0.99);
            RGBBlinkRateSet(2.0f);  //parpadeo 2 segundos
            parametro.lowEnergy = true;

            vTaskSuspend(pwmtask_handler);
            vTaskSuspend(energytask_handler);
            vTaskSuspend(vel_handler);
        }

        else if ((uxBits & CAIDA) == CAIDA){
            RGBInit(0);
            RGBEnable();              // Habilita la generacion PWM para el encendido de los LEDs
            parametro.robotFall = true;
            RGBSet(ui32Color,0.99);
            RGBBlinkRateSet(50.0f);  //parpadeo cte

            //FALTA DESACTIVAR INTERRUPCIONES
            vTaskSuspend(pwmtask_handler);
            vTaskSuspend(energytask_handler);
            vTaskSuspend(vel_handler);
        }

        else if ((uxBits & BURNING) == BURNING){
            if(motores_on){
                parametro.motorHot = true;
                if(!burning){
                    burning = true;
                    xTimerChangePeriod( xTimerMot, 6 * configTICK_RATE_HZ, 0 );
                    xTimerReset( xTimerMot, 0 );
                    xTimerStart( xTimerMot, 0 );
                }else{
                    RGBInit(0);
                    RGBEnable();              // Habilita la generacion PWM para el encendido de los LEDs
                    ui32Color[RED] = 0x0;   // Minimo: 0; Maxima 0xFFFF
                    ui32Color[BLUE] = 0xFFF0; // Minimo: 0; Maxima 0xFFFF
                    ui32Color[GREEN] = 0x0;    // Minimo: 0; Maxima 0xFFFF

                    RGBSet(ui32Color,0.99);
                    RGBBlinkRateSet(50.0f);

                    vTaskSuspend(pwmtask_handler);
                    vTaskSuspend(energytask_handler);
                    vTaskSuspend(vel_handler);
                }

            }
        }

        else if ((uxBits & MOT_ENCEND) == MOT_ENCEND){
            motores_on = true;
        }

        else if ((uxBits & MOT_APAG) == MOT_APAG){
            motores_on = false;
        }

//        else if ((uxBits & BURNED) == BURNED){
//
//
//        }

        else if ((uxBits & SOLVED) == SOLVED){
            parametro.motorHot = false;
            burning = false;
        }

        i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARM,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
        if (i32Numdatos>=0)
        {
            xSemaphoreTake(USBSemaphoreMutex,portMAX_DELAY);
            send_frame(pui8Frame,i32Numdatos);
            xSemaphoreGive(USBSemaphoreMutex);
        }
    }
}


//*************************** TAREA TIMER DATOS ************************************//

void vTimerCallback( TimerHandle_t pxTimer )
{
    if(pxTimer == xTimer)
    {
        xSemaphoreGive(semaforo_freertos2);
    }

    if(pxTimer == xTimer2)
    {
        xSemaphoreGive(semaforo_energy);
    }

    if(pxTimer == xTimerMot)
    {
        xEventGroupSetBits(FlagsEventosAlarm, BURNING);
    }

//    if(pxTimer == xTimerMot2)
//    {
//        xEventGroupSetBits(FlagsEventosAlarm, BURNING);
//    }
//    if(pxTimer == xTimerMot3)
//    {
//        xEventGroupSetBits(FlagsEventosAlarm, BURNING);
//    }
    //TE HAS QUEDADO HACIENDO LO DE LOS TIMERS
}

//**********************************************************************************************************************************************************
//
// -------------------------------------------------------------- MAIN -------------------------------------------------------------------------------------
//
//**********************************************************************************************************************************************************
int main(void)
{

    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); //Ponemos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)
    g_ui32SystemClock = SysCtlClockGet(); // Get the system clock speed.

    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5); // Para que funbcione el modulo rgb

    MAP_SysCtlPeripheralClockGating(true);  //Habilita el clock gating de los perifericos durante el bajo consumo

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

    //___________________________CONFIGURACIONES INICIALES PWM___________________________________//

    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);  // NHR: Configure PWM Clock divide system clock by 2

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // NHR: Se usara salidas puerto E para PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // NHR: Se usara salidas puerto F para PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // NHR: Se usa el PWM1

    GPIOPinConfigure(GPIO_PE5_M1PWM3); // NHR: Configura PE5 como salida 3 del modulo PWM 1(ver pinmap.h)
    GPIOPinConfigure(GPIO_PE4_M1PWM2); // NHR: Configura PE4 como salida 2 del modulo PWM 1(ver pinmap.h)
    GPIOPinConfigure(GPIO_PF3_M1PWM7); // NHR: Configura PF3omo salida 7 del modulo PWM 1(ver pinmap.h)
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4); // NHR: PE5 y PE4 van a tener funcion PWM
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3); // NHR: PF3 va a tener funcion PWM

    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);

    //___________________________CONFIGURACIONES INICIALES BOTONES___________________________________//

    ButtonsInit();
    //ESTAS SON PARA EL BOTON EXTRA (PinD1)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIODirModeSet(GPIO_PORTD_BASE, BOTON3, GPIO_DIR_MODE_IN); //
    MAP_GPIOPadConfigSet(GPIO_PORTD_BASE, BOTON3,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
    MAP_GPIOIntTypeSet(GPIO_PORTD_BASE, BOTON3,GPIO_BOTH_EDGES);

    MAP_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);// Misma prioridad que configMAX_SYSCALL_INTERRUPT_PRIORITY
    MAP_IntPrioritySet(INT_GPIOD,configMAX_SYSCALL_INTERRUPT_PRIORITY);// Misma prioridad que configMAX_SYSCALL_INTERRUPT_PRIORITY
    MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
    MAP_IntEnable(INT_GPIOF);
    MAP_GPIOIntEnable(GPIO_PORTD_BASE,BOTON3);
    MAP_IntEnable(INT_GPIOD);

    //___________________________CONFIGURACIONES INICIALES ADC___________________________________//

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_ANALOG);


    TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SystemClock * 0.25 -1);
    TimerEnable(TIMER2_BASE,TIMER_A);
    //timer configurado

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   // Habilita ADC0
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);   // Habilita ADC0
    ADCSequenceDisable(ADC0_BASE, 1); // Deshabilita el secuenciador 1 del ADC0 para su configuracion
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);
    TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
    ADCHardwareOversampleConfigure(ADC0_BASE,64);
    // Configuramos los 4 conversores del secuenciador 1 para muestreo del sensor de temperatura
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END); // El conversor 4 es el ultimo, y  se genera un aviso de interrupcion // este canal para leer y detectar caida
    // Tras configurar el secuenciador, se vuelve a habilitar

    // Instrucciones relacionadas con la habilitacinn de interrupciones
    IntEnable(INT_ADC0SS1); // Habilitacinn a nivel global del sistema
    ADCIntEnable(ADC0_BASE,1); // Habilitacinn del secuenciador dentro del perifnrico
    ADCIntClear(ADC0_BASE,1); // Borramos posibles interrupciones pendientes
    // FALTA ALGO PARA HABILITAR LA INTERRUPCION

    ADCSequenceEnable(ADC0_BASE, 1);

    //___________________________CONFIGURACIONES INICIALES RGB___________________________________//

//    RGBInit(0);
//    RGBDisable();

    //-------------------------------------------------------------------------------------------------------------------------------------------------


    // Inicializa el sistema de depuracion por terminal UART
    if (initCommandLine(256,tskIDLE_PRIORITY + 1) != pdTRUE)
    {
        while(1);
    }

    USBSerialInit(32,32);   //Inicializo el  sistema USB

    //*****************************************************************************************
    //---------------------------------- CREACION TAREAS --------------------------------------
    //*****************************************************************************************


    if(xTaskCreate(USBMessageProcessingTask,"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(PWMTask, "PWM", PWMTASKSTACKSIZE, NULL ,tskIDLE_PRIORITY + PWMTASKPRIO, &pwmtask_handler) != pdPASS))
    {
        while(1);
    }

    if((xTaskCreate(VelTask, "VelCalc", VELTASKSTACKSIZE, NULL ,tskIDLE_PRIORITY + VELTASKPRIO, &vel_handler) != pdPASS))
    {
        while(1);
    }

    if(xTaskCreate(ButtonsTask, "Botones",512, NULL, tskIDLE_PRIORITY + 2, &buttontask_handler) != pdTRUE)
    {
        while(1);
    }

    if(xTaskCreate(EnergyTask, "Energia",512, NULL, tskIDLE_PRIORITY + 2, &energytask_handler) != pdTRUE)
    {
        while(1);
    }

    if(xTaskCreate(AlarmTask, "Alarmas",512, NULL, tskIDLE_PRIORITY + 2, &alarmtask_handler) != pdTRUE)
    {
        while(1);
    }

    // ____________________ CREACION DE COLAS _________________________ //

    cola_freertos_bot=xQueueCreate(3,sizeof(int32_t));
    if (NULL==cola_freertos_bot)
        while(1);

    cola_freertos_bot2=xQueueCreate(3,sizeof(int32_t));
    if (NULL==cola_freertos_bot2)
        while(1);

    cola_gled=xQueueCreate(3,sizeof(int32_t));
    if (NULL==cola_freertos_bot2)
        while(1);

    cola_freertos_mot=xQueueCreate(POSICIONES_COLA,sizeof(uint32_t));
    if (NULL==cola_freertos_mot)
        while(1);

    cola_freertos_mot2=xQueueCreate(POSICIONES_COLA,sizeof(uint32_t));
    if (NULL==cola_freertos_mot2)
        while(1);

    cola_energy=xQueueCreate(POSICIONES_COLA,sizeof(uint32_t));
    if (NULL==cola_energy)
        while(1);

    cola_ADC=xQueueCreate(3,sizeof(uint32_t));
    if (NULL==cola_ADC)
        while(1);
    cola_giroscopio=xQueueCreate(1,sizeof(uint32_t));
    if (NULL==cola_giroscopio)
        while(1);

//    cola_alarm_energy=xQueueCreate(POSICIONES_COLA,sizeof(uint32_t));
//    if (NULL==cola_alarm_energy)
//        while(1);

    // ______________________________ CREACION TIMERS _______________________________ //

    // "Creacion timer 200 ms"
    xTimer = xTimerCreate("TimerSW", TIEMPOT1 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback); // Creacion del timerSW cada 200ms
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
    // "Creacion timer 1 s"
    xTimer2 = xTimerCreate("TimerSW2", configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback); // Creacion del timerSW cada 1s
    if( NULL == xTimer2 )
         {
            while(1);
         }
    else{
        if( xTimerStart( xTimer2, 0 ) != pdPASS ) //Inicializacion timerSW
        {
            while(1);
        }
    }

    xTimerMot = xTimerCreate("TimerMot", 4 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback); // Creacion del timerSW cada 1s
        if( NULL == xTimerMot )
         {
            while(1);
         }
    xTimerMot2 = xTimerCreate("TimerMot2", 4 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback); // Creacion del timerSW cada 1s
            if( NULL == xTimerMot )
             {
                while(1);
             }
    xTimerMot3 = xTimerCreate("TimerMot3", 4 * configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallback); // Creacion del timerSW cada 1s
            if( NULL == xTimerMot )
             {
                while(1);
             }
    // ___________________ CREACION SEMAFOROS ____________________ //

    semaforo_freertos2=xSemaphoreCreateBinary();
    if ((semaforo_freertos2==NULL))
    {
        while (1);
    }

    semaforo_energy=xSemaphoreCreateBinary();
    if ((semaforo_energy==NULL))
    {
        while (1);
    }

    // ___________________________ DEFINICION GRUPOS DE COLAS ___________________________ //

    grupo_colas = xQueueCreateSet( POSICIONES_COLA + 1);
    if (NULL == grupo_colas)
        while(1);

    if (xQueueAddToSet(cola_freertos_mot2, grupo_colas)!=pdPASS)
    {
        while(1);
    }
    if (xQueueAddToSet(semaforo_freertos2, grupo_colas)!=pdPASS)
    {
        while(1);
    }

    grupo_colas_energy = xQueueCreateSet( POSICIONES_COLA + 1);
    if (NULL == grupo_colas_energy)
        while(1);

    if (xQueueAddToSet(cola_energy, grupo_colas_energy)!=pdPASS)
    {
        while(1);
    }
    if (xQueueAddToSet(semaforo_energy, grupo_colas_energy)!=pdPASS)
    {
        while(1);
    }

    grupo_colas_bot = xQueueCreateSet(10);
    if (NULL == grupo_colas_bot)
        while(1);

    if (xQueueAddToSet(cola_freertos_bot, grupo_colas_bot)!=pdPASS)
    {
        while(1);
    }
    if (xQueueAddToSet(cola_freertos_bot2, grupo_colas_bot)!=pdPASS)
    {
        while(1);
    }
    if (xQueueAddToSet(cola_ADC, grupo_colas_bot)!=pdPASS)
    {
        while(1);
    }
    if (xQueueAddToSet(cola_giroscopio, grupo_colas_bot)!=pdPASS)
    {
        while(1);
    }

    grupo_colas_pwm = xQueueCreateSet( POSICIONES_COLA + 6);
        if (NULL == grupo_colas_pwm)
            while(1);

        if (xQueueAddToSet(cola_freertos_mot, grupo_colas_pwm)!=pdPASS)
        {
            while(1);
        }
        if (xQueueAddToSet(cola_gled, grupo_colas_pwm)!=pdPASS)
        {
            while(1);
        }

     // GRUPO EVENTOS

    //Crea el grupo de eventos
    FlagsEventosAlarm = xEventGroupCreate();
    if( NULL ==  FlagsEventosAlarm)
    {
        while(1);
    }

    USBSemaphoreMutex = xSemaphoreCreateMutex(); // Creacion del mutex
    if (NULL == USBSemaphoreMutex)
    {
        while (1);
    }

    vTaskStartScheduler(); // Arrancamos el organizador de tareas
    while(1);
}

// ***********************************************************************************************
// ------------------------------------ INTERRUPCIONES -------------------------------------------
// ***********************************************************************************************

void GPIOFIntHandler(void)
{
    signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;
    int32_t i32Status = MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);
    i32Status |= MAP_GPIOPinRead(GPIO_PORTD_BASE,BOTON3);
    xQueueSendFromISR (cola_freertos_bot,&i32Status,&higherPriorityTaskWoken);
    MAP_GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);
    MAP_GPIOIntClear(GPIO_PORTD_BASE,BOTON3);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void ADCIntHandler(void)
{
    uint32_t valor_potenciometro,valor_giroscopio;
    uint32_t ui32ADC0Value[4];
    signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;   //Hay que inicializarlo a False!!
    ADCIntClear(ADC0_BASE, 1);
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    valor_potenciometro = ((ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2])/3)+0.5;
    valor_giroscopio = ui32ADC0Value[3];
    xQueueSendFromISR (cola_ADC,&valor_potenciometro,&higherPriorityTaskWoken);
    xQueueSendFromISR (cola_giroscopio,&valor_giroscopio,&higherPriorityTaskWoken);
    //Cesion de control de CPU si se ha despertado una tarea de mayor prioridad
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

