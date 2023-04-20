/*
 * Listado de los tipos de mensajes empleados en la aplicación, así como definiciones de sus parámetros.
*/
#ifndef __USB_MESSAGES_TABLE_H
#define __USB_MESSAGES_TABLE_H

#include<stdint.h>

//Codigos de los mensajes. El estudiante deberá definir los códigos para los mensajes que vaya
// a crear y usar. Estos deberan ser compatibles con los usados en la parte Qt

typedef enum {
    MENSAJE_NO_IMPLEMENTADO,
    MENSAJE_PING,
    MENSAJE_SLITHERS,
    MENSAJE_DATOS_VELOCIDAD,
    MENSAJE_BUTTONS,
    MENSAJE_ENERGY,
    MENSAJE_ALARM,
    //etc, etc...
} messageTypes;

//Estructuras relacionadas con los parametros de los mensajes. El estuadiante debera crear las
// estructuras adecuadas a los mensajes usados, y asegurarse de su compatibilidad con el extremo Qt

#pragma pack(1)   //Con esto consigo que el alineamiento de las estructuras en memoria del PC (32 bits) no tenga relleno.
//Con lo de abajo consigo que el alineamiento de las estructuras en memoria del microcontrolador no tenga relleno
#define PACKED //__attribute__ ((packed))

typedef struct {
    uint8_t message;
}PACKED PARAM_MENSAJE_NO_IMPLEMENTADO;

typedef struct {
    int8_t valor_motor_1;
    int8_t valor_motor_2;
}PACKED PARAM_MENSAJE_SLITHERS;

typedef struct {
    uint32_t travelDistance;
    float rVel;
    int32_t rAngle;
}PACKED PARAM_MENSAJE_DATOS_VELOCIDAD;

typedef struct {
    uint8_t fLeft:1;
    uint8_t fRight:1;
    uint8_t fMid:1;
    uint8_t distADC;
} PACKED PARAM_MENSAJE_BUTTONS;

typedef struct {
    int32_t energy;
}PACKED PARAM_MENSAJE_ENERGY;

typedef struct {
    bool lowEnergy;
    bool robotFall;
    bool motorHot;
    bool motorBurnt;
    bool energybelow4k;

} PACKED PARAM_MENSAJE_ALARM;

#pragma pack()    //...Pero solo para los mensajes que voy a intercambiar, no para el resto





#endif
