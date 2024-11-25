#ifndef PRO_H
#define PRO_H

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bus communication
#define PRO_DefaultBaud 921600
#define PRO_MaxTotalCommandLength (30 + 1)	// ex: //999XXXX-2147483648\r Adding 1 for end string char (\0)
//										// ex: //999XX000000000000000000\r
#define PRO_Timeout 100						// in ms
#define PRO_CommandStart "#"
#define PRO_CommandReplyStart "*"
#define PRO_CommandEnd "\r"
// PRO constants
#define PRO_ID_Default 0
#define PRO_ID_Min 0
#define PRO_ID_Max 250
#define PRO_Mode255ID 255
#define PRO_BroadcastID 254

#define PRO_CommStatus_Idle 0
#define PRO_CommStatus_ReadSuccess 1
#define PRO_CommStatus_ReadTimeout 2
#define PRO_CommStatus_ReadWrongID 3
#define PRO_CommStatus_ReadWrongIdentifier 4
#define PRO_CommStatus_ReadWrongFormat 5
#define PRO_CommStatus_ReadNoBus 6
#define PRO_CommStatus_ReadUnknown 7
#define PRO_CommStatus_WriteSuccess 8
#define PRO_CommStatus_WriteNoBus 9
#define PRO_CommStatus_WriteUnknown 10

// Parameter for setter
#define PRO_SetSession 0
#define PRO_SetConfig 1

// Parameter for gyre direction
#define PRO_GyreClockwise 1
#define PRO_GyreCounterClockwise -1

// LED colors
#define PRO_LED_Black 0
#define PRO_LED_Red 1
#define PRO_LED_Green 2
#define PRO_LED_Blue 3
#define PRO_LED_Yellow 4
#define PRO_LED_Cyan 5
#define PRO_LED_Magenta 6
#define PRO_LED_White 7

// Commands - actions
#define PRO_ActionReset "RESET"
#define PRO_ActionLimp "L"
#define PRO_ActionHold "H"
#define PRO_ActionParameterTime "T"
#define PRO_ActionParameterSpeed "S"
#define PRO_ActionMove "D"

// Commands - actions settings
#define PRO_ActionColorLED "LED"
#define PRO_ActionGyreDirection "G"

// Commands - queries
#define PRO_QueryPosition "QD"
#define PRO_QuerySpeed "QWD"
#define PRO_QueryStatus "Q"

// PRO status
#define PRO_StatusUnknown 0
#define PRO_StatusLimp 1
#define PRO_StatusAccelerating 2
#define PRO_StatusTravelling 3
#define PRO_StatusDecelerating 4
#define PRO_StatusHolding 5
#define PRO_StatusError 6

// Commands - Configurations
#define PRO_ConfigColorLED "CLED"
#define PRO_ConfigGyreDirection "CG"

// PRO structure definition
typedef struct {
    int servo_id;
    int serial_fd;
    bool is_initialized;
} PRO;

// Function declarations
bool pro_init_bus(const char* port_name, int baud_rate);
void pro_close_bus(void);
PRO* pro_create(int id);
void pro_destroy(PRO* pro);
bool pro_reset(PRO* pro);
bool pro_limp(PRO* pro);
bool pro_hold(PRO* pro);
bool pro_move(PRO* pro, int position);
int pro_get_position(PRO* pro);
int pro_get_speed(PRO* pro);
int pro_get_status(PRO* pro);
bool pro_set_color_led(PRO* pro, int color);
bool pro_set_gyre(PRO* pro, int gyre);

#ifdef __cplusplus
}
#endif

#endif  // PRO_H