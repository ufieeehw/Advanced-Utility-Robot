/*
 * pololu_driver.h
 *
 * Created: 10/15/2014 5:45:04 PM
 *  Author: Austin
 */

#ifndef POLOLU_DRIVER_H_INCLUDED
#define POLOLU_DRIVER_H_INCLUDED

#include <asf.h>

/*
 * This macro return the value of the capture register.
 */
#define GetCaptureValue(_tc)  ( _tc->CCA )

/*
 * Struct to hold the settings for each Pololu
 */
typedef struct {
	PORT_t *PORT;
	TC0_t * TC0;
	TC1_t* TC1;
	} pololu_t;

// Static pololu definitions
extern pololu_t pololu_left;
extern pololu_t pololu_right;

void pololuInit(void);
void pololuDrive(pololu_t *pololu, int8_t percent);

bool QDEC_Total_Setup(PORT_t * qPort,
                      uint8_t qPin,
                      bool invIO,
                      uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      bool useIndex,
                      EVSYS_QDIRM_t qIndexState,
                      TC1_t * qTimer,
                      TC_EVSEL_t qEventChannel,
                      uint16_t lineCount);

bool QDEC_Port_Setup(PORT_t * qPort, uint8_t qPin, bool useIndex, bool invIO);

bool QDEC_EVSYS_Setup(uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      bool useIndex,
                      EVSYS_QDIRM_t qIndexState );

void QDEC_TC_Dec_Setup(TC1_t * qTimer,
                       TC_EVSEL_t qEventChannel,
                       uint16_t lineCount);
					   
uint8_t QDEC_Get_Direction(TC1_t * qTimer);

#endif // POLOLU_DRIVER_H_