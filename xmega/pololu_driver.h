/*
 * pololu_driver.h
 *
 * Created: 10/15/2014 5:45:04 PM
 *  Author: Austin
 */

#ifndef POLOLU_DRIVER_H_INCLUDED
#define POLOLU_DRIVER_H_INCLUDED

#include <asf.h>

/*! This macro return the value of the capture register.
 *
 * \param  _tc   The Timer/Counter to get the capture value from.
 */
#define GetCaptureValue(_tc)  ( _tc->CCA )



typedef struct {
	PORT_t *PORT;
	TC0_t * TC0;
	TC1_t* TC1;
	} pololu_t;

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
                      TC0_t * qTimer,
                      TC_EVSEL_t qEventChannel,
                      uint16_t lineCount);

bool QDEC_Port_Setup(PORT_t * qPort, uint8_t qPin, bool useIndex, bool invIO);

bool QDEC_EVSYS_Setup(uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      bool useIndex,
                      EVSYS_QDIRM_t qIndexState );

void QDEC_TC_Dec_Setup(TC0_t * qTimer,
                       TC_EVSEL_t qEventChannel,
                       uint16_t lineCount);

void QDEC_TC_Freq_Setup(TC1_t * qTimer,
                        TC_EVSEL_t qEventChannel,
                        EVSYS_CHMUX_t qPinInput,
                        TC_CLKSEL_t clksel);

uint8_t QDEC_Get_Direction(TC0_t * qTimer);

uint16_t QDEC_Get_Rpm(TC1_t *TC1);

#endif // POLOLU_DRIVER_H_