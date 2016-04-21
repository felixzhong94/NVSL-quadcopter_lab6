#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "Alarms.h"

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/


//RAW RC values will be store here
volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

// A buffer to maintain data being received by radio.
const int RF_BUFFER_SIZE = 128;
char radioRXBuffer[RF_BUFFER_SIZE];
volatile uint8_t rxDataLength = 0;
// Global variable shared between RX ISRs
uint8_t rssiRaw;

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver() {
  /*************************   ATMEGA128RF RX Setup   ********************************/
  #if defined(ATMEGA128RF)

    //For debug only
    //SerialOpen(1,9600);

    memset(radioRXBuffer,0, RF_BUFFER_SIZE);

	// Transceiver Pin Register -- TRXPR.
	// This register can be used to reset the transceiver, without
	// resetting the MCU.
	TRXPR |= (1<<TRXRST);   // TRXRST = 1 (Reset state, resets all registers)

	// Transceiver Interrupt Enable Mask - IRQ_MASK
	// This register disables/enables individual radio interrupts.
	// First, we'll disable IRQ and clear any pending IRQ's
	IRQ_MASK = 0;  // Disable all IRQs

	// Transceiver State Control Register -- TRX_STATE
	// This regiseter controls the states of the radio.
	// First, we'll set it to the TRX_OFF state.
	TRX_STATE = (TRX_STATE & 0xE0) | TRX_OFF;  // Set to TRX_OFF state
	delay(1);

	//TODO
	// Transceiver Status Register -- TRX_STATUS
	// This read-only register contains the present state of the radio transceiver.
	// After telling it to go to the TRX_OFF state, we'll make sure it's actually
	// there.
	// Pray here!
	//if ((TRX_STATUS & 0x1F) != TRX_OFF) // Check to make sure state is correct
	//  return 0;	// Error, TRX isn't off

	// Transceiver Control Register 1 - TRX_CTRL_1
	// We'll use this register to turn on automatic CRC calculations.
	TRX_CTRL_1 |= (1<<TX_AUTO_CRC_ON);  // Enable automatic CRC calc.

	// Enable RX start/end and TX end interrupts
	IRQ_MASK = (1<<RX_START_EN) | (1<<RX_END_EN) | (1<<TX_END_EN);

	// Transceiver Clear Channel Assessment (CCA) -- PHY_CC_CCA
	// This register is used to set the channel. CCA_MODE should default
	// to Energy Above Threshold Mode.
	// Channel should be between 11 and 26 (2405 MHz to 2480 MHz)
	PHY_CC_CCA = (PHY_CC_CCA & 0xE0) | ATMEGA128RF_CHANNEL; // Set the channel

	// Finally, we'll enter into the RX_ON state. Now waiting for radio RX's, unless
	// we go into a transmitting state.
	TRX_STATE = (TRX_STATE & 0xE0) | RX_ON; // Default to receiver
  #endif
}


/**************************************************************************************/
/***************               ATmega128RF RX reading              ********************/
/**************************************************************************************/
#if defined(ATMEGA128RF)

// This function sends a string of characters out of the radio.
// Given a string, it'll format a frame, and send it out.
void rfPrint(char * toPrint, unsigned char length)
{
  uint8_t frame[RF_BUFFER_SIZE];  // We'll need to turn the string into an arry

  // Cut the data if length is > 126, return.
  if(length > RF_BUFFER_SIZE-2)
	  length = RF_BUFFER_SIZE-2;

  // Fill our array with bytes in the string, it doesn't send the null
  for (int i=0; i<length; i++)
  {
    frame[i] = toPrint[i];
  }

  // Transceiver State Control Register -- TRX_STATE
  // This regiseter controls the states of the radio.
  // Set to the PLL_ON state - this state begins the TX.
  TRX_STATE = (TRX_STATE & 0xE0) | PLL_ON;  // Set to TX start state
  while(!(TRX_STATUS & PLL_ON));  // Wait for PLL to lock

  // Start of frame buffer - TRXFBST
  // This is the first byte of the 128 byte frame. It should contain
  // the length of the transmission.
  // Protocol, total 128 bytes = <lenght 1 byte> <Data 126 bytes> <FCS 1 byte>
  TRXFBST = length + 2;
  memcpy((void *)(&TRXFBST+1), frame, length);
  // Transceiver Pin Register -- TRXPR.
  // From the PLL_ON state, setting SLPTR high will initiate the TX.
  TRXPR |= (1<<SLPTR);   // SLPTR high
  TRXPR &= ~(1<<SLPTR);  // SLPTR low

  // After sending the byte, set the radio back into the RX waiting state.
  TRX_STATE = (TRX_STATE & 0xE0) | RX_ON;
}

// Returns the length of the data if there is data available, 0 if not.
unsigned int rfAvailable()
{
  return rxDataLength;
}

// This function returns the current received data.
char * rfRead()
{
  rxDataLength =0;
  return radioRXBuffer;
}

// This interrupt is called when radio TX is complete. We'll just
// use it to turn off our TX LED.
ISR(TRX24_TX_END_vect)
{

}

// This interrupt is called the moment data is received by the radio.
// We'll use it to gather information about RSSI -- signal strength --
// and we'll turn on the RX LED.
ISR(TRX24_RX_START_vect)
{
  rssiRaw = PHY_RSSI;  // Read in the received signal strength
}

// This interrupt is called at the end of data receipt. Here we'll gather
// up the data received. And store it into a global variable.
ISR(TRX24_RX_END_vect)
{
  uint8_t length;
  uint8_t frame_checksum;

  // The received signal must be above a certain threshold.
  if (PHY_RSSI & (1<<RX_CRC_VALID))
  {
    // The length of the message will be the first byte received.
    length = TST_RX_LENGTH;
    // The remaining bytes will be our received data.
    memcpy(&radioRXBuffer[0], (void*)&TRXFBST, length);
    // Get the frame checksum Sequence (FCS)
    frame_checksum = radioRXBuffer[length-2];
    // Replace the checksum with a null or '\0'
    radioRXBuffer[length-2] = '\0';
  }

}

#endif

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
void computeRC() {
	uint8_t chan;

	//Data comes from the ATMEGA128RF internal transceiver
	if(rfAvailable()>=RC_CHANS*sizeof(int16_t)) {
		memcpy(rcData, rfRead(), RC_CHANS*sizeof(int16_t));
		#if defined(FAILSAFE)
			failsafeCnt = 0;
		#endif
	}

	// rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
	if (rcSerialCount > 0){
		for (chan = 0; chan < RC_CHANS; chan++) {
			if (chan<8) {
				rcSerialCount --;
				if (rcSerial[chan] >900) {rcData[chan] = rcSerial[chan];} // only relevant channels are overridden
			}
		}
	}
}
