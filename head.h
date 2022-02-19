#include <compat/deprecated.h>
#include <util/delay.h>

#define H1On sbi(DDRH,PH1);cbi(PORTH,PH1)	//Rx RS485
#define H1Off sbi(DDRH,PH1);sbi(PORTH,PH1)
#define H2On sbi(DDRH,PH2);cbi(PORTH,PH2)	//Tx RS485
#define H2Off sbi(DDRH,PH2);sbi(PORTH,PH2)
#define H3On sbi(DDRH,PH3);cbi(PORTH,PH3)	//Status
#define H3Off sbi(DDRH,PH3);sbi(PORTH,PH3)
#
//----------------------------------------------------------------------------------
void SystemInit(void);
void zloz_pakiet(void);
void testuj(void);
void send_udp(char *dane, uint8_t ile, uint16_t port);
//----------------------------------------------------------------------
char zmiana;


