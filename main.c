#include <head.h>	
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/iom2560.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "ip_arp_udp.h"
#include "enc28j60.h"
#include "net.h"

//SUT0,BOOTSZ0,BOOTSZ1,CKSEL3

// network defines
static uint8_t mymac[6] = {0x44,0x4f,0x4d,0x30,0x30,0x32}; //mÛj MAC
static uint8_t myip[4] = {192,168,1,100};						//mÛj IP

//static uint8_t mac_dest[6] = {0x02,0x99,0x07,0x81,0x32,0x3a};	// mac WIFI A6Q00VA docelowy
static uint8_t mac_dest[6] = {0xB8,0x27,0xeb,0x35,0x5d,0x0b};
static uint8_t dstip[4] = {192,168,1,2}; 						//docelowy IP a6q00VA

static uint8_t iphdr[9]={0x45,0,0,0x82,0,0,0x40,0,0x20}; //nag≥owek w ip parametry
static uint16_t myport =1200; 								// listen port for udp

// how did I get the mac addr? Translate the first OLEK02 numbers into ascii 
#define BUFFER_SIZE 500
static uint8_t buf[BUFFER_SIZE+1]; 

// #define MAX_FRAMELEN 100 //500       enc28j60.h

char dane[10];							//bufor danych do wys≥ania 

 
uint16_t plen;
uint8_t i=0;

volatile int z=0;  
char msek,test;

uint8_t wejl,wejf,wejk,wejc,wejd;
uint8_t templ,tempf,tempk,tempc,tempd;

//------------------------------------------------------------------------------------------------------------
int main(void){
	msek=0;
	test=0;
	wejl=0;
	wejf=0;
	wejk=0;
	wejc=0;
	wejd=0;
	
	SystemInit();
	enc28j60Init(mymac);
	_delay_ms(20); 
	init_ip_arp_udp(mymac,myip);

	DDRK=0x00;PORTK=0xFF;
	DDRF=0x00;PORTF=0xFF;
	DDRL=0x00;PORTL=0xFF;
	DDRD=0x00;PORTD=0xFF;
	DDRC=0x00;PORTC=0xFF;
	
	testuj();
	zloz_pakiet();
	send_udp(dane,10,myport);
	
	while(1){ 
	 
	//	if(bit_is_clear(PIND,PD7)){enc28j60Init(mymac);}		//do usuniecia
//----------------------------------Obs≥uga enc---wysy≥anie----------------------------------	
	if(test==1){testuj();test=0;}	//testuj co 100ms
	if(zmiana==1){					//jeúli by≥a zmiana to wyúlij pakiet
			zloz_pakiet();
			send_udp(dane,10,myport);
			zmiana=0;
			}
//----------------------------------Obs≥uga enc---odbieranie--------------------------------  
  plen = enc28j60PacketReceive(BUFFER_SIZE, buf);
    if(plen==0){
         continue;
        }
		// arp is broadcast if unknown but a host may also verify the mac address by sending it to a unicast address. 
    if(eth_type_is_arp_and_my_ip(buf,plen)){    //odpowiedz na zapytranie arp jezeli moj ip 
        make_arp_answer_from_request(buf,plen);
		continue;
        }
		// check if ip packets (icmp or udp) are for us:
    if(eth_type_is_ip_and_my_ip(buf,plen)==0){  //to jest to co przepusza udp i ping aledok≥adnie nie wiadomo 
        continue;
        }
    if(buf[IP_PROTO_P]==IP_PROTO_ICMP_V && buf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V){
       make_echo_reply_from_request(buf,plen);		// a ping packet, let's send pong
       continue;
       }
	if(buf[IP_PROTO_P]==IP_PROTO_UDP_V){   // jeúli udp 
		// 	tu obs≥uga bufora 
		//	sprawdzenie nag≥Ûwka ((buf[42]==0x3C)&(buf[43]==0x44)&(buf[44]==0x3E)) 
		//							{ obs≥uga uk≥adu / wyjúÊ itp }
		//	ewentualnie odpowiedü    zloz_pakiet();
			;
		}
//------------------------------------------------------------------------------------------

	}
}

//--------------------------------------------testuj-------------------------------------------
void testuj(void){

wejl=~PINL;
wejf=~PINF;
wejk=~PINK;
wejc=~PINC;
wejd=~PIND;

/* testowy fragment
if(wejc==0xFF) wejk++;
if(wejd==0xFF) wejc++;
if(wejf==0xFF) wejd++;
if(wejl==0xFF) wejf++;
wejl++;
*/

zmiana=0;
if(wejl != templ){templ=wejl;zmiana=1;}
if(wejf != tempf){tempf=wejf;zmiana=1;}
if(wejk != tempk){tempk=wejk;zmiana=1;}
if(wejc != tempc){tempc=wejc;zmiana=1;}
if(wejd != tempd){tempd=wejd;zmiana=1;}
}

//---------------------------------------------------------------------------------------------
void zloz_pakiet(){

dane[0]=0x3C;
dane[1]=0x44;
dane[2]=0x3E; //strcpy(dane,"<D>");

dane[3]=wejl;
dane[4]=wejf;
dane[5]=wejd;
dane[6]=wejc;
dane[7]=wejk;
}

//------------------------------------------------------------------------------------------------------------
void SystemInit(void){

TIMSK1=1<<TOIE1;         // zezwolenie na przerwania z timera 1
TCCR1B=0x05;            // preskaler /1024
TCNT1L=0xE5;			//ustalona wartoùä do odliczania czasu 100msek  1563
TCNT1H=0xF9;			//ustalona wartoùä do odliczania czasu 100msek 

// initialize SPI interface
DDRB  |= 1<<PB2 | 1<<PB1 | 1<<PB0; // mosi, sck output
DDRB&=~(_BV(PINB3));		// MISO is input
PORTB&=~(_BV(PB2));		// MOSI low
PORTB&=~(_BV(PB1));		// SCK low
// master mode and Fosc/2 clock:
SPCR = (1<<SPE)|(1<<MSTR);
SPSR |= (1<<SPI2X); 
sei();
}

//-------------------------------timer T3 odliczanie 100ms-------------------------------------
SIGNAL (SIG_OVERFLOW1){
	msek++;
	test=1;
	if(msek>9) {msek=0;H3On;}//enc28j60Init(mymac);} 
	if(msek==1){H3Off;}
	TCNT1L=0xE5;			//ustalona wartoùä do odliczania czasu 100msek
	TCNT1H=0xF9;			//ustalona wartoùä do odliczania czasu 100msek
	
}

//--------------------------------------------------------------------------------------------
void send_udp(char *dane, uint8_t ile, uint16_t port)
{
uint8_t	i=0;		
        while(i<6){
            buf[ETH_DST_MAC +i]=mymac[i]; // gw mac in local lan or host mac
            buf[ETH_SRC_MAC +i]=mac_dest[i];
            i++;
			}
        buf[ETH_TYPE_H_P] = ETHTYPE_IP_H_V;
        buf[ETH_TYPE_L_P] = ETHTYPE_IP_L_V;
		memcpy(buf+IP_P,iphdr,9);
        // total length field in the IP header must be set:
        buf[IP_TOTLEN_H_P]=0;
        // done in transmit: buf[IP_TOTLEN_L_P]=IP_HEADER_LEN+UDP_HEADER_LEN+datalen;
        buf[IP_PROTO_P]=IP_PROTO_UDP_V;
        i=0;
        while(i<4){
            buf[IP_DST_P+i]=myip[i];
            buf[IP_SRC_P+i]=dstip[i];
            i++;
			}
        // done in transmit: fill_ip_hdr_checksum(buf);
	/*	buf[UDP_SRC_PORT_H_P]=4;
		buf[UDP_SRC_PORT_L_P]=0xb0;	//1200 source port
		buf[UDP_DST_PORT_H_P]=4;
		buf[UDP_DST_PORT_H_P]=0xb0; //1200 dest port
     */ buf[UDP_SRC_PORT_H_P]=port>>8;
		buf[UDP_SRC_PORT_L_P]=port&0xFF;	//1200
		buf[UDP_DST_PORT_H_P]=port>>8;
		buf[UDP_DST_PORT_H_P]=port&0xFF; //1200		
		
		buf[UDP_LEN_H_P]=0;
        // done in transmit: buf[UDP_LEN_L_P]=UDP_HEADER_LEN+datalen;
        // zero the checksum
        buf[UDP_CHECKSUM_H_P]=0;
        buf[UDP_CHECKSUM_L_P]=0;
		
		make_udp_reply_from_request(buf,dane,ile,port);	 //max data len 220bytes -> ip_arp_udp.c
}

