#include "main.h"
#define IMU_DATA "000000000000000000"
/********************************************************************************************************************************/
#define test1 "B01"

#define IS_LEAP(year) (years%4 == 0)

int years,ryears; //year calculation for lckl and rclk
static int isInitialized = 0;
static uip_ipaddr_t my_addr;
static uip_ipaddr_t dest_addr;
static uip_ipaddr_t bcast_ipaddr;
static uip_lladdr_t bcast_lladdr = {{0, 0, 0, 0, 0, 0, 0, 0}};
static struct uip_udp_conn *listen_conn;
static struct uip_udp_conn *send_conn;
static struct etimer periodic_timer1,rtc_timer;
FIL fil;
FRESULT res;
UINT bw;
FATFS fat;
DIRS dir;
FILINFO fno;
char file_name[14]= "WRIST1";
#define DLE	0x10
#define SOH     0x01
#define EOT	0x04
#define PKT_LEN 24

//Set of expected Packets
char data1[3]= {'B','0','1'};
char data2[3]= {'B','0','2'}; //1
char data3[3]= {'B','0','3'}; //2
char data4[3]= {'B','0','4'};
char data5[3]= {'N','0','5'};
//char data5[3]={'B','0','5'};

int cycle=0;

/********************************************************/
int LED1_STATE_CHECK=0;
int LED2_STATE_CHECK=0;
int tag0=0;
int vat1=0;
int vat=0;
int time_recvd=0;
int SD_BUSY=0;
unsigned char RSSI_PACKET[11]= {0,0,0,0,0,0,0,0,0,0,0};
unsigned char IMU_PACKET[PKT_LEN]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char outPkt[50];
int BLE=0;
/*********************************************************/

pkt_data_IMU IMU;//36BYtes



/*********************************************************/
//TODO: Added by Amy, 11/12/2015
#define	GPT_CONF_BASE 		GPT_0_BASE
#define IOC_CONF_SEL 		IOC_PXX_SEL_GPT0_ICP1  // act as GPTimer0 ICP1
#define PWM_GPIO_CONF_PORT 	GPIO_A_NUM
#define PWM_GPIO_CONF_PIN 	1

void enable_gptimer(){
	/* Enable */
	REG(GPT_CONF_BASE + GPTIMER_CTL) |= GPTIMER_CTL_TAEN;
}

void disable_gptimer(){
	/* Stop the timer */
	REG(GPT_CONF_BASE + GPTIMER_CTL) = 0;
}

#define GPTIMER_ON 1
#define GPTIMER_OFF 0
int get_gptimer_state(){
	if(REG(GPT_CONF_BASE + GPTIMER_CTL) == 0)
		return GPTIMER_OFF;
	else
		return GPTIMER_ON;
}


//Config PA1 as an PWM output pin
void initPWM() {
	//16Mhz 
	//printf("init PWM start\n");
	/* Enable module clock for the GPTx in Active mode, GPT0 clock enable, CPU running */
	REG(SYS_CTRL_RCGCGPT) |= SYS_CTRL_RCGCGPT_GPT0;

	disable_gptimer();

	/* Use 16-bit timer */
	REG(GPT_CONF_BASE + GPTIMER_CFG) = 0x04;

	/* Configure PWM mode, 0x00000008 Timer A alternate mode. */
	REG(GPT_CONF_BASE + GPTIMER_TAMR) = 0;
	REG(GPT_CONF_BASE + GPTIMER_TAMR) |= GPTIMER_TAMR_TAAMS;

	/* To enable PWM mode, the TACM bit must be cleared and the lowest 2 bits 
	   (TAMR) field must be configured to 0x2.
	   GPTIMER_TnMR bit values, GPTIMER_TAMR_TAMR_PERIODIC is 0x00000002 */
	REG(GPT_CONF_BASE + GPTIMER_TAMR) |= GPTIMER_TAMR_TAMR_PERIODIC;

	//how often the counter is incremented:  every  pre-scaler / clock 16000000 seconds
	REG(GPT_CONF_BASE + GPTIMER_TAPR) = 0; 	//PRESCALER_VALUE  

	/* Set the start value (period), count down */
	REG(GPT_CONF_BASE+ GPTIMER_TAILR) = 16000;//0xF42400; 	//16000: 3E80, so period is 1s.  16000000:F42400

	/* Set the deassert period */
	REG(GPT_CONF_BASE + GPTIMER_TAMATCHR) = 12800;//0x7A1200; //800: 0x1F40, so the duty rate is 50%, 8000000: 7A1200

	// Defined in contiki/cpu/cc2538/dev/ioc.h
	/* Function select for Port:Pin.
	   The third param sel can be any of the IOC_PXX_SEL_xyz defines. 
	   For example, IOC_PXX_SEL_UART0_TXD will set the port to act as UART0 TX.

     	   Selects one of the 32 pins on the four 8-pin I/O-ports (port A, port B, port C, and port D) to be the GPT0OCP1.
           Configure pin : PA:1 selected as GPT0OCP1*/
	ioc_set_sel(PWM_GPIO_CONF_PORT, PWM_GPIO_CONF_PIN, IOC_CONF_SEL);

	/* Set Port:Pin override function, IOC_OVERRIDE_OE: Output */
	ioc_set_over(PWM_GPIO_CONF_PORT, PWM_GPIO_CONF_PIN, IOC_OVERRIDE_OE);

	/* Configure the pin to be under peripheral control with PIN_MASK of port with PORT_BASE.*/
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(PWM_GPIO_CONF_PORT), GPIO_PIN_MASK(PWM_GPIO_CONF_PIN));
	
	enable_gptimer();

	//printf("init PWM end\n");
}


//***********************************************************************************************************************************//
void byteStuff (unsigned char* dataPkt, unsigned char len, unsigned char* outPkt, unsigned char* outLen) {
	outPkt [0] = DLE;
	outPkt [1] = SOH;
	unsigned char j = 2;
	unsigned char i = 0;
	for (i = 0; i < len; i++) {
		if (dataPkt [i] != DLE)
			outPkt [j++] = dataPkt [i];
		else {
			outPkt [j++] = DLE;
			outPkt [j++] = DLE;
		}
	}
	outPkt [j++] = DLE;
	outPkt [j++] = EOT;

	*outLen = j;
}





void Delay_ms(unsigned int n) { //maximal n =65000=65s
	while(n--) {
		unsigned int k=4000; //K=4000 when SysCtrlIOClockGet()=32Mhz
		//unsigned int k=SysCtrlIOClockGet()/(32000000/4000);
		while(k--) {
			asm("nop");
		}
	}
}





static void
recv_handler(void) {
	char* str;
	int z;
	int tag1=0;
	int tag2=0;
	int tag3=0;
	int tag4=0;
	int tag5=0;
//int tag5=0;
	str = uip_appdata;
	//printf("string is: '%s':",str);
	if (BLE==0) {
		set_bluetooth();
		initPWM();
		BLE=1;
	}



// For Beacon 1
	for (z=0; z<=2; z++) {
		if (str[z]==data1[z]) {
			tag1=1;
		} else {
			tag1=0;

			break;
		}

	}

// For Beacon 2

	for (z=0; z<=2; z++) {
		if (str[z]==data2[z]) {
			tag2=1;
		} else {
			tag2=0;

			break;
		}

	}


// For Beacon 3

	for (z=0; z<=2; z++) {
		if (str[z]==data3[z]) {
			tag3=1;
		} else {
			tag3=0;

			break;
		}

	}


//For Beacon 4
	for (z=0; z<=2; z++) {
		if (str[z]==data4[z]) {
			tag4=1;
		} else {
			tag4=0;

			break;
		}

	}

	for (z=0; z<=2; z++) {
		if (str[z]==data5[z]) {
			tag5=1;
		} else {
			tag5=0;

			break;
		}

	}





	if (tag5==1) {
		cycle=1;
		if (vat1==0) {
			GPIO_SET_OUTPUT(GPIO_C_BASE, GPIO_PIN_MASK(1));
			GPIO_SET_PIN(GPIO_C_BASE, GPIO_PIN_MASK(1));
			vat1=1;
		}

		else if (vat1==1) {

			GPIO_CLR_PIN(GPIO_C_BASE, GPIO_PIN_MASK(1));
			vat1=0;
		}

	} else if (tag1==1)

	{
		IMU.RSS1=packetbuf_attr(PACKETBUF_ATTR_RSSI);

		//TODO:  when near beacon1, start vibrate.
		if(IMU.RSS1 <= 100)
		{
			if(get_gptimer_state() == GPTIMER_OFF){}
				//enable_gptimer();
		}
		else
		{
			if(get_gptimer_state() == GPTIMER_ON){}
				//disable_gptimer();
		}

	}

	else if (tag2==1)

	{
		IMU.RSS2=packetbuf_attr(PACKETBUF_ATTR_RSSI);

	} else if (tag3==1)

	{
		IMU.RSS3=packetbuf_attr(PACKETBUF_ATTR_RSSI);

	} else if (tag4==1) {
		IMU.RSS4=packetbuf_attr(PACKETBUF_ATTR_RSSI);
	}
}



/*
 * Baud rate defines used in uart_init() to set the values of UART_IBRD and
 * UART_FBRD in order to achieve the configured baud rates.
 */
#define UART_CLOCK_RATE       16000000 /* 16 MHz */
#define UART_CTL_HSE_VALUE    0
#define UART_CTL_VALUE        (UART_CTL_RXE | UART_CTL_TXE | (UART_CTL_HSE_VALUE << 5))

/* DIV_ROUND() divides integers while avoiding a rounding error: */
#define DIV_ROUND(num, denom) (((num) + (denom) / 2) / (denom))

#define BAUD2BRD(baud)        DIV_ROUND(UART_CLOCK_RATE << (UART_CTL_HSE_VALUE + 2), (baud))
#define BAUD2IBRD(baud)       (BAUD2BRD(baud) >> 6)
#define BAUD2FBRD(baud)       (BAUD2BRD(baud) & 0x3f)
//*********************************************************************************************************************//
// Change UART baud rate from 9600 to 115200 after we set bluetooth frequency to 115200
void set_uart_baud_rate(unsigned int uart_base) {
	/* Make sure the UART is disabled before trying to configure it */
	REG(uart_base + UART_CTL) = UART_CTL_VALUE;

	/* Baud Rate Generation */
	REG(uart_base + UART_IBRD) = BAUD2IBRD(115200);
	REG(uart_base + UART_FBRD) = BAUD2FBRD(115200);

	/* UART Control: 8N1 with FIFOs */
	REG(uart_base + UART_LCRH) = UART_LCRH_WLEN_8 | UART_LCRH_FEN;

	/* UART Enable */
	REG(uart_base + UART_CTL) |= UART_CTL_UARTEN;
}


//*********************************************************************************************************************//

//*****************************************************************************************************************//

void set_bluetooth() {
	GPIO_SET_OUTPUT(GPIO_D_BASE, GPIO_PIN_MASK(3));
	GPIO_SET_PIN(GPIO_D_BASE, GPIO_PIN_MASK(3));
//Make sure the Reset is off on bluetooth


//GPIO_SET_OUTPUT(GPIO_B_BASE, GPIO_PIN_MASK(0));
//GPIO_SET_PIN(GPIO_B_BASE, GPIO_PIN_MASK(0));
	/*
	uart_write_byte(0,'P');
	uart_write_byte(0,'O');
	uart_write_byte(0,'W');
	uart_write_byte(0,'E');
	uart_write_byte(0,'R');
	uart_write_byte(0,' ');
	uart_write_byte(0,'O');
	uart_write_byte(0,'N');
	uart_write_byte(0,'\r');
	*/


	uart_write_byte(0,'R');
	uart_write_byte(0,'E');
	uart_write_byte(0,'S');
	uart_write_byte(0,'T');
	uart_write_byte(0,'O');
	uart_write_byte(0,'R');
	uart_write_byte(0,'E');
	uart_write_byte(0,'\r');
	Delay_ms(1000);

	uart_write_byte(0,'R');
	uart_write_byte(0,'E');
	uart_write_byte(0,'S');
	uart_write_byte(0,'E');
	uart_write_byte(0,'T');
	uart_write_byte(0,'\r');
	Delay_ms(400);

//TODO: 'SET BAUD=115200\r'
	uart_write_byte(0,'S');
	uart_write_byte(0,'E');
	uart_write_byte(0,'T');
	uart_write_byte(0,' ');
	uart_write_byte(0,'B');
	uart_write_byte(0,'A');
	uart_write_byte(0,'U');
	uart_write_byte(0,'D');
	uart_write_byte(0,'=');
	uart_write_byte(0,'1');
	uart_write_byte(0,'1');
	uart_write_byte(0,'5');
	uart_write_byte(0,'2');
	uart_write_byte(0,'0');
	uart_write_byte(0,'0');
	uart_write_byte(0,'\r');
	Delay_ms(400);

	uart_write_byte(0,'w');
	uart_write_byte(0,'r');
	uart_write_byte(0,'i');
	uart_write_byte(0,'t');
	uart_write_byte(0,'e');
	uart_write_byte(0,'\r');
	Delay_ms(400);

	uart_write_byte(0,'R');
	uart_write_byte(0,'E');
	uart_write_byte(0,'S');
	uart_write_byte(0,'E');
	uart_write_byte(0,'T');
	uart_write_byte(0,'\r');
	Delay_ms(400);

//TODO: change UART to 115200 after we set Bluetooth to 115200
	set_uart_baud_rate(UART_CONF_BASE);
	
	uart_write_byte(0,'D');
	uart_write_byte(0,'I');
	uart_write_byte(0,'S');
	uart_write_byte(0,'C');
	uart_write_byte(0,'O');
	uart_write_byte(0,'V');
	uart_write_byte(0,'E');
	uart_write_byte(0,'R');
	uart_write_byte(0,'A');
	uart_write_byte(0,'B');
	uart_write_byte(0,'L');
	uart_write_byte(0,'E');
	uart_write_byte(0,' ');
	uart_write_byte(0,'O');
	uart_write_byte(0,'N');
	uart_write_byte(0,'\r');


	Delay_ms(400);
	uart_write_byte(0,'A');
	uart_write_byte(0,'U');
	uart_write_byte(0,'T');
	uart_write_byte(0,'O');
	uart_write_byte(0,'C');
	uart_write_byte(0,'O');
	uart_write_byte(0,'N');
	uart_write_byte(0,'N');
	uart_write_byte(0,'=');
	uart_write_byte(0,'0');
	uart_write_byte(0,'\r');
	Delay_ms(400);

	uart_write_byte(0,'S');
	uart_write_byte(0,'E');
	uart_write_byte(0,'T');
	uart_write_byte(0,' ');
	uart_write_byte(0,'D');
	uart_write_byte(0,'E');
	uart_write_byte(0,'E');
	uart_write_byte(0,'P');
	uart_write_byte(0,'_');
	uart_write_byte(0,'S');
	uart_write_byte(0,'L');
	uart_write_byte(0,'E');
	uart_write_byte(0,'E');
	uart_write_byte(0,'P');
	uart_write_byte(0,'=');
	uart_write_byte(0,'O');
	uart_write_byte(0,'F');
	uart_write_byte(0,'F');
	uart_write_byte(0,'\r');

	Delay_ms(400);
	uart_write_byte(0,'M');
	uart_write_byte(0,'A');
	uart_write_byte(0,'X');
	uart_write_byte(0,'_');
	uart_write_byte(0,'R');
	uart_write_byte(0,'E');
	uart_write_byte(0,'C');
	uart_write_byte(0,'=');
	uart_write_byte(0,'0');
	uart_write_byte(0,'\r');

	Delay_ms(400);


	uart_write_byte(0,'S');
	uart_write_byte(0,'E');
	uart_write_byte(0,'T');
	uart_write_byte(0,' ');
	uart_write_byte(0,'D');
	uart_write_byte(0,'I');
	uart_write_byte(0,'S');
	uart_write_byte(0,'C');
	uart_write_byte(0,'O');
	uart_write_byte(0,'V');
	uart_write_byte(0,'E');
	uart_write_byte(0,'R');
	uart_write_byte(0,'A');
	uart_write_byte(0,'B');
	uart_write_byte(0,'L');
	uart_write_byte(0,'E');
	uart_write_byte(0,'=');
	uart_write_byte(0,'2');
	uart_write_byte(0,' ');
	uart_write_byte(0,'0');
	uart_write_byte(0,'\r');
	Delay_ms(400);


	uart_write_byte(0,'w');
	uart_write_byte(0,'r');
	uart_write_byte(0,'i');
	uart_write_byte(0,'t');
	uart_write_byte(0,'e');
	uart_write_byte(0,'\r');
	Delay_ms(400);

	uart_write_byte(0,'R');
	uart_write_byte(0,'E');
	uart_write_byte(0,'S');
	uart_write_byte(0,'E');
	uart_write_byte(0,'T');
	uart_write_byte(0,'\r');
	Delay_ms(400);

	uart_write_byte(0,'E');
	uart_write_byte(0,'N');
	uart_write_byte(0,'T');
	uart_write_byte(0,'E');
	uart_write_byte(0,'R');
	uart_write_byte(0,'_');
	uart_write_byte(0,'D');
	uart_write_byte(0,'A');
	uart_write_byte(0,'T');
	uart_write_byte(0,'A');
	uart_write_byte(0,'\r');

	Delay_ms(400);

}

//**********************************************************************************************************************//



//***********************************************************************************************************************//
unsigned int PKT_NUM = 0;

PROCESS(MPU_DATA, "For Getting the IMU Data");
PROCESS(RSSI_SCAN,"For RSSI Scan");
//PROCESS(RTC, "RTC based on main crystal");
AUTOSTART_PROCESSES(&RSSI_SCAN,&MPU_DATA);
PROCESS_THREAD(MPU_DATA, ev, data) {
	static struct etimer sdtimer;

	PROCESS_BEGIN();

	if(isInitialized==0) {


// Turn off 3.3-V domain (lcd/sdcard power, output low)
//GPIOPinTypeGPIOOutput(BSP_3V3_EN_BASE, BSP_3V3_EN);
//GPIOPinWrite(BSP_3V3_EN_BASE, BSP_3V3_EN, 0);
		GPIO_SET_OUTPUT(BSP_3V3_EN_BASE, BSP_3V3_EN);
		GPIO_CLR_PIN(BSP_3V3_EN_BASE, BSP_3V3_EN);

// If 3.3-V domain is initially off, make sure it's off >1 ms for a complete
// sd card power cycle

// Approx 10 ms delay
		Delay_ms(10);

// Enable 3.3-V domain (it takes <= 600 us to stabilize)
//GPIOPinWrite(BSP_3V3_EN_BASE, BSP_3V3_EN, BSP_3V3_EN);  // high
		GPIO_SET_PIN(BSP_3V3_EN_BASE, BSP_3V3_EN);
		Delay_ms(100);//100ms

//Disable LCD
//GPIOPinTypeGPIOOutput(GPIO_B_BASE, (5));
//GPIOPinWrite(GPIO_B_BASE, (5), (5));
		GPIO_SET_OUTPUT(GPIO_B_BASE, GPIO_PIN_MASK(5));
		GPIO_SET_PIN(GPIO_B_BASE, GPIO_PIN_MASK(5));
		GPIO_SET_INPUT(GPIO_B_BASE, GPIO_PIN_MASK(3));
		GPIO_CLEAR_INTERRUPT(GPIO_B_BASE, 0xFF);
		GPIO_ENABLE_INTERRUPT(GPIO_B_BASE, 0X08);
		GPIO_DETECT_RISING(GPIO_B_BASE, 0X08);

//Turn on Bluetooth
//GPIO_SET_OUTPUT(GPIO_D_BASE, GPIO_PIN_MASK(3));
//GPIO_SET_PIN(GPIO_D_BASE, GPIO_PIN_MASK(3));
		uart_init(UART_CONF_BASE);

		GPIO_SET_OUTPUT(GPIO_B_BASE, GPIO_PIN_MASK(6));
		GPIO_CLR_PIN(GPIO_B_BASE, GPIO_PIN_MASK(6));

		clock_delay(6000);
		GPIO_SET_PIN(GPIO_B_BASE, GPIO_PIN_MASK(6));
		clock_delay(6000);
		init_i2c();
		clock_delay(6000);
		init_MPU9150 ();


		isInitialized = 1;



	}



	while(1) {

//  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sdtimer));

//GPIO_SET_PIN(GPIO_C_BASE, lGPIO_PIN_MASK(2));
		etimer_set(&sdtimer, CLOCK_SECOND/400);    //TODO: change from 80 ---> 200

		PROCESS_YIELD();


//GPIO_SET_PIN(GPIO_C_BASE, GPIO_PIN_MASK(1));
#define GPIO_B_BASE             0x400DA000  // GPIO
#define GPIO_PIN_3              0x00000008  // GPIO pin 3

		uint32_t gpio = REG(GPIO_B_BASE + (0x00000000 + (GPIO_PIN_3 << 2))) & GPIO_PIN_3;
		if(gpio) {
			read_sensor_data(IMU.Payload);

			PKT_NUM ++;
			IMU_PACKET[0]=IMU.Payload[0];
			IMU_PACKET[1]=IMU.Payload[1];
			IMU_PACKET[2]=IMU.Payload[2];
			IMU_PACKET[3]=IMU.Payload[3];
			IMU_PACKET[4]=IMU.Payload[4];
			IMU_PACKET[5]=IMU.Payload[5];
			IMU_PACKET[6]=IMU.Payload[6];
			IMU_PACKET[7]=IMU.Payload[7];
			IMU_PACKET[8]=IMU.Payload[8];
			IMU_PACKET[9]=IMU.Payload[9];
			IMU_PACKET[10]=IMU.Payload[10];
			IMU_PACKET[11]=IMU.Payload[11];
			IMU_PACKET[12]=IMU.Payload[12];
			IMU_PACKET[13]=IMU.Payload[13];
			IMU_PACKET[14]=IMU.Payload[14];
			IMU_PACKET[15]=IMU.Payload[15];
			IMU_PACKET[16]=IMU.Payload[16];
			IMU_PACKET[17]=IMU.Payload[17];

			if (cycle==1) {

				IMU_PACKET[18]=IMU.RSS1;
				IMU_PACKET[19]=IMU.RSS2;
				IMU_PACKET[20]=IMU.RSS3;
				IMU_PACKET[21]=IMU.RSS4;

				if (vat==0) {
					GPIO_SET_OUTPUT(GPIO_C_BASE, GPIO_PIN_MASK(0));
					GPIO_SET_PIN(GPIO_C_BASE, GPIO_PIN_MASK(0));
					vat=1;
				}

				else if (vat==1) {

					GPIO_CLR_PIN(GPIO_C_BASE, GPIO_PIN_MASK(0));
					vat=0;
				}
				cycle=0;
					
				
			}

			else if (cycle==0) {

				IMU_PACKET[18]=0;
				IMU_PACKET[19]=0;
				IMU_PACKET[20]=0;
				IMU_PACKET[21]=0;
			}

			IMU_PACKET[22]=PKT_NUM & 0x00ff;
			IMU_PACKET[23]=PKT_NUM >> 8 & 0x00ff ;

			unsigned char outLen = 0;
			byteStuff (&IMU_PACKET[0], PKT_LEN, outPkt, &outLen);
			char i;
			for(i=0; i<outLen; i++) {
				uart_write_byte(0,outPkt[i]);

			}

		}

//push_data_to_buffer_imu();


		/*
		if ((stage_imu==1) &&(SD_BUSY==0))
		writedata_imu();
		*/

	}
	PROCESS_END();
}

//**************************************************************************************************************************************************//
PROCESS_THREAD(RSSI_SCAN, ev, data) {
	static struct etimer rssitimer;
	static struct rtimer rt;
	PROCESS_BEGIN();
	// Set the local address
	uip_ip6addr(&my_addr, 0, 0, 0, 0, 0, 0, 0, 0);
	uip_ds6_set_addr_iid(&my_addr, &uip_lladdr);
	uip_ds6_addr_add(&my_addr, 0, ADDR_MANUAL);

	//listening to port 3001 from beacons and sensors
	listen_conn = udp_new(NULL, UIP_HTONS(0), NULL);

	//Server Listen connection is bound too port 4001
	udp_bind(listen_conn, UIP_HTONS(RECEIVER_PORT));

	while(1) {
		//printf("inside while\n\r");
		//etimer_set(&rssitimer, CLOCK_SECOND/200);
		//GPIO_SET_PIN(GPIO_C_BASE, GPIO_PIN_MASK(1));
		//PROCESS_YIELD();
		PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
		if ((ev == tcpip_event)) {
			recv_handler();
		}
	}
	PROCESS_END();
}


//*****************************************************************************************************************************************//

