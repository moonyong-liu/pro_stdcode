/**
  ******************************************************************************
  * @file    stdcode.h
  * @author  LHw
  * @version V1.0
  * @date    2017/07/14
  * @brief   standard function hearder, all c file need include this
  ******************************************************************************
  * @attention
	* @updateRecord
	# who: lHw | when: 20170829 | content:
	1. add _FREPackage,DETECAXIS,DETECFLG_AXIS
	2. add saveFreqAxesCap();
	# who: lHw | when: 20170825 | content:
	1.add eia485Maintain();
	# who: lHw | when: 20170824 | content:
	1.add  (*_pClrBufIdx)(u16)
	2.add _DCHPack;
	# who: lHw | when: 20170822 | content:
	1.add _JMQPack,_IOKPack
	# who: lHw | when: 20170817 | content:
	1.modi axisfun.c func
	# who: lHw | when: 20170815 | content:
	1.add Locomotive,_BufPointer
	2.add Locomotive system define
	# who: lHw | when: 20170725 | content:
	1.add typedef "_pClrTimFlgAxis30s"
	2.add axisfun.c func.
	# who: lHw | when: 20170724 | content:
	1.add comment with every struct, define, enmu ...
  ******************************************************************************
  */
	
#ifndef _STDCODE_H_
#define _STDCODE_H_

/*include--------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stdio.h"
#include "string.h"

/* Memory Alignment */
#pragma pack(1)

/*variable-------------------------------------------------------------------*/
/*define --------------------------------------------------------------------*/

#define PACKSIZE			256										// Master package size define				
#define MINPACKSIZE   7
#define DATASIZE      PACKSIZE-MINPACKSIZE

/* Locomotive system define */
/* max=32 nodes : 1111 1111 1111 1111 1111 1111 1111 1111*/
#define NODESUM      0x0000001f        // 485 device number 
#define NODENUM      5                 // 485 device number
#define PASSNUM      4                 // H-A axes number
#define MINJuGeNODE  3        // min juGement 485 device number

/* count how many 1b */
#define __COUNT(_val,_cnt) while(_val){_val &= _val-1;_cnt++;}

/* magnetic define */
enum{ MAGNETICA=1,				// Magnetic steel A activation flag
			DETECAXIS=1,        // detection axis
			MAGNETICB=2,				// Magnetic steel B activation flag
	    DETECAXES=2,        // detection axes
			DETCSUCCESS=3};			// Detection success flag

			
#define DETECFLG_AXES 0x01
#define DETECFLG_SUCC 0x02
#define DETECFLG_AXIS 0x04
			
/*typedef--------------------------------------------------------------------*/

/* speed detect device struct */
typedef struct{
	u8 Head;					// 0x02
	u8 Version;				
	/* 任意轴触发磁钢A或B的时间间隔 */
	u8 IntervalsH;		// intervals high 
	u8 IntervalsL;		// intervals low
	u8 Dirc; 					// Dircetion
	u8 Illum; 				// Illumination
	u8 TempH; 				// temperature high
	u8 TempL; 				// temp. low
	u8 Count; 				// Transmit count
	u8 CheckSum;		
	u8 Tail;					// 0x03
}_CSKPack;
/* io_decoder device struct */
/* IOCard */
typedef struct{
	u8 Head;					// 0x01
	u8 AddrH;					// board address high
	u8 AddrL;		  		// board address high 
	u8 CMDa;					// command 1
	u8 CMDb; 					// command 2
	u8 CMDc; 					// command 3
	u8 Port; 					// port number
	u8 DataA; 				// data 1
	u8 DataB; 				// data 2
	u8 CheckSum;		
	u8 Tail;					// 0x04
}_IOKPack;

/* Decoder  */
typedef struct{
	u8 HeadA;					
	u8 HeadB;					
	u8 HeadC;
	u8 HeadD;
	u8 Data[4];
}_JMQPack ;

/* count axes device struct */
typedef struct{
	u8 Head;					  // 0x40
	u8 MagneticNu;		  // magnetic number
	u8 AxisCnt;         // Axis count
	/* 两轴时间间隔 */
	u8 AxesInterTimH;		// intervals high 
	u8 AxesInterTimL;		// intervals low
	u8 AxisFea; 				// Axis feature
	u8 AxisPro; 				// Axis proportion
	u8 Version; 				// version
	u8 CheckSum;			
	u8 Tail;						//0x26
}_JZKPack;

/* dch device struct */
typedef struct{
	u8 Head;					  // 0x40
	u8 MagneticNu;		  // magnetic number
	u8 AxisCnt;         // Axis count
	/* 任意轴触发磁钢A或B的时间间隔 */
	u8 IntervalsH;		  // intervals high 
	u8 IntervalsL;		  // intervals low
	u8 AxisFea; 				// Axis feature
	u8 AxisPro; 				// Axis proportion
	/* 两轴时间间隔 */
	u8 AxesInterTimH;		// intervals high 
	u8 AxesInterTimL;		// intervals low
	u8 Dirc; 					  // Dircetion
	u8 Version; 				// version
	u8 CheckSum;			
	u8 Tail;					  // 0x26
}_DCHPack;

/* over run device struct */
typedef struct{
	u8 Head;		
	u8 IN1;				
	u8 IN2;		
	u8 IN3;	
	u8 IN4; 			
	u8 IN5; 			
	u8 IN6; 			
	u8 IN7; 		
	u8 IN8; 
	u8 IN9;
	u8 CheckSum;
	u8 Tail;				
}_CXKPack;

/* over run device struct - version b */
typedef struct{
	u8 Head;				
	u8 INline[9];				   
	u8 CheckSum;	
	u8 Tail;			
}_CXKPack_Vb;

/* locomotive -----------------------*/
/* slave collect device TX struct */
typedef union{
	u8 _BYTE;
	struct{
		u8 Switch : 1;
		u8 Ovspd  : 1;
	  u8 NDF    : 6;
	}_BIT;
}_SCDNodeSta;

typedef struct{ u8 AxesH; u8 AxesL;	}_Axes_byte;
typedef union{ u16 Axes_16b; _Axes_byte Axes_8b; }_Axes;

typedef struct{
	_Axes Axes;
	_SCDNodeSta State;
	u8 CheckSum;
}_SCDContentTx;

/* slave collect device content RX struct */
typedef struct{
	u8 CheckSum;
}_SCDContentRx;

/* master collect device content Tx struct */
typedef struct{
	u8 CheckSum;
}_MCDContentTx;

/* master collect device message RF Tx struct */
typedef union{
	u8 _BYTE;
	struct{
		u8 CarIn : 1;
		u8 Ovspd : 1;
	  u8 NDF   : 6;
	}_BIT;
}_MCDNodeSta;

typedef struct{
	_MCDNodeSta Node[NODENUM];
	u8 CheckSum;
}_MCDMESRFTx;

/* multiple protocol union */
typedef union{
	uint8_t	Data[DATASIZE];
	_SCDContentTx ScdDataTx;
	_SCDContentRx ScdDataRx;
	_MCDContentTx McdDataTx;
	_MCDMESRFTx   McdRFDaTx;
}_MultipPack;

/* ji che contrl protocol */
typedef struct{
	u8 HeadA;
	u8 HeadB;
	u8 DataLen;
	u8 SourAddr;
	u8 TargAddr;	
	u8 MDataType;
	u8 SDataType;	
	_MultipPack Data;	
}_JCKZPackage;

/* frequency detect protocol */
typedef struct{
	u8 ASIIC_I;
	u8 ASIIC_S;
	u8 ASIIC_P;
	u8 ASIIC_D;	
	u8 ASIIC_M;
	u8 FreqIH;
	u8 FreqIM;
	u8 FreqIL;
	u8 Space1;
	u8 ASIIC_q;
	u8 ASIIC_s;
	u8 ASIIC_p;
	u8 ASIIC_d;	
	u8 ASIIC_m;
	u8 FreqQH;
	u8 FreqQM;
	u8 FreqQL;
	u8 Space2;
	u8 Unit1;
	u8 Unit2;
	u8 per;
	u8 Unit3;
}_FREPackage;

/* SYS CONFIG STRUCT */
typedef struct{
	u8 HumbeinMode;
}_SYSConfig;

/* locomotive system node struct */
typedef struct{
	u16 IndexEnd;
	u16 IndexCur;
	u16 Loop;
}_IndexRx;

typedef struct{
	u16 LastAxes;         // last axes count
	u16 CurrAxes;         // current axes count
	s32 DiffAxes;         // current - last axes count
	s32 IncrAxes;         // (next next node's DiffAxes - next node's DiffAxes)
}_NodeAxes;

typedef struct{
	u32 NodeMessOk;
	u32 NodeLock;
	u32 NodeSwitch;
}_NodeFlags;

typedef struct{
	_NodeAxes NodeAxes[NODENUM];
	_NodeFlags NodeFlags;
}_NodeInfo;

/* ALL package struct in here */
typedef union{
	uint8_t		    Data[PACKSIZE];
	_CSKPack    	CskPack;					// son 'speed detection' package
	_JZKPack      JzkPack;          // son 'count axes device' package
	_CXKPack      CxkPack;          // son 'over run ' package
	_CXKPack_Vb   CxkPack_Vb;
	_JCKZPackage  CjkPack_sla;      // son 'ji che contrl ' package, slave device
	_JCKZPackage  CjkPack_mas;      // son 'ji che contrl ' package, master device
	_JCKZPackage  SysConfig;
	_IOKPack      IokPack;          // son 'io card device' package
	_JMQPack      JmqPack;          // son 'decoder device' package
	_DCHPack      DchPack;          // son 'da che hao device' package
	_FREPackage   FrePack;
}_Package;										    // Master package 

/* breath flag */
typedef struct{ u8 BrethFlg; }_SPEflag;
/* falling/rising trigger sign */
typedef enum{	FALLINGtRI=1,	RISINGtRI=2 }_extDetType; 

/* count axis device flag */
typedef struct{
	u16 MissMagnA;
	u16 MissMagnB;
	u16 CountAxis;
	u16 LastMagnInterTim;  // last two magnatic interval time
	u16 LastAxesInterTim;  // last two axes interval time
	u8  AxisFeaflag;
}_JZflag;

/* call back pointer to function  */
/* clear timer flag, Axis Ns */
typedef void (*_pClrTimFlgAxisNs)(void);
/* with paramater */
typedef u8 (*_pClrBufIdx)(u16);
typedef void (*_pDlyfun)(u8);

/* delay.c func -------------------------------------------------------------*/
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

/* nvicfg.c func ------------------------------------------------------------*/
u8 nviconfig(	uint8_t 	 IRQn, uint8_t         Prio, 
							uint8_t SubPrio, FunctionalState sta );

/* usart.c func -------------------------------------------------------------*/

typedef struct{
	u8* RxCacheBuf;
	u8* TxCacheBuf;
	u8* RxDmaBuf;
	u8* TxDmaBuf;
	u16* TxIndex;
	_IndexRx* RxIndex;
	FlagStatus* DMATxFlg;
	u16 RxBufLen;
	u16 TxBufLen;
	u16 RxDmaLen;
	u16 TxDmaLen;
}_BufPointer;

/* usart1 */
#define U1RXBUFLEN    	1024								//USART1 RX cache BUFFER SIZE
#define U1TXBUFLEN    	1024								//USART1 RX cache BUFFER SIZE
#define U1RXDMABUFLEN 	1024 	      			//USART1 RX DMA BUFFER SIZE
#define U1TXDMABUFLEN 	1024								//USART1 TX DMA BUFFER SIZE
#define U1RXDMACH DMA1_Channel5						//USART1 RX DMA CHANNEL
#define U1TXDMACH DMA1_Channel4					  //USART1 TX DMA CHANNEL
#define U1PERIADDR (u32)(&USART1->DR) 		//USART1 DMA Peripheral Address
#define IS_U1TX_DATA_LEN_LEGAL(LEN) (LEN<=U1TXBUFLEN)	

/* usart2 */
#define U2RXBUFLEN    	1024								//USART2 RX cache BUFFER SIZE
#define U2TXBUFLEN    	1024								//USART2 RX cache BUFFER SIZE
#define U2RXDMABUFLEN 	1024 	      			//USART2 RX DMA BUFFER SIZE
#define U2TXDMABUFLEN 	1024								//USART2 TX DMA BUFFER SIZE
#define U2RXDMACH DMA1_Channel6						//USART2 RX DMA CHANNEL
#define U2TXDMACH DMA1_Channel7						//USART2 TX DMA CHANNEL
#define U2PERIADDR (u32)(&USART2->DR) 		//USART2 DMA Peripheral Address
#define IS_U2TX_DATA_LEN_LEGAL(LEN) (LEN<=U2TXBUFLEN)	


/* usart3 */
#define U3RXBUFLEN    	1024								//USART3 RX cache BUFFER SIZE
#define U3TXBUFLEN    	1024								//USART3 RX cache BUFFER SIZE
#define U3RXDMABUFLEN 	1024 	      			//USART3 RX DMA BUFFER SIZE
#define U3TXDMABUFLEN 	1024								//USART3 TX DMA BUFFER SIZE
#define U3RXDMACH DMA1_Channel3						//USART3 RX DMA CHANNEL
#define U3TXDMACH DMA1_Channel2						//USART3 TX DMA CHANNEL
#define U3PERIADDR (u32)(&USART3->DR) 		//USART3 DMA Peripheral Address
#define IS_U3TX_DATA_LEN_LEGAL(LEN) (LEN<=U3TXBUFLEN)	


u8 usartCfg( GPIO_TypeDef* GPIOx, USART_TypeDef* USARTx,
						uint16_t GPIO_Pin_Rx, uint16_t GPIO_Pin_Tx,
						uint32_t BaudRate, uint16_t ITdefine,
						uint16_t Flag, FunctionalState sta);
						
u8 usartDmaCfg(	DMA_Channel_TypeDef* Channel, USART_TypeDef* USARTx,
								uint32_t PeriAddr, uint16_t Request,
								uint32_t MemAddr, uint32_t DIR,
								uint32_t Buffsize, uint32_t PeriInc,
								uint32_t MemInc, uint32_t PeriDataSize,
								uint32_t MemDataSize, uint32_t Mode,
								uint32_t Prio, uint32_t M2M,
								uint32_t DMAIT, FunctionalState sta );

u8 usartMaintain(DMA_Channel_TypeDef* DMAy_Cx_Tx, DMA_Channel_TypeDef* DMAy_Cx_Rx,
													uint16_t* TxLen, FlagStatus* DMATxFinFlag);
u8 eia485Maintain(DMA_Channel_TypeDef* DMAy_Cx_Tx, DMA_Channel_TypeDef* DMAy_Cx_Rx,
													uint16_t* TxLen, FlagStatus* DMATxFinFlag, _pDlyfun dlySwiRT);
u8 clearUsartSRDR(USART_TypeDef* USARTx);
u8 usart_send( u8* TarBuf, u8* SrcBuf, u16 Len, u16* indexTxcb, u16 txBuflen );

u8 usart_printf( void* SrcBuf ); // realization in app files
u8 maintainDataFlow(void);       // realization in app files
/* timer.c func -------------------------------------------------------------*/

typedef struct {
	TIM_TypeDef * const TIMx; // TIMx
	u8 DetecFlag;  // detec success flag
	u8 Lock;					// Detection process sign
	u8 Dirc;  				// Direction
	u8 Pair;        	// A->B or B->A make a Pair; A->A or B->B can't make a Pair
	u8 Counter;				// Detection counter
	u16 InterCarry;     // Intervals carry
	u16 Intervals;		// Final Intervals
	u16 IntervalsAB; 	// A->B Intervals
	u16 IntervalsBA; 	// B->A Intervals
}_TimrValus;

typedef struct{
	TIM_TypeDef * const TIMx; // TIMx
	u8 DetecFlag;     // detec success flag
	u8 Lock;					// Detection process sign
	u8 Dirc;  				// Direction
	u8 Pair;        	// A->B or B->A make a Pair; A->A or B->B can't make a Pair
	u8 Counter;				// Detection counter
	u16 InterCarry;     // Intervals carry
	u16 Intervals;		// Final Intervals
	u16 IntervalsAB; 	// A->B Intervals
	u16 IntervalsBA; 	// B->A Intervals
}_TimrFreqValues;
typedef struct {
	u16 Counter;
	u8  Set;
}_TimrFLags;		// Timer set a flag

/* timer check group */
typedef struct{	_TimrValus* pTimVal; u8 MutexSign; }_CheckGroup;

u8 genTimCfg( TIM_TypeDef *TIMx, uint16_t Period, 
							uint16_t 		 Pres, uint16_t Div, 
							uint16_t 		CMode, uint16_t Flag,
							uint16_t ITsource, FunctionalState sta);

u8 genTimCaMpCfg(	TIM_TypeDef 	 *TIMx, uint16_t Channel,
									uint16_t 		Polarity, uint16_t Selec,
									uint16_t 				Pres, uint16_t Filter,
									GPIO_TypeDef 	*GPIOx, uint16_t GPIO_Pin,
									uint16_t 		ITsource, GPIOMode_TypeDef GPIO_Mode,
									uint32_t  GPIO_Remap, FunctionalState sta );
									
u8 genTimCaMpDmaCfg(	DMA_Channel_TypeDef* Channel, TIM_TypeDef* TIMx,
									uint32_t PeriAddr, uint16_t Request,
									uint32_t MemAddr, uint32_t DIR,
									uint32_t Buffsize, uint32_t PeriInc,
									uint32_t MemInc, uint32_t PeriDataSize,
									uint32_t MemDataSize, uint32_t Mode,
									uint32_t Prio, uint32_t M2M,
									uint32_t DMAIT, FunctionalState sta );


/* gpio.c func ---------------------------------------------------------*/

#define LEDBUTTON(gpio,pin,sta) sta ? GPIO_SetBits(gpio, pin) \
																		: GPIO_ResetBits(gpio, pin)
																		
#define SETGPIO(gpio,pin,sta) sta ? GPIO_SetBits(gpio, pin) \
																		: GPIO_ResetBits(gpio, pin)

u8 gpioCfg(	GPIO_TypeDef* GPIOx,uint16_t Pin,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef Speed,FlagStatus sta);

/* extiline.c func ----------------------------------------------------------*/
u8 extiCfg(	GPIO_TypeDef* GPIOx, uint16_t Pin, GPIOMode_TypeDef GMode,
						uint8_t PortSource, uint8_t PinSource, uint32_t Line, 
						EXTIMode_TypeDef EMode,	EXTITrigger_TypeDef Trigger, 
						FunctionalState sta);
						
/* wdg.c func --------------------------------------------------------------*/
void initWWDG(void);
void initIWDG(void);

/* axisfun.c func. ----------------------------------------------------------*/
#define MAGNATICINTER 60000 // two magnatic interval (10e-5m)
u8 checkSum(u8* srcdata, u16 len);
u8 reCheckSum(u8* srcdata, u8 len);
u8 calcCRC8Lsb(u8 value);
u8 calcCRC8BufLsb(u8 *ptr, u8 len);
u8 reCalcCRC8BufLsb(u8 *ptr, u8 len);
u8 saveAxesCap( _pClrTimFlgAxisNs sysENtim,_CheckGroup* timNval,_JZflag* flag,
								 _TimrValus* timAVal, _TimrValus* timBVal );	
u8 saveFreqAxesCap( _pClrTimFlgAxisNs sysENtim,_CheckGroup* timNval,_JZflag* axesFlag,
								 _JZflag* axesBFlag, _TimrValus* timAVal, _TimrValus* timBVal );								 
u8 countAxesProperty(_Package* pPack, _TimrValus* values, _JZflag* flag);

/* Locomotive system --------------------------------------------------------*/

#define HEAD_QH       0x5148
#define HEAD_Q 				0x51
#define HEAD_H 				0x48

/* system config and all device used */
#define HOSTADDRESS   0x00 
#define RFHOSTADDR    0x00
#define BROADCAST     0xff
#define PCADDRESS     0xfe

#define TYPE_M_NULL  0x00
#define TYPE_S_NULL  0x00

#define TYPE_F_SYS       0xff
#define TYPE_S_SYS_HUMAN 0x01
#define TYPE_S_SYS_CLR   0xfe
#define TYPE_S_SYS_RST   0xff

/* slave collect device status */
#define SCDSTA_SWITCH  0x01
#define SCDSTA_OVSPD   0x02
#define SCDSTA_EFGH    0x04
#define SCDSTA_IJKL    0x08

#define SCDSTA_SWITCH_P  0x00
#define SCDSTA_OVSPD_P   0x01
#define SCDSTA_EFGH_P    0x02
#define SCDSTA_IJKL_P    0x03

/* master collect device status */
#define MCDSTA_CARIN   0x01
#define MCDSTA_OVSPD   0x02
#define MCDSTA_EFGH    0x04
#define MCDSTA_IJKL    0x08

#define MCDSTA_CARIN_P   0x00
#define MCDSTA_OVSPD_P   0x01
#define MCDSTA_EFGH_P    0x02
#define MCDSTA_IJKL_P    0x03

#define TYPE_F_MCD      0x07
#define TYPE_S_MCD_INQ  0x01
#define TYPE_S_MCD_CLR  0X02
#define TYPE_S_MCD_SETSPD 
#define TYPE_S_MCD_SEND 0x04

#define TYPE_F_SCD      0x06
#define TYPE_S_SCD_SEND 0x01
#endif /*_STDCODE_H_ */
