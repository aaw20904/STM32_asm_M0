    INCLUDE stm32f103c8.inc
	THUMB
    AREA    MyLib, CODE, READONLY

  EXPORT gpio_init
gpio_init   PROC
    LDR R0, =0x00000015   ; enable alternate, GPIOC and GPIOA clock (Bit 4 of RCC_APB2ENR)
    LDR R1, =RCC_APB2ENR  ; Load the address of RCC_APB2ENR
    STR R0, [R1]          ; Store the value into RCC_APB2ENR
    BX LR                 ; Return from subroutine
	LTORG
            ENDP
	
  ;---dummy-delay-procedure				
	EXPORT delay_
delay_            PROC
	LDR R2, =0xF4240; 1000000 ;counter value
	LTORG  ; Insert literal pool here
1
    SUBS R2, R2, #0x00000001  ;decrement
	BNE 1             ;until great zero
	BX LR
	LTORG
	               ENDP
	;*********MUST BE UPDATED!
	;FUNCTION initialization counter in simple mode with interrupt	
    ;parameters:  @32bit - [  prescaler(16)| counter(16) b0]  ,@32bit-null	
    EXPORT tim3SimpleModeInitInt 
tim3SimpleModeInitInt  PROC
	;enable clock TIM3
	LDR R3, =0x02 ; TIM3 clock enable
    LDR R0, =RCC_APB1ENR
	LDR R1, [R0] ;load content  APB1ENR
	ORR R1, R1, R3 ;ON bit
	STR R1, [R0]  ;update APB1ENR
	;---enable interrupts
	LDR R0, =TIM3_DIER
	LDR R1, =0x01 ;UIE flag
	STR R1, [R0]  ;set reg
	LDR R0, =0xE000E100      ; NVIC_ISER0 address
	LDR R1, [R0] ; load current content
	ORR R1, R1, #(1 << 2)    ; Enable TIM3 interrupt (position 28 in ISER0)
	STR R1, [R0]  ;save NVIC_ISER0
	;--load prescaler and counter from stack
	POP {R2,R1} ;R3 - is a 'ballast' in this case, because M0 can`t push/pop only one reg
	;--high 16 bit - is a prescaler,low 16 bit is a counter (ARR)
	MOV R3, R1
	LSR R3, R3, #16
    ;A) ----prescaler:
    LDR R0, =TIM3_PSC;
    STR R3, [R0] 
    ;B) counter---
	LDR R0, =TIM3_ARR
    LDR R3, =0x0000FFFF;
    AND R1, R3
	STR R1, [R0]  	
	;-----load constant in count reg
	LDR R0, =TIM3_ARR
	LDR R1, =0x000F4240 ; 1000000
	STR R1, [R0]  ;set reg
	;---optional: enable (Start) counter----
    LDR R0, =TIM3_CR1
	LDR R1, =0x01 ; CEN bit
	STR R1, [R0]
	BX LR  ;return
	LTORG
	               ENDP
	;**********
	;FUNCTION initialization counter in simple mode with interrupt	
    ;parameters:  @32bit - [b31  prescaler| counter b0]  ,@32bit-null
    EXPORT 	tim2SimpleModeInitInt
tim2SimpleModeInitInt  PROC
	;enable clock TIM2
	LDR R3, =0x01 ; TIM2 clock enable
    LDR R0, =RCC_APB1ENR
	LDR R1, [R0] ;load content  APB1ENR
	ORR R1, R1, R3 ;ON bit
	STR R1, [R0]  ;update APB1ENR
	;---enable interrupts
	LDR R0, =TIM2_DIER
	LDR R1, =0x01 ;UIE flag
	STR R1, [R0]  ;set reg
	LDR R0, =0xE000E100      ; NVIC_ISER0 address
	LDR R1, [R0] ; load current content
	ORR R1, R1, #(1 << 28)    ; Enable TIM2 interrupt (position 28 in ISER0)
	STR R1, [R0]  ;save NVIC_ISER0
	;--load prescaler and counter from stack
	 ;R3 - is a 'ballast' in this case, because M0 can`t push/pop only one reg
	;--high 16 bit - is a prescaler,low 16 bit is a counter (ARR)
    ;A) ----prescaler:
    LDR R0, =TIM2_PSC;
	LDR R1, [SP]
	LSR R1, R1, #0x10
    STR R1, [R0] 
    ;B) counter---
	LDR R0, =TIM2_ARR
	LDR R1, [SP]
    LDR R2, =0x0000FFFF;
    AND R1,R1, R2
	STR R1, [R0]  	
	;---optional: enable (Start) counter----
    LDR R0, =TIM2_CR1
	LDR R1, =0x01 ; CEN bit
	STR R1, [R0]
	;--return SP 
	POP {R2,R1}
	BX LR  ;return
	LTORG
	               ENDP
	;====FUNCTION initialize TIM2 CH2 PWM
	; A@[b31 presc(16)| period(16) b0],
	;Width@[b31  -----|  width(16) b0]
    ;example bassing params: PUSH {A,Width}
	EXPORT tim2Pwm1Ch2Setup
tim2Pwm1Ch2Setup     PROC
	 ;enable clock TIM2
	LDR R3, =0x01 ; TIM2 clock enable
    LDR R0, =RCC_APB1ENR
	LDR R1, [R0] ;load content  APB1ENR
	ORR R1, R1, R3 ;ON bit
	STR R1, [R0]  ;update APB1ENR
	;--load prescaler and counter from stack
;A) ----prescaler:
    LDR R0, =TIM2_PSC;
	LDR R1, [SP]
	LSR R1, R1, #0x10
    STR R1, [R0] 
    ;B) counter---
	LDR R0, =TIM2_ARR
	LDR R1, [SP]
    LDR R2, =0x0000FFFF;
    AND R1,R1, R2
	STR R1, [R0]  	
	;C) width
	LDR R1, [SP,#0x8]
	LDR R0, =TIM2_CCR2
	STR R1, [R0]
	; set PWM mode
	LDR R0, =TIM2_CCMR1;
	LDR R1, =0x6000 ; PWM mode1 CH2
	STR R1, [R0];
	; enable channel 2
	LDR R0, =TIM2_CCER
	LDR R1, =0x0010; CH2 enable
	STR R1, [R0]
	;enable preload, start counter
	LDR R0, =TIM2_CR1
	LDR R1, =0x0001; 
	STR R1, [R0];
	ADD SP ,SP, #0x8
	 BX LR
	 LTORG
                  ENDP
	;====FUNCTION initialize TIM2 CH2 Output Compare
	;    A@[b31 presc| period b0],
	;Width@[b31 null |  width b0]
	;example bassing params PUSH {A,Width}
	EXPORT tim2OcCh2Setup 
tim2OcCh2Setup     PROC
	 ;enable clock TIM2
	LDR R3, =0x01 ; TIM2 clock enable
    LDR R0, =RCC_APB1ENR
	LDR R1, [R0] ;load content  APB1ENR
	ORR R1, R1, R3 ;ON bit
	STR R1, [R0]  ;update APB1ENR
	;A) ----prescaler:
    LDR R0, =TIM2_PSC;
	LDR R1, [SP]
	LSR R1, R1, #0x10
    STR R1, [R0] 
    ;B) counter---
	LDR R0, =TIM2_ARR
	LDR R1, [SP]
    LDR R2, =0x0000FFFF;
    AND R1,R1, R2
	STR R1, [R0]  	
	;C) width
	LDR R1, [SP,#0x8]
	LDR R0, =TIM2_CCR2
	STR R1, [R0]
	; set OC mode - toggle on match
	LDR R0, =TIM2_CCMR1;
	LDR R1, =0x3000 ; PWM mode1 CH2
	STR R1, [R0];
	; enable channel 2
	LDR R0, =TIM2_CCER
	LDR R1, =0x0010; CH2 enable
	STR R1, [R0]
	;enable preload, start counter
	LDR R0, =TIM2_CR1
	LDR R1, =0x0001; 
	STR R1, [R0];
	ADD SP,SP,#0x8
	 BX LR
	 LTORG
                  ENDP
;****NOTE ABOUT UART FRACTIONAL DIVIDER
; there are fixed pointer format in BRR register [b15  whole_part   b4|b3 remainder  b0]
; example: Fclk = 25 000 000Hz, required speed 9600 Baud
; divisor (BRR) = Fclk/(speed * 16) = 162.76
;So, whole part  (162) save in b15-b4, it equals to multiplication on 16
; remainder_part = (remainder *16)=0.46*16=12.16, write whole number in b3-b0
; Divisor (BRR) should be: [0x00000A2.C] (a point here is for more conviniency)

;=======FUNCTION uart_init_tx with Interrupt
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  par2@[-|-|-|interrupts_enable]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;interrupts_enable: 0xFFFF -> enable TXE TC interrupts, 0->interrupts disable
  EXPORT  uart_init_tx 
uart_init_tx PROC
	 ;turn on  AHB to clock UART2
	 LDR R0, =RCC_APB1ENR
	 LDR R1, [R0]
	 LDR R2, =(1<<17) ;USART2 Enable
	 ORR R1, R1, R2
	 STR R1, [R0]
	 ;BaudRate
     EOR R0, R0, R0
	 EOR R4,R4,R4
	 LDR R1, [SP]
	 ;extract b0-b15 (BitRate)
	 LDR R2, =0x0000FFFF;
	 AND R1,R1, R2
	 LDR R0, =USART2_BRR
	 ;store result to UART_BRR
	 STR R1, [R0]
	 ;--Data length
	 LDR R1, [SP]
	 LDR R2, =0x01000000;
	 AND R1,R1,R2;
	 LSR R1,R1, #12
     MOV R4, R1
	 ;--stop bits
	 LDR R1, [SP]
	 LDR R0, =0x00030000;
	 AND R1,R1,R0
	 LSR R1,R1,#0x4
	 LDR R0, =USART2_CR2
	 STR R1, [R0] ;store
	 ;--Interrupts ;TXE, TC interrupt enable
	 LDR R1, =(1<<6)|(1<<7)  
	 LDR R0, =USART2_CR1
	 ;enable/disable intrrupts
	 LDR R3, [SP,#0x4];
	 ANDS R3, R1
	 BEQ u_tx_l1
	 ORR R4,R4,R1 ;apply new
     STR R4, [R0]	 
u_tx_l1
	 ;--NVIC set
	 LDR R0, =0xE000E104  ;NVIC_ISER1
	 LDR R1, [R0] ;load current
	 LDR R2, =(1<<6)  ;bit6 
	 ORR R1, R1, R2  ;apply new
	 STR R1, [R0]  ;load new
	 ;--enable transmitter
	 LDR R0, =USART2_CR1
	 LDR R1, [R0]
	 LDR R2, =(1<<13)|(1<<3)  ;UE, TE
	 ORR R1, R1, R2
	 STR R1, [R0]
	 ;free memory
	 ADD SP,SP,#0x8
	BX LR
	LTORG
	ENDP
;=======FUNCTION uart_init_rx with Interrupt
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  par2@[-|interrupt_RX_enable(8)|parity_type(8)|prity_en(8)]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, 0b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;parity_en:   0-> disable parity, 1-> enable parity
;parity_type: 1->odd, 0->even
;interrupt_enable: FF->enable
   EXPORT uart_init_rx
uart_init_rx PROC
	 ;turn on  AHB to clock UART2
	 LDR R0, =RCC_APB1ENR
	 LDR R1, [R0]
	 LDR R2, =(1<<17) ;USART2 Enable
	 ORR R1, R1, R2
	 STR R1, [R0]
	 ;BaudRate
	 EOR R4, R4, R4
     EOR R0, R0, R0
	 LDR R1, [SP]
	 ;extract b0-b15 (BitRate)
	 LDR R2, =0x0000FFFF;
	 AND R1,R1, R2
	 LDR R0, =USART2_BRR
	 ;store result to UART_BRR
	 STR R1, [R0]
	 ;--Data length
	 LDR R1, [SP]
	 LDR R2, =0x01000000;
	 AND R1,R1,R2;
	 LSR R1,R1, #12
	 MOV R4, R1
	 ;STR R1, [R0]  ;store word length
	 ;--stop bits
	 LDR R1, [SP]
	 LDR R0, =0x00030000;
	 AND R1,R1,R0
	 LSR R1,R1,#0x4
	 LDR R0, =USART2_CR2
	 STR R1, [R0] ;store
	 ;--parity enable/disable
	 LDR R0, [SP,#0x4]
	 LDR R1, =0x01;
	 AND R1, R1, R0
	 LSL R1,R1,#0xA
	 ORR R4,R4,R1
	 ;--parity type
	 LDR R0, [SP,#0x4]
	 LDR R1, =0x0100;
	 AND R1, R1, R0
	 LSL R1,R1,#0x1
	 ORR R4,R4,R1
	 ;--Interrupts RX (en/dis) ;RXNE PE interrupt enable
	 LDR R1, =(1<<5)|(1<<8)  
	 LDR R3, [SP,#0x04]
	 LSR R3,R3,#0x10
	 ANDS R3, R3, R1
	 BEQ u_rx_l1
	 ORR R4,R4,R1 ;apply
     LDR R0, =USART2_CR1
     STR R4, [R0] ;store valuse to reg	 
u_rx_l1
	 ;--NVIC set
	 LDR R0, =0xE000E104  ;NVIC_ISER1
	 LDR R1, [R0] ;load current
	 LDR R2, =(1<<6)  ;bit6 
	 ORR R1, R1, R2  ;apply new
	 STR R1, [R0]  ;load new
	 ;--enable receiver
	 LDR R0, =USART2_CR1
	 LDR R1, [R0]
	 LDR R2, =(1<<13)|(1<<2)  ;UE, RE
	 ORR R1, R1, R2
	 STR R1, [R0]
	 ;restore SP
	 ADD SP,SP,#0x8
	BX LR
	LTORG
	ENDP
		
;=======FUNCTION uart_init_duplex with Interrupt
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  par2@[interrupt_tx_enable(8)|interrupt_rx_enable(8)|parity_type(8)|prity_en(8)]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, 0b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;parity_en:   0-> disable parity, 1-> enable parity
;parity_type: 1->odd, 0->even
;interrupt_enable: FF->enable
   EXPORT uart_init_duplex
uart_init_duplex PROC
	 ;turn on  AHB to clock UART2
	 LDR R0, =RCC_APB1ENR
	 LDR R1, [R0]
	 LDR R2, =(1<<17) ;USART2 Enable
	 ORR R1, R1, R2
	 STR R1, [R0]
	 ;BaudRate
	 EOR R4, R4, R4
     EOR R0, R0, R0
	 LDR R1, [SP]
	 ;extract b0-b15 (BitRate)
	 LDR R2, =0x0000FFFF;
	 AND R1,R1, R2
	 LDR R0, =USART2_BRR
	 ;store result to UART_BRR
	 STR R1, [R0]
	 ;--Data length
	 LDR R1, [SP]
	 LDR R2, =0x01000000;
	 AND R1,R1,R2;
	 LSR R1,R1, #12
	 MOV R4, R1
	 ;STR R1, [R0]  ;store word length
	 ;--stop bits
	 LDR R1, [SP]
	 LDR R0, =0x00030000;
	 AND R1,R1,R0
	 LSR R1,R1,#0x4
	 LDR R0, =USART2_CR2
	 STR R1, [R0] ;store
	 ;--parity enable/disable
	 LDR R0, [SP,#0x4]
	 LDR R1, =0x01;
	 AND R1, R1, R0
	 LSL R1,R1,#0xA
	 ORR R4,R4,R1
	 ;--parity type
	 LDR R0, [SP,#0x4]
	 LDR R1, =0x0100;
	 AND R1, R1, R0
	 LSL R1,R1,#0x1
	 ORR R4,R4,R1
	 ;--Interrupts (en/dis)   interrupt enable
	 EOR R1,R1,R1
	 LDR R3, [SP,#0x04]
	 LDR R0, =0x00FF0000
	 ANDS R3, R0
	 BEQ u_rxtx_l1
	 LDR R1, =(1<<5)|(1<<8) ;RXNE PE
	 ORR R4,R4,R1 ;apply
      
u_rxtx_l1
	 LDR R3, [SP,#0x04]
	 LDR R0, =0xFF000000
     ANDS R3, R0
	 BEQ u_rxtx_l2
	 LDR R1, =(1<<6)|(1<<7)   ;TXE, TC
u_rxtx_l2
     LDR R0, =USART2_CR1
     STR R4, [R0] ;store valuse to reg	
	 ;--NVIC set
	 LDR R0, =0xE000E104  ;NVIC_ISER1
	 LDR R1, [R0] ;load current
	 LDR R2, =(1<<6)  ;bit6 
	 ORR R1, R1, R2  ;apply new
	 STR R1, [R0]  ;load new
	 ;--enable receiver and transmitter
	 LDR R0, =USART2_CR1
	 LDR R1, [R0]
	 LDR R2, =(1<<13)|(1<<2)|(1<<13)|(1<<3)  ;UE, TE  ;UE, RE
	 ORR R1, R1, R2
	 STR R1, [R0]
	 ;restore SP
	 ADD SP,SP,#0x08
	BX LR
	LTORG
	ENDP
;=======USART2_TRANSMITTER_DRIVEN_DMA==============
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  
;--par2@[-|-|-|interrupts_enable]
;--par3@[pointerToDmaBuffer]   
;--par4@[null(4)|DMA_TC_interrupt(4)|priority_channel(8)|buffer_size(16)]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;interrupts_enable: 0xFFFF -> enable TXE TC interrupts, 0->interrupts disable
;***************************************************************
;NOTES: 1) After a full DMA transaction content of the DMA1_CNDTR7 reister reaches zero
;It MUST be restored before start the next DMA transaction
;     2)To start DMA TX transaction - set bit 0 in DAM_CCR7 channel    
  EXPORT uart_init_tx_dma 
uart_init_tx_dma PROC
	 ;turn on  AHB to clock UART2
	 LDR R0, =RCC_APB1ENR
	 LDR R1, [R0]
	 LDR R2, =(1<<17) ;enable clock USART2	 
	 ORR R1, R1, R2
	 STR R1, [R0]
	
	 ;BaudRate
     EOR R0, R0, R0
	 EOR R4,R4,R4
	 LDR R1, [SP]
	 ;extract b0-b15 (BitRate)
	 LDR R2, =0x0000FFFF;
	 AND R1,R1, R2
	 LDR R0, =USART2_BRR
	 ;store result to UART_BRR
	 STR R1, [R0]
	 ;--Data length
	 LDR R1, [SP]
	 LDR R2, =0x01000000;
	 AND R1,R1,R2;
	 LSR R1,R1, #12
     MOV R4, R1
	 ;--stop bits
	 LDR R1, [SP]
	 LDR R0, =0x00030000;
	 AND R1,R1,R0
	 LSR R1,R1,#0x4
	 LDR R0, =USART2_CR2
	 STR R1, [R0] ;store
	 ;--Interrupts ;TXE, TC interrupt enable
	 LDR R1, =(1<<6)|(1<<7)  
	 LDR R0, =USART2_CR1
	 ;enable/disable intrrupts
	 LDR R3, [SP,#0x4];
	 ANDS R3, R1
	 BEQ u_tx_l2
	 ORR R4,R4,R1 ;apply new
     STR R4, [R0]	 
u_tx_l2
	 ;--NVIC set
	 LDR R0, =0xE000E104  ;NVIC_ISER1
	 LDR R1, [R0] ;load current
	 LDR R2, =(1<<7)  ;bit6 (uart2) bit7 (dma1 ch7)(1<<6)|
	 ORR R1, R1, R2  ;apply new
	 STR R1, [R0]  ;load new
	 ;--enable transmitter
	 LDR R0, =USART2_CR1
	 LDR R1, [R0]
	 LDR R2, =(1<<13)|(1<<3)  ;UE, TE
	 ORR R1, R1, R2
	 STR R1, [R0]
	 ;;;dma---
	  ;	enable clock DMA (ch.7 , usart2_tx)
	 LDR R0, =RCC_AHBENR
	 LDR R1, [R0]
	 LDR R2, =0x00000001
	 ORR R1, R2
	 STR R1, [R0]
	 ;-1)Set the peripheral register address
	 LDR R0, =DMA1_CPAR7
	 LDR R1, =USART2_DR
	 STR R1,[R0]
	 ;--2)assign buffer address
	 LDR R1, [SP,#0x08]
	 LDR R0, =DMA1_CMAR7
	 STR R1, [R0]
	 ;--3) buffer size
	 ;NOTE: after each USART event, this value be decremented
	 ;and must be restored manually before next transaction
	 LDR R1, [SP,#0xC]
	 LDR R0, =0x0000FFFF
	 AND R1, R0
	 LDR R0,=DMA1_CNDTR7
	 STR R1,[R0]
	 ;--4)priority of channel
	 LDR R1, [SP,#0xC]
	 LDR R0,=0x00030000;
	 AND R1,R0
	 LSR R1, #0x4
	 LDR R0, =DMA1_CCR7
	 STR R1,[R0]
	 ;--5)Configure data transfer direction, circular mode,
	 ;peripheral & memory incremented 
	 ;mode, peripheral & memory data size,
	 LDR R4, [R0]
	 LDR R1, =(1<<7)|(1<<4) ; memory increment, mem-to-peripherial
	 ORR R4, R1
	 ;transmission complete interrupt enable/disable
	 LDR R1, [SP,#0xC]
	 LDR R2,=0x0F000000;
	 ANDS R1,R2
	 BEQ u2_dma_no_tc
	 LDR R1,=(1<<1) ;TCIE
	 ORR R4,R1
u2_dma_no_tc
     STR R4,[R0]
	 ;7777777777777777777777
	 ;;;dma end
	 ;free memory
	 ADD SP,SP,#16
	 ;POP {R0,R1}
	 ;POP {R0,R1}
	BX LR
	LTORG
	ENDP

;======function clock config
;first@32[ b31 PLL_mult(8) | HSE_psc(8) |AHB_psc(8) |MCO_EN(8) ] ,
;second@32[b31 APB1_psc(8) |APB2_psc(8) |ADC_psc(8) | USB_psc(8) ]
;exaple passing parameters -> PUSH {first, second}
;NOTE: dividers passed in CFGR register as is
;plese see the datasheet
;SP + 0 after return
;NOTE: Fsys=Fcrystal *(2+PLL_mult), please see datasheet
;paassparameters example: PUSH {first,second}
  EXPORT clkConfig
clkConfig   PROC
	;load params 
    ;A)---Turn on the HSE (crystal oscillator:
	LDR R1, =(1 << 16)  ;HSE_ON bit
	LDR R0, =RCC_CR
	STR R1, [R0]; save in reg
    ;B)---Waiting until HSE has been ready:
	LDR R3, =(1 << 17) ;HSE_RDY - bit template
clk_hse_rdy
	LDR R1, [R0]
	ANDS R1, R1, R3 
	BEQ  clk_hse_rdy
	;extract divider)
	LDR R3, [SP]
	LDR R2, =0xFF000000;
	AND R3,R3, R2
	LSR R3, R3, #0x6
	MOV R4, R3  ;store in R4
	;extract HSE psc
	LDR R3, [SP]
	LDR R2, =0x00010000;
	AND R3, R3, R2 
	LSL R3, R3, #0x1
	ORR R4,R4,R3
	;extract MCO bit
	LDR R3, [SP]
	LDR R2, =0x00000001;
	AND R3, R3, R2 
	LSL R3, R3, #0x1A 
	ORR R4,R4,R3
	;HSE - input for sysclock
	LDR R3, =(1<<16)
	ORR R4,R4,R3
    ;C)----Clock Source for the PLL and HSE divider:
	LDR R0, =RCC_CFGR
	STR R4, [R0];
	;D) Enable PLL
	LDR R3, =(1<<24) ; PLL_ON
	LDR R0, =RCC_CR
	LDR R1, [R0]
	ORR R1, R1, R3
	STR R1, [R0]
	;E) waiting until PLL stabilized:
	LDR R3, =(1<<25) ;PLL_RDY
clk_pll_rdy
    LDR R1 , [R0]  ;load RCC_CR
    ANDS R1, R1, R3
    BEQ  clk_pll_rdy
	;F)set-up AHB, APB1, APB2 bus dividers
	;clear  var in RAM firstly:
	EOR R4, R4, R4  ;clear 
     ;1) AHB
  	LDR R3, [SP]  ;load parameter from stack
	LDR R2, =0x00000F00  ;template for the parameter
	AND R3, R3, R2  ;save only intrersted bits (AHB)
	LSR R3, R3, #0x4  ;shift to acheive order bits as in RCC_CFGR
	ORR R4,R4,R3 
	  ;2)APB1
  	LDR R3, [SP,#0x04]
	LDR R2, =0x07000000
	AND R3, R3, R2
	LSR R3, R3, #0x10
    ORR R4,R4,R3 ;add prev to current
   
      ;3)APB2
    LDR R3, [SP,#0x4]
    LDR R2, =0x00070000;
    AND R3, R3, R2
    LSR R3, R3, #0x5
    ORR R4,R4,R3
       ;4) ADC
    LDR R3, [SP,#0x4]
    LDR R2, =0x00000300;
    AND R3, R3, R2
    LSL R3, R3, #0x6
    ORR	R4,R4,R3
	   ;5) USB
    LDR R3, [SP,#0x4]
    LDR R2, =0x00000001;
    AND R3, R3, R2	
	LSL R3, R3, #0x17
	ORR R4,R4,R3
	   ;6) apply content to the CFGR
	LDR R0 , =RCC_CFGR
	LDR R3, [R0]
	ORR R4, R4, R3
	STR R4, [R0]
    ;G) SEt PLL as clock source for system:
    LDR R0, =RCC_CFGR
    LDR R1, [R0]
    LDR R3, =(1<<1) ; SW bits:  PLL as system clock
	ORR R1, R1, R3
	STR R1, [R0]
	;H)wait until RCC has been swiched to PLL:
    LDR R1, [R0]
    LDR R3, =(1<<3) ; SW bits:  PLL as system clock
clk_rcc_bus_rdy
    LDR R1, [R0]
	ANDS R1, R1, R3
	BEQ clk_rcc_bus_rdy
    ;free memory
	ADD SP,SP,#0x8
	BX LR
	LTORG
	ENDP

    END
