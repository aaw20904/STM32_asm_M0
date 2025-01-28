    INCLUDE stm32f103c8.inc
	THUMB
    AREA    MyLib, CODE, READONLY

  EXPORT nvicEnableIRQ
nvicEnableIRQ  PROC
  ;params:
  ;one@[-|-|priority|IRQ_Number]
  ;stack_frame
  ; 1B  18 17  14 13       10  F     C B     8 7     4 3    0
  ; [par_1|offset|bitPosition|quotient|product|var2|var1]  
  SUB SP, SP, #24 ;  4*6= 24bytes allocate memory from stack
nvicEI_par1 EQU 0x18
nvicEI_offset EQU 0x14
nvicEI_bitPosition EQU 0x10
nvicEI_quotient EQU 0xC
nvicEI_product EQU 0x08
nvicEI_remainder EQU 0x04
nvicEI_var1 EQU 0x00	  
  ;A) offset address
   LDR R1, [SP,#nvicEI_par1]
   AND R1, R1, #0x000000FF ;extract IRQ number
   LSR R1, R1, #0x5  ; x = x / 32
   LSL R1 , R1 , #0x2 ; additional address fo r offset x = x * 4
   STR R1, [SP,#nvicEI_offset] 
   ;B)Bit position
   LDR R1, [SP,#nvicEI_par1]
   AND R1, R1, #0x000000FF ;extract IRQ number
   LSR R2, R1, #0x5
   STR R2, [SP,#nvicEI_quotient] ;store quotient
   LSL R2, #0x5  ; x = x* 32
   STR R2, [SP, #nvicEI_product] ; store product
   LDR R1, [SP,#nvicEI_par1]
   AND R1, #0x000000FF
   SUB R1, R1, R2
   STR R1, [SP,nvicEI_bitPosition]
   ;load base register  address
   LDR R0, =NVIC_ISER0
   ;add offset
   LDR R1, [SP,#nvicEI_offset]
   ADD R0, R1
   ;load bit position
   LDR R2, [SP,#nvicEI_bitPosition]
   LDR R3, =0x01
   LSL R3, R2  ;shift 1 left on specifified position
   ;update intrrrupt enable reg
   LDR R2, [R0]
   ORR R2, R3  ;apply bit
   STR R2, [R0]
   ;Part 2. Priority level set
	  ;A) offset address
	   LDR R1, [SP,#nvicEI_par1]
	   AND R1, R1, #0x000000FF ;extract IRQ priority
	   ;LSR R1, R1, #0x8 ;normalize
	   LSR R1, R1, #0x2  ; x = x / 4
	   LSL R1 , R1 , #0x2 ; additional address fo r offset x = x * 4
	   STR R1, [SP,#nvicEI_offset] 
	   ;B)Bit field position
	   LDR R1, [SP,#nvicEI_par1]
	   AND R1, R1, #0x000000ff;extract IRQ number
	   ;LSR R1, #0x8 ;normalize
	   LSR R2, R1, #0x2 ;x = x/4
	   STR R2, [SP,#nvicEI_quotient] ;store quotient
	   LSL R2, #0x2  ; x = x* 4
	   STR R2, [SP, #nvicEI_product] ; store product
	   LDR R1, [SP,#nvicEI_par1]
	   AND R1, #0x000000fF
	   ;LSR R1, #0x8 ; normalize
	   SUB R1, R1, R2
	   ;times 8 - because there are one byte per IRQ channel
       LSL R1, #0x3	   
	   STR R1, [SP,nvicEI_bitPosition]
	   ;load base register  address
	   LDR R0, =NVIC_IPR0
	   ;add offset
	   LDR R1, [SP,#nvicEI_offset]
	   ADD R0, R1
	   ;load bit position
	   LDR R2, [SP,#nvicEI_bitPosition]
	   ;load priority level
	   LDR R1, [SP,#nvicEI_par1]
	   AND R1, #0x0000FF00 ;extract parameter
	   LSR R1, #0x8 ;normalize
	   LSL R1, R1, R2  ;shift  left on specifified position
	   ;update intrrrupt priority reg
	   LDR R2, [R0]
	   ORR R2, R1  ;apply pri level
	   STR R2, [R0] 
   
  ADD SP, SP, #24 ;  4*6= 24bytes free memory to stack
  POP {R0}
  BX LR
	ENDP
		
		
  EXPORT gpio_init
gpio_init   PROC
    LDR R0, =(1)|(1<<2)|(1<<3)|(1<<4)   ; enable alternate, GPIOC  GPIOA GPIOB
    LDR R1, =RCC_APB2ENR  ; Load the address of RCC_APB2ENR
    STR R0, [R1]          ; Store the value into RCC_APB2ENR
    BX LR                 ; Return from subroutine
	LTORG
            ENDP
	
  ;---dummy-delay-procedure				
	EXPORT delay_
delay_            PROC
	LDR R2, =0xF4240; 1000000 ;counter value
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
	LDR R0, =NVIC_ISER0
	LDR R1, [R0] ; load current content
	ORR R1, R1, #(1 << 29)    ; Enable TIM3 interrupt (position 28 in ISER0)
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
	 ENDP
	LTORG
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
	ENDP
	LTORG
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
    ENDP
	LTORG
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
    ENDP
	LTORG
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
	 LDR R0, =NVIC_ISER1
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
	ENDP
	LTORG
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
	 LDR R0, =NVIC_ISER1
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
	ENDP
	LTORG
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
	 LDR R0, =NVIC_ISER1
	 LDR R1, [R0] ;load current
	 LDR R2, =(1<<6)  ;bit6 USART2 interrupt enable 
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
	ENDP
	LTORG
;=======USART2_TRANSMITTER_DRIVEN_DMA==============
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  
;--par2@[-|-|-|-]
;--par3@[pointerToDmaBuffer]   
;--par4@[null(4)|DMA_TC_interrupt(4)|priority_channel(8)|buffer_size(16)]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;interrupts_enable: 0xFFFF -> enable TXE TC interrupts, 0->interrupts disable
;***************************************************************
;NOTES: 1) After a full DMA transaction content of the DMA1_CNDTR7 reister reaches zero
;It MUST be restored (disable channel->restore CNDTR->enable_channel(start)) before start the next DMA transaction
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
	 ;to correct run DMA engine
	 ;NVIC interrupt for USART2 should be  off
	 LDR R1, =(1<<6)|(1<<7)  
	 LDR R0, =USART2_CR1
	 ORR R4,R4,R1 ;apply new
     STR R4, [R0]	 
	 ;--enable DMA TX
	 LDR R0, =USART2_CR3
	 LDR R1, =(1<<7) ;bit7-DMAT , bit6-DMAR
	 STR R1, [R0]  
	 ;--NVIC stay off
	 
	 ;--enable transmitter
	 LDR R0, =USART2_CR1
	 LDR R1, [R0]
	 LDR R2, =(1<<13)|(1<<3)  ;UE, TE
	 ORR R1, R1, R2
	 STR R1, [R0]
	 ;*****************
	 ;---D M A 1-C H 7---init---
	 ;---enable-clock-DMA-(ch.7 , usart2_tx)
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
     STR R4,[R0]  ; send to the DMA1_CCR7 reg
	 ;enable interrupts for DMA1 CH7 in NVIC
	  LDR R0, =NVIC_ISER0
	  LDR R1, =(1<<17)  ; DMA1 Channel 7 interrupt, bit 17 of ISER0
	  STR R1, [R0]
	 ;;;dma end
	 ;free memory
	 ADD SP,SP,#16
	 ;POP {R0,R1}
	 ;POP {R0,R1}
	BX LR
	ENDP
	LTORG

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
	ENDP
	LTORG
;*********SPI***BEGIN-{
;---F U N C T I O N-----SPI1-Master-Transmitter-only-with-interrupts
   EXPORT spi1MasterTxOnlyIt
spi1MasterTxOnlyIt PROC
;INFO: point "."  a separator of nibbles (4bit fields), '|' byte separator
;parameter1 @ [frame_length.divider|LSBFIRST|SSM|CPOL.CPHA]
;bits:
;(0-3) CPHA , (16-23) LSBFIRST
;(4-7) CPOL , (24-27) Divider 0->clk/2 , 7->clk/256 
;(8-15) SSM , (28-31) frame length
;---------------------------
;parameter2 @ [-|-|-|-]
;---------------------------
   ;preparing: GPIOs  A5, A7
    LDR R0, =GPIOA_CRL
    LDR R2, =0xB0B00000 ;PA7, PA5 alternative push-pull, 50MHz
    LDR R1, [R0] ;load curent CRL
    ORR R1, R2 ;apply
    STR R1, [R0] ;store CRL
   ;1)Enable peripherial
	LDR R0, =RCC_APB2ENR
	LDR R2, [R0] ;load current CR1 value
	LDR R1, =(1<<12) ;bit 12 -enable SPI1 
	ORR R2, R1 ;apply changes
	STR R2, [R0] ;strore updated data
	 ;1A)Enable SPI1 interrupts in NVIC
    LDR R0, =NVIC_ISER1
    LDR R1, [R0] ;load ISER1
    LDR R2, =(1<<3)
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store ISER1
	;1B)Clear registers
	LDR R0, =SPI1_CR1
	LDR R1, =0x0
	STR R1, [R0]
	LDR R0, =SPI1_CR2
	STR R1, [R0]
	;2)Is there software select management?
	LDR R1, [SP]
	LDR R2, =(1<<8) ; bit8 - SSM
	ANDS R1, R2
	BEQ lbHardwareSS
		;when software slave ON
		;a) turn on SSM and SSI bits.  
		LDR R0, =SPI1_CR1
		LDR R1, [R0] ;load current
		LDR R2, =(1<<9)|(1<<8) ;SSM->bit9, SSI->bit8
		ORR R1, R2 ;update
		STR R1, [R0] ;store
		B lb001
lbHardwareSS
    ;when software slave mgm OFF - hardware management
	;a)enable pin GPIOA4 (NSS pin)
	LDR R0, =GPIOA_CRL
	LDR R1, [R0] ; load current value
	LDR R2, =0x000B0000  ;A4 push-pull alternative, 50Mz
	ORR R2, R1
	STR R2, [R0]
	;b)enable SS output
	LDR R0, =SPI1_CR2
	LDR R1, [R0]
	LDR R2, =(1<<2) ;SSOE bit 2
	ORR R1, R2  ;apply new value 
	STR R1, [R0] ;  store
lb001
   ;3)Set up divider
    LDR R0, =SPI1_CR1 
    LDR R1, [SP]
	LDR R2, =0x07000000 ; mask for divider parameter 
    AND R1, R2 ;extract divider
	LSR R1, R1, #21  ;shift in b3-b5 (as in SPI_CR1)
	LDR R2, [R0] ;load current CR1
	ORR R1, R2 
	STR R1, [R0]  ;udate CR1
	;4)SetUp phase and polarity  ERROR: phase and polarity not recognized properly
    LDR R1, [SP]
    LDR R2, =0x00000011
    AND R2, R1
    MOV R3, R2 ;copy 
    LSR R2, R2, #3 ;shift CPOL in according to the CR1
	AND R3, #1
    ORR R2, R3  ; apply CPHA bit
    LDR R1, [R0] ;load current
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store
  ;5)Frmae length (DFF  bit11)
    LDR R0, =SPI1_CR1
    LDR R1, [SP]
    LDR R2, =0x10000000
    AND R1, R2 ;filter DFF
    LSR R1, R1, #17 ;shit the bit to b11 as in CR1
    LDR R2, [R0] ;load CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store changes
 ;6)LSB/MSBIFRST
    LDR R1, [SP]
    LDR R2, =0x00010000
    AND R1, R2 ;extract LSBFIRST bit
    LSR R1, R1, #9 ;sift to bit7 
    LDR R2, [R0] ;load current
    ORR R1, R2 ;apply
    STR R1, [R0] ;save CR1
 ;7) Tx Only mode
   ;BIDIMODE b15=1 (bidirectional MOSI line)
   ;BIDIOE b14=1 (transmitter only mode)
   ;RXONLY b10=0 
    LDR R1, =(1<<15)|(1<<14)
    LDR R2, [R0] ;load current CR1
    ORR R1, R2 ;apply new data
    STR R1, [R0] ;save
  ;8) TXE interrut enable
    LDR R0, =SPI1_CR2
    LDR R2, =(1<<7)
    LDR R1, [R0] ;address of CR2
    ORR R1, R2 ;apply new data
    STR R1, [R0] ;save to CR2
  ;9)Master mode (b2) and enable SPI (b6)
    LDR R0, =SPI1_CR1
    LDR R2, =(1<<6)|(1<<2)
    LDR R1, [R0] ;load current CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;save CR1
 
   ;free stack from function parameters
	ADD SP,SP,#8
	BX LR
	 LTORG
	ENDP
   
	
;----FUNCTION-----SPI1-slave-Rx-only-with-interrupt
  EXPORT spi1SlaveRxOnlyIt
spi1SlaveRxOnlyIt PROC
;INFO: point "."  a separator of nibbles (4bit fields), '|' byte separator
;parameter1 @ [frame_length. -|LSBFIRST|SSM|CPOL.CPHA]
;bits:
;(0-3) CPHA , (16-23) LSBFIRST
;(4-7) CPOL , (24-27) NOT USED (because slave mode and extern. clock) 
;(8-15) SSM , (28-31) frame length
;---------------------------
;parameter2 @ [-|-|-|-]
;---------------------------
;Because it is a slave mode - all pins 
;(MOSI, SCK, maybe NSS when enable) acts as inputs
   ;1)Enable peripherial
	LDR R0, =RCC_APB2ENR
	LDR R2, [R0] ;load current CR1 value
	LDR R1, =(1<<12) ;bit 12 -enable SPI1 
	ORR R2, R1 ;apply changes
	STR R2, [R0] ;strore updated data
	   ;9)Enable interrupts from SPI in NVIC
    LDR R0, =NVIC_ISER1
    LDR R1, [R0] ;load ISER1
    LDR R2, =(1<<3)
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store ISER1 
	;1A)Clear registers
	LDR R0, =SPI1_CR1
	LDR R1, =0x0
	STR R1, [R0]
	LDR R0, =SPI1_CR2
	STR R1, [R0]
  ;2)Frmae length (DFF  bit11)
    LDR R0, =SPI1_CR1
    LDR R1, [SP]
    LDR R2, =0x10000000
    AND R1, R2 ;filter DFF
    LSR R1, R1, #17 ;shit the bit to b11 as in CR1
    LDR R2, [R0] ;load CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store changes	    
  ;3)SetUp phase and polarity
    LDR R1, [SP]
    LDR R2, =0x00000011
    AND R2, R1
    MOV R3, R2 ;copy 
    LSR R2, R2, #3 ;shift CPOL in according to the CR1
	AND R3, #1
    ORR R2, R3  ; apply CPHA bit
    LDR R1, [R0] ;load current
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store
   ;4)LSB/MSBIFRST
    LDR R1, [SP]
    LDR R2, =0x00010000
    AND R1, R2 ;extract LSBFIRST bit
    LSR R1, R1, #9 ;sift to bit7 
    LDR R2, [R0] ;load current
    ORR R1, R2 ;apply
    STR R1, [R0] ;save CR1
   ;5)Is there software select management?
	LDR R1, [SP]
	LDR R2, =(1<<8) ; bit8 - SSM
	ANDS R1, R2
	BEQ lbHardwareSS_3
		;when software slave management
		;GPIOA5,7 init  0x4
		LDR R0, =GPIOA_CRL
		LDR R1, [R0]
		LDR R2, =0x40400000 ;A5, A7 - digital input
		ORR R1, R2 ;read current
		STR R1, [R0] ;update
		;a) turn on SSM and SSI bits.  
		LDR R0, =SPI1_CR1
		LDR R1, [R0] ;load current
		LDR R2, =(1<<9);SSM->bit9, SSI->bit8
		ORR R1, R2 ;update
		STR R1, [R0] ;store	
		B L0013
lbHardwareSS_3
        ;GPIOA4,5,7
        LDR R0, =SPI1_CR1
		LDR R1, [R0]
		LDR R2, =0x40440000 ;A4,A5, A7 - digital input
		ORR R1, R2 ;read current
		STR R1, [R0] ;update
L0013		LDR R0, =SPI1_CR1
  ;6)Setting Rx only mode
   ;BIDIMODE b15=1 (bidirectional MOSI line)
   ;BIDIOE b14=0  
   ;RXONLY b10=1 (Rx only) 
    LDR R1, =(1<<10)  ;(1<<15)|
    LDR R2, [R0] ;load current CR1
    ORR R1, R2 ;apply new data
    STR R1, [R0] ;save  
  ;7)RXNE interrupt enble
    LDR R0, =SPI1_CR2
    LDR R2, =(1<<6)
    LDR R1, [R0] ;address of CR2
    ORR R1, R2 ;apply new data
    STR R1, [R0] ;save to CR2
   ;8)Slave mode (b2=0) and enable SPI (b6=1)
    LDR R0, =SPI1_CR1
    LDR R2, =(1<<6)
    LDR R1, [R0] ;load current CR1
    ORR R1, R2 ;apply new	
	STR R1, [R0]
 
    ;free memory
	ADD SP,SP,#0x8
	BX LR
	ENDP 
	LTORG	
;---HAD NOT BEEN DEBUGGED!--------------	
;---FUNCTION-----SPI1-Master-Full--Duplex-with-interrupts
;A4-NSS, A5-SCK, A6-MISO, A7-MOSI
   EXPORT spi1MasterFullDuplexIt
spi1MasterFullDuplexIt PROC
;INFO: point "."  a separator of nibbles (4bit fields), '|' byte separator
;parameter1 @ [frame_length.divider|LSBFIRST|SSM|CPOL.CPHA]
;bits:
;(0-3) CPHA , (16-23) LSBFIRST
;(4-7) CPOL , (24-27) Divider 0->clk/2 , 7->clk/256 
;(8-15) SSM , (28-31) frame length
;---------------------------
;parameter2 @ [-|-|-|-]
;---------------------------
   ;preparing: GPIOs  A5, A7
    LDR R0, =GPIOA_CRL
    LDR R2, =0xB0B00000 ;PA7, PA5 alternative push-pull, 50MHz
    LDR R1, [R0] ;load curent CRL
    ORR R1, R2 ;apply
    STR R1, [R0] ;store CRL
   ;1)Enable peripherial
	LDR R0, =RCC_APB2ENR
	LDR R2, [R0] ;load current CR1 value
	LDR R1, =(1<<12) ;bit 12 -enable SPI1 
	ORR R2, R1 ;apply changes
	STR R2, [R0] ;strore updated data
	;1A)Clear registers
	LDR R0, =SPI1_CR1
	LDR R1, =0x0
	STR R1, [R0]
	LDR R0, =SPI1_CR2
	STR R1, [R0]
	;2)Is there software select management?
	LDR R1, [SP]
	LDR R2, =(1<<8) ; bit8 - SSM
	ANDS R1, R2
	STR R1, [R0]
	BEQ lbHardwareSS_12
		;when software slave ON
		;a) turn on SSM and SSI bits.  
		LDR R0, =SPI1_CR1
		LDR R1, [R0] ;load current
		LDR R2, =(1<<9)|(1<<8) ;SSM->bit9, SSI->bit8
		ORR R1, R2 ;update
		STR R1, [R0] ;store
		B 21
lbHardwareSS_12
    ;when software slave mgm OFF - hardware management
	;a)enable pin GPIOA4 (NSS pin)
	LDR R0, =GPIOA_CRL
	LDR R1, [R0] ; load current value
	LDR R2, =0x000B0000  ;A4 push-pull alternative, 50Mz
	ORR R2, R1
	STR R2, [R0]
	;b)enable SS output
	LDR R0, =SPI1_CR2
	LDR R1, [R0]
	LDR R2, =(1<<2) ;SSOE bit 2
	ORR R1, R2  ;apply new value 
	STR R1, [R0] ;  store
21
   ;3)Set up divider
    LDR R0, =SPI1_CR1 
    LDR R1, [SP]
	LDR R2, =0x07000000 ; mask for divider parameter 
    AND R1, R2 ;extract divider
	LSR R1, R1, #21  ;shift in b3-b5 (as in SPI_CR1)
	LDR R2, [R0] ;load current CR1
	ORR R1, R2 
	STR R1, [R0]  ;udate CR1
  ;4)SetUp phase and polarity
     LDR R1, [SP]
    LDR R2, =0x00000011
    AND R2, R1
    MOV R3, R2 ;copy 
    LSR R2, R2, #3 ;shift CPOL in according to the CR1
	AND R3, #1
    ORR R2, R3  ; apply CPHA bit
    LDR R1, [R0] ;load current
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store
  ;5)Frmae length (DFF  bit11)
    LDR R0, =SPI1_CR1
    LDR R1, [SP]
    LDR R2, =0x10000000
    AND R1, R2 ;filter DFF
    LSR R1, R1, #17 ;shit the bit to b11 as in CR1
    LDR R2, [R0] ;load CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store changes
 ;6)LSB/MSBIFRST
    LDR R1, [SP]
    LDR R2, =0x00010000
    AND R1, R2 ;extract LSBFIRST bit
    LSR R1, R1, #9 ;sift to bit7 
    LDR R2, [R0] ;load current
    ORR R1, R2 ;apply
    STR R1, [R0] ;save CR1
 ;7) Full Duplex mode
   ;BIDIMODE b15=0 (2-line unidirectional data mode)
   ;BIDIOE b14=0 
   ;RXONLY b10=0 
    LDR R1, =(0<<15)|(0<<14)
    LDR R2, [R0] ;load current CR1
    ORR R1, R2 ;apply new data
    STR R1, [R0] ;save
  ;8) TXE and RXNE interrut enable
    LDR R0, =SPI1_CR2
    LDR R2, =(1<<7)|(1<<6)
    LDR R1, [R0] ;address of CR2
    ORR R1, R2 ;apply new data
    STR R1, [R0] ;save to CR2
  ;9)Master mode (b2) and enable SPI (b6)
    LDR R0, =SPI1_CR1
    LDR R2, =(1<<6)|(1<<2)
    LDR R1, [R0] ;load current CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;save CR1
  ;10)Enable SPI1 interrupts in NVIC
    LDR R0, =NVIC_ISER1
    LDR R1, [R0] ;load ISER1
    LDR R2, =(1<<3)
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store ISER1
   ;free stack from function parameters
	ADD SP,SP,#8
	BX LR
	LTORG
	ENDP
;--F U N C T I O N-----SPI1-Master-Transmitter-with-DMA-
   EXPORT spi1MasterOnlyTransmitterDMAIt
spi1MasterOnlyTransmitterDMAIt  PROC 
 ;NECCESSERY NOTE for PROGRAMMERS: to S T A R T  DMA transmission- 
	;1)When run second time or later - Set DMA1_CNDTR3 amount of words in transaction
	     ;(it cleared at the end of each transaction)
		 ;When it runs for the first time - in shoul be setted later  
	;2)Turn ON DMA1_CCR3 [bit 0]. Transaction is starting NOW.
;***************************************************************************	
;INFO: point "."  a separator of nibbles (4bit fields), '|' byte separator
;parameter1 @ [frame_length.divider|LSBFIRST|SSM|CPOL.CPHA]
;----------------------------
;bits:
;(0-3) CPHA , (16-23) LSBFIRST
;(4-7) CPOL , (24-27) Divider 0->clk/2 , 7->clk/256 
;(8-15) SSM , (28-31) frame length
;---------------------------
;parameter2 @ [addressOfBuffer]
;---------------------------
;parameter3 @ [-|circularMode|wordsInBuffer]
;wordsInBuffer - amount of 8bit or 16bit transactions to transmitt
;circularMode - enable circular mode
;------------------------------
   ;preparing: GPIOs  A5, A7
    LDR R0, =GPIOA_CRL
    LDR R2, =0xB0B00000 ;PA7, PA5 alternative push-pull, 50MHz
    LDR R1, [R0] ;load curent CRL
    ORR R1, R2 ;apply
    STR R1, [R0] ;store CRL
	;enable  c) DMA interrupts
	LDR R0, =NVIC_ISER0
	LDR R1, [R0]
	LDR R2, =(1 << 13)   ; Enable interrupt for DMA1 Channel3 (IRQ12 corresponds to DMA1 Channel3)
	ORR R1, R2
	STR R1, [R0]
	
	;1)Enable peripherial a) SPI:
	LDR R0, =RCC_APB2ENR
	LDR R2, [R0] ;load current CR1 value
	LDR R1, =(1<<12) ;bit 12 -enable SPI1  
	ORR R2, R1 ;apply changes
	STR R2, [R0] ;strore updated data
	;enable peripherial b) DMA1:
	LDR R0, =RCC_AHBENR
	LDR R2, =0x1 ; bit0 DMA1EN
	LDR R1, [R0] ;load current 
	ORR R1, R2 ;modify
	STR R1, [R0] ;update AHBENR
	
	;1A)Clear registers
	LDR R0, =SPI1_CR1
	LDR R1, =0x0
	STR R1, [R0]
	LDR R0, =SPI1_CR2
	STR R1, [R0]
	;2)Is there software select management?
	LDR R1, [SP]
	LDR  R2, =(1<<8) ; bit8 - SSM
	ANDS R1, R2
	BEQ lbHardwareSS_4
		;when software slave ON
		;a) turn on SSM and SSI bits.  
		LDR R0, =SPI1_CR1
		LDR R1, [R0] ;load current
		LDR R2, =(1<<9)|(1<<8) ;SSM->bit9, SSI->bit8
		ORR R1, R2 ;update
		STR R1, [R0] ;store
		B L31
lbHardwareSS_4
    ;when software slave mgm OFF - hardware management
	;a)enable pin GPIOA4 (NSS pin)
	LDR R0, =GPIOA_CRL
	LDR R1, [R0] ; load current value
	LDR R2, =0x000B0000  ;A4 push-pull alternative, 50Mz
	ORR R2, R1
	STR R2, [R0]
	;b)enable SS output
	LDR R0, =SPI1_CR2
	LDR R1, [R0]
	LDR R2, =(1<<2) ;SSOE bit 2
	ORR R1, R2  ;apply new value 
	STR R1, [R0] ;  store
L31
   ;3)Set up divider
    LDR R0, =SPI1_CR1 
    LDR R1, [SP]
	LDR R2, =0x07000000 ; mask for divider parameter 
    AND R1, R2 ;extract divider
	LSR R1, R1, #21  ;shift in b3-b5 (as in SPI_CR1)
	LDR R2, [R0] ;load current CR1
	ORR R1, R2 
	STR R1, [R0]  ;udate CR1
  ;4)SetUp phase and polarity
    LDR R1, [SP]
    LDR R2, =0x00000011
    AND R2, R1
    MOV R3, R2 ;copy 
    LSR R2, R2, #3 ;shift CPOL in according to the CR1
	AND R3, #1
    ORR R2, R3  ; apply CPHA bit
    LDR R1, [R0] ;load current
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store
  ;5)Frmae length (DFF  bit11)
    LDR R0, =SPI1_CR1
    LDR R1, [SP]
    LDR R2, =0x10000000
    AND R1, R2 ;filter DFF
    LSR R1, R1, #17 ;shit the bit to b11 as in CR1
    LDR R2, [R0] ;load CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store changes
 ;6)LSB/MSBIFRST
    LDR R1, [SP]
    LDR R2, =0x00010000
    AND R1, R2 ;extract LSBFIRST bit
    LSR R1, R1, #9 ;sift to bit7 
    LDR R2, [R0] ;load current
    ORR R1, R2 ;apply
    STR R1, [R0] ;save CR1
 ;7) Tx Only mode
   ;BIDIMODE b15=1 (bidirectional MOSI line)
   ;BIDIOE b14=1 (transmitter only mode)
   ;RXONLY b10=0 
    LDR R1, =(1<<15)|(1<<14)
    LDR R2, [R0] ;load current CR1
    ORR R1, R2 ;apply new data
    STR R1, [R0] ;save

  ;8)Enable DMA channel processing on Tx
    LDR R0, =SPI1_CR2
	LDR R1, [R0]
	LDR R2, =(1<<1)
	ORR R1, R2
	STR R1, [R0]
  ;9)Master mode (b2=1) and enable SPI (b6=1)
    LDR R0, =SPI1_CR1
    LDR R2, =(1<<2)|(1<<6)
    LDR R1, [R0] ;load current CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;save CR1    
  ;10)Initializing DMA1_CH3 for Tx
   ;a)disable DMA1 channel 3 (clear register)
    LDR R0, =DMA1_CCR3
	LDR R2, =0x00000000
	STR R2, [R0]
   ;b) peripherial address
	LDR R0, =DMA1_CPAR3
	LDR R1, =SPI1_DR
	STR R1, [R0]
   ;c)memory address
	LDR R0, =DMA1_CMAR3
	LDR R1, [SP,#4]
	STR R1, [R0]
   ;d)number_of_words_to_transmitt
	LDR R0, =DMA1_CNDTR3
	LDR R1, [SP,#8]
	LDR R2, =0x0000FFFF
	AND R1, R2 ;extract b0-b16
	STR R1, [R0] ; save
   ;e) Is there 8 bits or 16 bits?
	LDR R1, [SP]
	LDR R2, =0x10000000
	ANDS R1, R2
	BEQ oneByteDataX
	  ;16 bit word 
	  LDR R0, =DMA1_CCR3
	  LDR R1, [R0]
	  LDR R2, =0x00000500 ;psize and msize = 2 bytes
	  ORR R1, R2
	  STR R1, [R0]	
oneByteDataX
       ;8 bit word - default value (b11-b8), so do nothing
   ;f)memory increment b7, memory-to-peripherial b4
    LDR R1, [R0]
    LDR R2, =(1<<7)|(1<<4)
    ORR R1,R2
   ;g)interrupt on half (b2) and full (b1) transfer complete 
    LDR R2, =(1<<1) ;full transfer IT
	ORR R1, R2
   ;h)Circular mode 
    LDR R2, [SP, #8]
	LDR R3, =0x00010000  ;extract bit mask
	AND R2, R3 
	LSR R2, #11 ;move to b5 in acc to CCR
	ORR R1, R2 ;update
	STR R1, [R0] ;save
   
    ;free memory
	ADD SP,SP,#0x12
	BX LR
	ENDP 
	LTORG	
	
	
	;***TEST**PASSED*****
	;--F U N C T I O N-----SPI1-slave-Rx-with DMA
   EXPORT spi1SlaveOnlyReceiverDMAIt
spi1SlaveOnlyReceiverDMAIt  PROC 
;--To start : 2)Clear SSI (bit8) SPI1_CR1
;             1)Enable DMA1 channel 2 (Bit0) DMA1_CCR2
;             3)When mode NOT circular - update DMA1_CNDTR2 register after end of DMA transaction 
;***************************************************************************	
;INFO: point "."  a separator of nibbles (4bit fields), '|' byte separator
;parameter1 @ [frame_length.-|LSBFIRST|SSM|CPOL.CPHA]
;----------------------------
;bits:
;(0-3) CPHA , (16-23) LSBFIRST
;(4-7) CPOL , (24-27) - 
;(8-15) SSM , (28-31) frame length
;---------------------------
;parameter2 @ [addressOfBuffer]
;---------------------------
;parameter3 @ [-|circularMode|wordsInBuffer]
;wordsInBuffer - amount of 8bit or 16bit transactions to transmitt
;circularMode - enable circular mode
;------------------------------
	;D-1)enable peripherial b) DMA1:
	LDR R0, =RCC_AHBENR
	LDR R2, =0x1 ; bit0 DMA1EN
	LDR R1, [R0] ;load current 
	ORR R1, R2 ;modify
	STR R1, [R0] ;update AHBENR
	;D-2) enable    DMA interrupts (IRQ12)
	  ;-when an inner procedure has been calling - save LR in the stack before
	  ;and restore it after inner procedure
	  PUSH {LR}
	   ;one@[-|-|priority|IRQ_Number]
	  LDR R0, =0x0000000C
	  PUSH {R0} ;store LR for inner call
	  BL nvicEnableIRQ
	  POP {LR} ;restore LR
	;D-3)Enable peripherial Clock SPI:
	LDR R0, =RCC_APB2ENR
	LDR R2, [R0] ;load current CR1 value
	LDR R1, =(1<<12) ;bit 12 -enable SPI1  
	ORR R2, R1 ;apply changes
	STR R2, [R0] ;strore updated data
    ;D-4) GPIOA clock enable ????
	;D-5)GPIOA  init
	LDR R0, =GPIOA_CRL
    LDR R2, =0x40400000 ;PA7, PA5 digital input
    LDR R1, [R0] ;load curent CRL
    ORR R1, R2 ;apply
    STR R1, [R0] ;store CRL
	;D-6) DMA init
	;a)disable DMA1 channel 2 (clear register)
    LDR R0, =DMA1_CCR2
	LDR R2, =0x00000000
	STR R2, [R0]
   ;b) peripherial address
	LDR R0, =DMA1_CPAR2
	LDR R1, =SPI1_DR
	STR R1, [R0]
   ;c)memory address
	LDR R0, =DMA1_CMAR2
	LDR R1, [SP,#4]
	STR R1, [R0]
   ;d)number_of_words_to_transmitt
	LDR R0, =DMA1_CNDTR2
	LDR R1, [SP,#8]
	LDR R2, =0x0000FFFF
	AND R1, R2 ;extract b0-b16
	STR R1, [R0] ; save
   ;e) Is there 8 bits or 16 bits?
	LDR R1, [SP]
	LDR R2, =0x10000000
	ANDS R1, R2
	BEQ oneByteDataX1
	  ;16 bit word 
	  LDR R0, =DMA1_CCR2
	  LDR R1, [R0]
	  LDR R2, =0x00000500 ;psize and msize = 2 bytes
	  ORR R1, R2
	  STR R1, [R0]	
oneByteDataX1
       ;8 bit word - default value (b11-b8), so do nothing
   ;f)memory increment b7, memory-to-peripherial b4
    LDR R1, [R0]
    LDR R2, =(1<<7) ;|(1<<4)
    ORR R1,R2
   ;g)interrupt on half (b2) and full (b1) transfer complete 
    LDR R2, =(1<<1)|(1<<2) ;full and half transfer IT
	ORR R1, R2
   ;h)Circular mode 
    LDR R2, [SP, #8]
	LDR R3, =0x00010000  ;extract bit mask
	AND R2, R3 
	LSR R2, #11 ;move to b5 in acc to CCR
	ORR R1, R2 ;update
	STR R1, [R0] ;save
	;D-7)SPI init
	
	;1A)Clear registers
	LDR R0, =SPI1_CR1
	LDR R1, =0x0
	STR R1, [R0]
	LDR R0, =SPI1_CR2
	STR R1, [R0]
	;2)Is there software select management?
	LDR R1, [SP]
	LDR  R2, =(1<<8) ; bit8 - SSM
	ANDS R1, R2
	BEQ lbHardwareSS1_4
		;when software slave ON
		;a) turn on SSM and SSI bits.  
		LDR R0, =SPI1_CR1
		LDR R1, [R0] ;load current
		LDR R2, =(1<<9)|(1<<8) ;SSM->bit9, SSI->bit8
		ORR R1, R2 ;update
		STR R1, [R0] ;store
		B L311
lbHardwareSS1_4
    ;when software slave mgm OFF - hardware management
	;a)enable pin GPIOA4 (NSS pin)
	LDR R0, =GPIOA_CRL
	LDR R1, [R0]
	LDR R2, =0x40440000 ;A4,A5, A7 - digital input
	ORR R1, R2
	STR R1, [R0]
	;b)enable SS output
	LDR R0, =SPI1_CR2
	LDR R1, [R0]
	LDR R2, =(1<<2) ;SSOE bit 2
	ORR R1, R2  ;apply new value 
	STR R1, [R0] ;  store
L311
  ;4)SetUp phase and polarity
    LDR R0, =SPI1_CR1
    LDR R1, [SP]
    LDR R2, =0x00000011
    AND R2, R1
    MOV R3, R2 ;copy 
    LSR R2, R2, #3 ;shift CPOL in according to the CR1
	AND R3, #1
    ORR R2, R3  ; apply CPHA bit
    LDR R1, [R0] ;load current
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store
  ;5)Frmae length (DFF  bit11)
    LDR R0, =SPI1_CR1
    LDR R1, [SP]
    LDR R2, =0x10000000
    AND R1, R2 ;filter DFF
    LSR R1, R1, #17 ;shit the bit to b11 as in CR1
    LDR R2, [R0] ;load CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;store changes
 ;6)LSB/MSBIFRST
    LDR R1, [SP]
    LDR R2, =0x00010000
    AND R1, R2 ;extract LSBFIRST bit
    LSR R1, R1, #9 ;sift to bit7 
    LDR R2, [R0] ;load current
    ORR R1, R2 ;apply
    STR R1, [R0] ;save CR1
  ;7)Setting Rx only mode
   ;BIDIMODE b15=1 (bidirectional MOSI line)
   ;BIDIOE b14=0  
   ;RXONLY b10=1 (Rx only) 
    LDR R1, =(1<<10)  ;(1<<15)|
    LDR R2, [R0] ;load current CR1
    ORR R1, R2 ;apply new data
    STR R1, [R0] ;save 

  ;8)Enable DMA channel processing on Rx
    LDR R0, =SPI1_CR2
	LDR R1, [R0]
	LDR R2, =0x1
	ORR R1, R2
	STR R1, [R0]
  ;9)Slave mode (b2=0) and enable SPI (b6=1)
    LDR R0, =SPI1_CR1
    LDR R2, =(1<<6)
    LDR R1, [R0] ;load current CR1
    ORR R1, R2 ;apply new
    STR R1, [R0] ;save CR1    
  ;10)Initializing DMA1_CH2 for Rx
   
   
    ;free memory
	ADD SP,SP,#0x12
	BX LR
	ENDP 
	LTORG	

;*******SPI********END-}

    END
