
Stack_Size EQU 0x400
top_of_stack EQU 0x20000000 + Stack_Size
 ;---C R Y S T A L   Q U A R T Z  = 12 288 000Hz-----
 
  
  EXPORT  __Vectors                ; Export the vector table
  PRESERVE8
  THUMB
  AREA RESET, DATA, READONLY
	  ;INTERRUPT VECTOR TABLE 
__Vectors       DCD     top_of_stack  ; Top of Stack $00000000
                DCD     Start         ; Reset Handler  $00000004
                DCD     Def_Vec       ; NMI Handler  $08
                DCD     TRASH_ISR      ; Hard Fault Handler  $0C
                DCD     Def_Vec       ; MPU Fault Handler    $10
                DCD     Def_Vec       ; Bus Fault Handler   $14
                DCD     Def_Vec      ; Usage Fault Handler   $18
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     Def_Vec      ; SVCall Handler  $2C
                DCD     Def_Vec      ; Debug Monitor Handler  $30
                DCD     0            ; Reserved
                DCD     Def_Vec     ; PendSV Handler     $38
                DCD     Def_Vec     ; SysTick Handler   $3C

                ; External Interrupts
                DCD     Def_Vec    ; Window Watchdog $40
                DCD     Def_Vec    ; PVD through EXTI Line detect $44
                DCD     Def_Vec     ; Tamper  $48
                DCD     Def_Vec     ; RTC  $4C
                DCD     Def_Vec    ; Flash  $50
                DCD     Def_Vec    ; RCC  $54
                DCD     Def_Vec    ; EXTI Line 0  $58
                DCD     Def_Vec    ; EXTI Line 1  $5c
                DCD     Def_Vec    ; EXTI Line 2  $60
                DCD     Def_Vec    ; EXTI Line 3  $64
                DCD     Def_Vec    ; EXTI Line 4  $68
                DCD     Def_Vec   ; DMA1 Channel 1  $6C
                DCD     Def_Vec   ; DMA1 Channel 2  $70
                DCD     Def_Vec   ; DMA1 Channel 3  $74
                DCD     Def_Vec   ; DMA1 Channel 4  $78
                DCD     Def_Vec   ; DMA1 Channel 5  $7C
                DCD     Def_Vec   ; DMA1 Channel 6   $80
                DCD     DMA1_CH7_ISR   ; DMA1 Channel 7   $84
                DCD     Def_Vec   ; ADC1_2         $88   
                DCD     Def_Vec    ; USB High Priority or CAN1 TX  $8C
                DCD     Def_Vec    ; USB Low  Priority or CAN1 RX0  $90
                DCD     Def_Vec   ; CAN1 RX1                      $94
                DCD     Def_Vec    ; CAN1 SCE                      $98
                DCD     Def_Vec    ; EXTI Line 9..5               $9C
                DCD     Def_Vec   ; TIM1 Break                    $A0
                DCD     Def_Vec   ; TIM1 Update                  $A4
                DCD     Def_Vec   ; TIM1 Trigger and Commutation      $A8
                DCD     Def_Vec    ; TIM1 Capture Compare         $AC
                DCD     tim2UpdateISR   ; TIM2                      $B0
                DCD     Def_Vec    ; TIM3                      $B4
                DCD     Def_Vec    ; TIM4                       $B8
                DCD     Def_Vec    ; I2C1 Event                   $BC
                DCD     Def_Vec    ; I2C1 Error                   $C0
                DCD     Def_Vec    ; I2C2 Event                   $C4
                DCD     Def_Vec    ; I2C2 Error                   $C8
                DCD     Def_Vec    ; SPI1                      $CC
                DCD     Def_Vec    ; SPI2                      $D0
                DCD     Def_Vec    ; USART1                      $D4
                DCD     usart2ISR    ; USART2                      $D8
                DCD     Def_Vec    ; USART3                      $DC
                DCD     Def_Vec   ; EXTI Line 15..10               $E0
                DCD     Def_Vec   ; RTC Alarm through EXTI Line   $E4
                DCD     Def_Vec    ; USB Wakeup from suspend        $E8
					
   AREA myConst , DATA, READONLY 
prompt   DCB "Donald Duck is our favorite candidate in President election!!!!!!!!!!!"	

   AREA MainCode, CODE, READONLY
	  ; |.text|
	  INCLUDE library.s  
  
   ENTRY
   
Start         PROC
     LDR r0, =RCC_APB2ENR
	 
	  BL gpio_init     ; Call the gpio_init procedure from the other file
	  LDR R0, =0x00300000  ;MODE C13=11 (bits20,21),   
	  LDR R1, =GPIOC_CRH 
	  STR R0, [R1]
	  ;--
	  LDR R0, =GPIOA_CRH
	  LDR R1,=0x0000000B
	  STR R1,[R0]
	  ;PA1 - alternative function TIM2_CH2
	  LDR R0, =GPIOA_CRL
	  LDR R1, =0x00004BB0 ;B alternative function push-pull, 50MHz
	  STR R1, [R0]
	  ;PORT B h
	  LDR R0, =GPIOB_CRH
	  LDR R1, =0x00030000
	  STR R1,[R0]
	  ;PORTB low
	  LDR R0, =GPIOB_CRL
	  LDR R1,=0x00003000
	  STR R1, [R0]
	  ;----------------------C L O C K --------------------
	  ;---------sys_clock = (12 288 000Hz / 2) * 5 = 30 720 000 
	  LDR R5, =0x03010001
	  LDR R6, =0x00000000
	  PUSH {R5,R6}
  ;first@32[PLL_mult|HSE_psc|AHB_psc|MCO_EN] ,
  ;second@32[APB1_psc|APB2_psc|ADC_psc|USB_psc]
	  BL clkConfig
	  ;--turn led on
	  LDR R0, =0x00002000 ; bit13  set b13 ($2000), reset b29 ($20000000)
	  LDR R1, =GPIOC_BSRR
	  STR R0, [R1]
	  LDR R0, =0x20000000 ; bit13  set b13 ($2000), reset b29 ($20000000)
	  STR R0, [R1]
	  LDR R0, =0x01002000 ; bit13 
	  LDR R1, =GPIOC_ODR
	  STR R0,[R1]
	  ;address of  on/off register for a led
	  ;LDR R1, =0x007A1200;
	  ;LDR R2, =0x00bb;
	  ;PUSH {R1,R2}
	  ;BL tim2OcCh2Setup
	    
	 
;=======FUNCTION uart_init_tx with Interrupt
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)] 
;par2@[-|-|-|interrups]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
       ; LDR R0, =0x00000905
	    ;LDR R1, =0x00000000
	    ;PUSH {R0,R1}
	    ;BL uart_init_tx
	   
;=======FUNCTION uart_init_rx with Interrupt
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  
;par2@[interrupt_tx_enable(8)|interrupt_rx_enable(8)|parity_type(8)|prity_en(8)]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, 0b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;divider = [whole_part_divider | b3 remainder b0]
;parity_en:   0-> disable parity, 1-> enable parity
;parity_type: 1->odd, 0->even
       ;LDR R0, =0x00000C80	   
	   ;LDR R1, =0x00FF0000
	   ;PUSH {R0,R1}
	   ;BL uart_init_duplex
	   
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
  LDR R0, =0x00000C80
  LDR R1, =0x000000ff;
  LDR R2, =prompt
  LDR R3, =0x01010040 ;64bytes TX ROM buffer
  PUSH {R0,R1,R2,R3}
  BL uart_init_tx_dma
  LDR R0,=DMA1_CCR7
  LDR R2, =0x1
  LDR R1,[R0]
  ORR R1,R2
  STR R1,[R0]
	    ;LDR R2, =GPIOC_BSRR
	   ; LDR R1, =0x007A1200;
	    ;PUSH {R1,R2}
	    ;BL tim2SimpleModeInitInt
mylabel
    ;WFI
	B mylabel
	ENDP
;---data---area

		
		
 ;ISR handlers---------
 ;============default ISR handler.Infinite loop
Def_Vec PROC
    B   .  ; Infinite loop
    ENDP
;====TIM2=====ISR========		
tim2UpdateISR   PROC
	;clear flag UIE
	LDR R0, =TIM2_SR
	LDR R1, [R0]  ;load current content
	BIC R1, R1, #0x01    ; Clear UIF flag (bit 0)
	STR R1, [R0] ; update the SR
	;---LED BLINK--BEGIN
	;is the led in high (PC13) ?
	LDR R0, =GPIOC_IDR
	LDR R1, [R0]
	LDR R3, =(1 << 13)
	ANDS R1, R1, R3
	BEQ t001_cleared
	;when bit set - clear it
	  LDR R0, =GPIOC_BSRR
	  LDR R1, =0x20000000 ; bit13  set b13 ($2000), reset b29 ($20000000)
	  STR R1, [R0]
	  B t001_end
t001_cleared
      ;when bit cleared -set in to 1
      LDR R0, =GPIOC_BSRR
	  LDR R1, =0x00002000 ; bit13  set b13 ($2000), reset b29 ($20000000)
	  STR R1, [R0]
	  B t001_end      	  
t001_end
     ;---LED-BLINK---END
	  LDR R0, =USART2_DR
	  LDR R1, =0x30
	  STR R1, [R0]
	BX LR ;reti
	ENDP
;=====USART2==Interrupt Service Routine=
usart2ISR  PROC
	
	LDR  R0,=USART2_DR
	LDR R1,[R0]
	STR R1,[R0]
	BX LR
	  ENDP
;==========DMA1 CH7 (UART2 TX)
DMA1_CH7_ISR PROC
   BX LR
   ENDP
;======hard fault ISR
TRASH_ISR PROC
	B .
	ENDP
 
   ALIGN
   END