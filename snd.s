
Stack_Size EQU 0x400
top_of_stack EQU 0x20000000 + Stack_Size
    
	INCLUDE library.s 	

  EXPORT  __Vectors                ; Export the vector table
  PRESERVE8
  THUMB
  
  AREA RESET, DATA, READONLY
	  ;INTERRUPT VECTOR TABLE 
__Vectors               DCD     top_of_stack  ; Top of Stack $00000000
                DCD     Start         ; Reset Handler  $00000004
                DCD     Def_Vec       ; NMI Handler  $08
                DCD     Def_Vec       ; Hard Fault Handler  $0C
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
                DCD     Def_Vec   ; DMA1 Channel 7   $84
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
                DCD     Def_Vec    ; USART2                      $D8
                DCD     Def_Vec    ; USART3                      $DC
                DCD     Def_Vec   ; EXTI Line 15..10               $E0
                DCD     Def_Vec   ; RTC Alarm through EXTI Line   $E4
                DCD     Def_Vec    ; USB Wakeup from suspend        $E8
					
   AREA |.text|, CODE, READONLY
   ENTRY
   
Start         PROC
     LDR r0, =RCC_APB2ENR
	 
	  BL gpio_init     ; Call the gpio_init procedure from the other file
	  LDR R0, =0x00300000  ;MODE 13=11 (bits20,21)
	  LDR R1, =GPIOC_CRH 
	  STR R0, [R1]
	  ;PA1 - alternative function TIM2_CH2
	  LDR R0, =GPIOA_CRL
	  LDR R1, =0x000000B0 ; alternative function push-pull, 50MHz
	  STR R1, [R0]
	  ;PA8 -MCO
	  LDR R0, =GPIOA_CRH
	  LDR R1, =0x0000000B ; alternative function push-pull, 50MHz
	  STR R1, [R0]
	  ;clock
	  BL clkConfig
	  ;--turn led on
	  LDR R0, =0x00002000 ; bit13  set b13 ($2000), reset b29 ($20000000)
	  LDR R1, =GPIOC_BSRR
	  STR R0, [R1]
	  LDR R0, =0x20000000 ; bit13  set b13 ($2000), reset b29 ($20000000)
	  STR R0, [R1]
	  LDR R0, =0x00002000 ; bit13 
	  LDR R1, =GPIOC_ODR
	  STR R0,[R1]
	  ;address of  on/off register for a led
	    LDR R1, =0x007A1200;
	  LDR R2, =0x00bb;
	  PUSH {R1,R2}
	  BL tim2Pwm1Ch2Setup
	 ; LDR R2, =GPIOC_BSRR
	 ; LDR R1, =0x007A1200;
	 ; PUSH {R1,R2}
	 ; BL tim2SimpleModeInitInt
	
	  
mylabel
     
	B mylabel
	ENDP
		
		
 ;ISR handlers---------
 ;default ISR handler.Infinite loop
Def_Vec PROC
    B   .  ; Infinite loop
    ENDP
tim2UpdateISR   PROC
	;clear flag UIE
	LDR R0, =TIM2_SR
	LDR R1, [R0] ;load current content
	BIC R1, R1, #0x01          ; Clear UIF flag (bit 0)
	STR R1, [R0] ; update the SR
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
	BX LR ;reti
	ENDP
 
   ALIGN
   END