    INCLUDE stm32f103c8.inc
	THUMB
    AREA    text, CODE, READONLY
    EXPORT  gpio_init, delay_, tim2SimpleModeInitInt, tim2Pwm1Ch2Setup, clkConfig   ; Export the procedure to make it visible to the main file

gpio_init   PROC
    LDR R0, =0x00000015   ; enable alternate, GPIOC and GPIOA clock (Bit 4 of RCC_APB2ENR)
    LDR R1, =RCC_APB2ENR  ; Load the address of RCC_APB2ENR
    STR R0, [R1]          ; Store the value into RCC_APB2ENR
    BX LR                 ; Return from subroutine
            ENDP
delay_            PROC
	LDR R2, =0xF4240; 1000000 ;counter value
rep1
    SUBS R2, R2, #0x00000001  ;decrement
	BNE rep1             ;until great zero
	BX LR
	               ENDP
	;*********MUST BE UPDATED!
		;FUNCTION initialization counter in simple mode with interrupt	
    ;parameters:  @32bit - [b31  prescaler| counter b0]  ,@32bit-null	
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
	               ENDP
	;**********
	;FUNCTION initialization counter in simple mode with interrupt	
    ;parameters:  @32bit - [b31  prescaler| counter b0]  ,@32bit-null	
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
	POP {R2,R1} ;R3 - is a 'ballast' in this case, because M0 can`t push/pop only one reg
	;--high 16 bit - is a prescaler,low 16 bit is a counter (ARR)
	MOV R3, R1
	LSR R3, R3, #16
    ;A) ----prescaler:
    LDR R0, =TIM2_PSC;
    STR R3, [R0] 
    ;B) counter---
	LDR R0, =TIM2_ARR
    LDR R3, =0x0000FFFF;
    AND R1, R3
	STR R1, [R0]  	
	;-----load constant in count reg
	LDR R0, =TIM2_ARR
	LDR R1, =0x000F4240 ; 1000000
	STR R1, [R0]  ;set reg
	;---optional: enable (Start) counter----
    LDR R0, =TIM2_CR1
	LDR R1, =0x01 ; CEN bit
	STR R1, [R0]
	BX LR  ;return
	               ENDP
	;====FUNCTION initialize TIM2 CH2 PWM
	;    A@[b31 presc| period b0],
	;Width@[b31      |  width b0]
    ;example bassing params: PUSH {A,Width}
tim2Pwm1Ch2Setup     PROC
	 ;enable clock TIM2
	LDR R3, =0x01 ; TIM2 clock enable
    LDR R0, =RCC_APB1ENR
	LDR R1, [R0] ;load content  APB1ENR
	ORR R1, R1, R3 ;ON bit
	STR R1, [R0]  ;update APB1ENR
	;--load prescaler and counter from stack
	POP {R2,R1} ;R1:[presc|period],R2[..|width]
	MOV R3, R1
	LSR R3, R3, #16 ; R3>>16
    ;A) ----prescaler:
    LDR R0, =TIM2_PSC;
    STR R3, [R0] 
    ;B) counter---
	LDR R0, =TIM2_ARR
    LDR R3, =0x0000FFFF;
    AND R1, R3
	STR R1, [R0]  	
	;C) width
	LDR R0, =TIM2_CCR2
	STR R2, [R0]
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
	 BX LR
                  ENDP
	;====FUNCTION initialize TIM2 CH2 Output Compare
	;    A@[b31 presc| period b0],
	;Width@[b31 null |  width b0]
	;example bassing params PUSH {A,Width}
tim2OcCh2Setup     PROC
	 ;enable clock TIM2
	LDR R3, =0x01 ; TIM2 clock enable
    LDR R0, =RCC_APB1ENR
	LDR R1, [R0] ;load content  APB1ENR
	ORR R1, R1, R3 ;ON bit
	STR R1, [R0]  ;update APB1ENR
	;--load prescaler and counter from stack
	POP {R2,R1} ;R1:[presc|period],R2[..|width]
	MOV R3, R1
	LSR R3, R3, #16 ; R3>>16
    ;A) ----prescaler:
    LDR R0, =TIM2_PSC;
    STR R3, [R0] 
    ;B) counter---
	LDR R0, =TIM2_ARR
    LDR R3, =0x0000FFFF;
    AND R1, R3
	STR R1, [R0]  	
	;C) width
	LDR R0, =TIM2_CCR2
	STR R2, [R0]
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
	 BX LR
                  ENDP
;======function clock config
;first@32[ b31 PLL_mult | HSE_psc |AHB_psc |MCO_EN ] ,
;second@32[b31 APB1_psc |APB2_psc |ADC_psc | USB_psc ]
;exaple passing parameters -> PUSH {first, second}
;NOTE: dividers passed in CFGR register as is
;plese see the datasheet
;SP + 0 after return
clkConfig   PROC
	;load params 
	;allocate var
	MOV R0, SP
	SUB R0,R0, #0x04
	MOV SP, R0
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
	EOR R4, R4, R4  ;clear 
	STR R4, [SP] ;clear var
	;extract divider)
	LDR R3, [SP,#0x04]
	LDR R2, =0xFF000000;
	AND R3,R3, R2
	LSR R3, R3, #0x6
	STR R3, [SP]  ;store in memory
	;extract HSE psc
	LDR R3, [SP,#0x04]
	LDR R2, =0x00010000;
	AND R3, R3, R2 
	LSL R3, R3, #0x1
	LDR R4, [SP]
	ORR R3, R3, R4
	STR R3, [SP]
	;extract MCO bit
	LDR R3, [SP,#0x04]
	LDR R2, =0x00000001;
	AND R3, R3, R2 
	LSL R3, R3, #0x1A 
	LDR R4, [SP]
	ORR R3,R3,R4
	;HSE - input for sysclock
	LDR R4, =(1<<16)
	ORR R3, R3, R4
	STR R3,[SP]
    ;C)----Clock Source for the PLL and HSE divider:
	LDR R0, =RCC_CFGR
	LDR R1, [SP]
	STR R1, [R0];
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
	STR R4, [SP] ;clear var
     ;1) AHB
  	LDR R3, [SP,#0x04]  ;load parameter from stack
	LDR R2, =0x00000F00  ;template for the parameter
	AND R3, R3, R2  ;save only intrersted bits (AHB)
	LSR R3, R3, #0x4  ;shift to acheive order bits as in RCC_CFGR
	STR R3, [SP] ; store AHB coef 
	  ;2)APB1
  	LDR R3, [SP,#0x08]
	LDR R2, =0x07000000
	AND R3, R3, R2
	LSR R3, R3, #0x10
    LDR R1, [SP] ;store previous result
    ORR R3, R3, R1 ;add prev to current
    STR R3, [SP] ;stre again
      ;3)APB2
    LDR R3, [SP,#0x8]
    LDR R2, =0x00070000;
    AND R3, R3, R2
    LSR R3, R3, #0x5
    LDR R2, [SP]
    ORR R3, R3, R2
    STR R3, [SP]
       ;4) ADC
    LDR R3, [SP,#0x8]
    LDR R2, =0x00000300;
    AND R3, R3, R2
    LSL R3, R3, #0x6
    LDR R2, [SP]
    ORR	R3,R3,R2
	STR R3, [SP]
	   ;5) USB
    LDR R3, [SP,#0x8]
    LDR R2, =0x00000001;
    AND R3, R3, R2	
	LSL R3, R3, #0x17
	LDR R2, [SP]
	ORR R3, R3, R2
	STR R3, [SP]
	   ;6) apply content to the CFGR
	LDR R0 , =RCC_CFGR
	LDR R1, [R0]
	ORR R3, R3, R1
	STR R3, [R0]
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
	MOV R0, SP
	ADD R0, R0, #0x0C
	MOV SP, R0
	BX LR
	ENDP
 
    END
