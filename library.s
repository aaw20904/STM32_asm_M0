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
	;FUNCTION initialize TIM2 CH2 PWM
	;    A@[b31 presc| period b0],
	;Width@[b31      |  width b0]
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
; function clock config
clkConfig   PROC
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
    ;C)----Clock Source for the PLL and HSE divider:
	LDR R0, =RCC_CFGR
	         ;HSE - surce, HSE/2, x4, MCO  enable
	LDR R1, =0x0413000A
	STR R1, [R0];
	;D) Enable PLL
	LDR R3, =(1<<24) ; PLL_ON
	LDR R0, =RCC_CR
	LDR R1, [R0]
	ORR R1, R1, R3
	STR R1, [R0]
	;E) waiting until PLL enable:
	LDR R3, =(1<<25) ;PLL_RDY
clk_pll_rdy
    LDR R1 , [R0]  ;load RCC_CR
    ANDS R1, R1, R3
    BEQ  clk_pll_rdy
    ;F) SEt PLL as clock source for system:
    LDR R0, =RCC_CFGR
    LDR R1, [R0]
    LDR R3, =(1<<1) ; SW bits:  PLL as system clock
	ORR R1, R1, R3
	STR R1, [R0]
	;G)wait until RCC has been swiched to PLL:
    LDR R1, [R0]
    LDR R3, =(1<<3) ; SW bits:  PLL as system clock
clk_rcc_bus_rdy
    LDR R1, [R0]
	ANDS R1, R1, R3
	BEQ clk_rcc_bus_rdy
	BX LR
	ENDP
 
    END
