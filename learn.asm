;-ARM--ARCHITECTURE------------------
; Data can be only loaded/stored from memry. Data processing can be done only in registers 
;-----ARM--INSTRUCTION SETS choose------
;Before starting  - define instruction set for your CPU by directives (THUMB or ARM or  ThumbEE)
;there can be THUMB or ARM instruction set. When core runs with THUMB , it can`t runs any ARM instruction
;----B Y T E  O R D E R-----------------------
;ARM has little-endian order: low byte has lower address, high byte has bigger address
;-----S T M 32 F0---i n s t r u c t i o n----s e t------------
;The Cortex®-M0 processor implements the Arm®v6-M architecture, which is based on the 16-bit Thumb®
;instruction set and includes Thumb-2 technology
;--Cotex-M0--S T A C K--------------------------------
;Cortex-Mo has descending (нисходящий) stack.After PUSH stack pointer decremented (-1 or bigger).
;There are TWO sstacks - the MAIM stack and the PROCESS stack.When CPU runs in thread mode it uses
; the process stack;
;handler mode (exception handlers) ->main stack
;thread(applications)->main or process stack
;Bit 1 in CONTROL defines: 0 -> SP is a prosess stack pointer, 
;                          1 -> main stack pointer
;--PUSH/POP----------
;there must be at least two registers in a list, for example:
PUSH {R0,R2}
POP {R2,R0}
;-P R O C E S S O R---R E G I S T E R S--------
;R0-R12 (general purpose)
; SP(R13)-stack pointer, LR(R14) - link register,  PC(R15)-prog.counter
;special resg:
;PSR -program status, PRIMASK-interrupt mask, CONTROL
;--P R O G R A M---S T A T U S---R E G I S T E R
; a) APSR -application PSR
; [31-N|30-Z|29-C|28-V|......0]
; b) IPSR interrupt PSR
;[........|5 exception number 0]
;c) execution PSR
; [.......|24 thumb state bit|....0]
;--P R I M A S K--r e g i s t e r
;[...|0  Prevents the activation of all exceptions with configurable priority]
;---C O N T R O L---R E G I S T E R
;[.....|1 active stack ptr|.]
;0->MSP (main SP), 1->PSP (process SP)
;-----C o r t e x --M0--s t a c k-----f r a m e
;[R0|R1|R2|R3|R12|LR|PC|xPSR]
;---P R O C E D U R E S--
;When a procedure calle from main program, the return address 
; stores into the link register (LR).When a procedure called from 
; an another procedure, return address stored in the stack.
;-----I N T E R R U P T S-------

