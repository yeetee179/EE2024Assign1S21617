 	.syntax unified
 	.cpu cortex-m3
 	.thumb
 	.align 2
 	.global	pid_ctrl
 	.thumb_func
@  EE2024 Assignment 1: pid_ctrl(int en, int st) assembly language function
@  CK Tham, ECE, NUS, 2017
pid_ctrl:
@ PUSH the registers you modify, e.g. R2, R3, R4 and R5*, to the stack
@ * this is just an example; the actual registers you use may be different
@ (this will be explained in lectures)
	PUSH	{R2-R5}

@  Write PID controller function in assembly language here
@  Currently, nothing is done and this function returns straightaway
//write code here
///////////////////////////////////////////////////////////////////////////////////////////////

@input is en and st, assume both are integers
@en is in R0, st is R1
@set R10 to 0
@load everything into register.
   @LDR R0,en
   @LDR R1,st

	LDR R2,=sn    @R5 = sn     address
	LDR R3,=enOld @R6 = enold  address
	LDR R4,=un    @R7 = un     address

	LDR R5,[R2]   @R5 = sn     value
	LDR R6,[R3]   @R6 = enOld  value
	LDR R7,[R4]   @R7 = un     value

	CMP R1,#1     @if (st ==1)  sn = enOld = 0.0
	ITT EQ
	MOVSEQ R5, #0
	MOVSEQ R6, #0

	ADD R5,R0     @sn = sn + en

	CMP R5,#95
	IT GE
	MOVGE R5,#95

@@compare negative here dunno how to do




	LDR R8,KP      @R8 = Kp
	MUL R9,R0,R8   @R9 = Kp*en
	LDR R8,KI      @R8 = ki
	MUL R10,R5,R8  @R10 = Ki*sn
				   @R8 is free
	ADD R10,R9	   @R10 = Kp*en + Ki*sn
	               @R9 is free
	LDR R9,KD      @R9 = Kd
				   @R9 is NOT free
	SUB R0,R6      @R0 = (en-enOld)
	MUL R8,R0,R9   @R8 = kd * (en-enOld)
				   @R8 is not free
	ADD R8,R10     @R8 = un

	STR R0,[R3]

	MOV R0,R8




//declare constant.
KP:
	.word 25
KI:
	.word 10
KD:
	.word 80
CONSTANT_1:
	.word 1
CONSTANT_0:
	.word 0
CONSTANT_EN_TEST_300:
	.word 300
CONSTANT_COMPARE_NEGATIVE:
	.word -95
//declare variables
.lcomm sn 4
.lcomm enOld 4
.lcomm un 4




@ POP the registers you modify, e.g. R2, R3, R4 and R5*, from the stack
@ * this is just an example; the actual registers you use may be different
@ (this will be explained in lectures)
//why stack (push and pop)////////////////////
	POP	{R2-R5}
 	BX	LR
//BX exit from assemblys
