				INCLUDE MSP432xx_constants.s	; Load Constant Definitions
				INCLUDE MSP432xx_tim_constants.s   ; TIM Constants
				AREA    main, CODE, READONLY
string 			DCB "manipal",0
				EXPORT	__main		; make __main visible to linker
				ENTRY


				
				
__main			PROC
	
				LDR R0,=string
				LDR R10,=0x00000007 ; string length 
				MOV R2,#'a'; letter to be found
				MOV R3,#0 ; occurrence of a 

__loop
				LDRB R1,[R0,# 1]! ;load first letter
				SUBS R10,R10,#1   
				CMP  R1,R2
				BEQ found
				BNE __loop
				ENDP
				
				
found				
				ADD R3,R3,#1
				B __loop
	

				
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END
				
				
