		INCLUDE MSP432xx_constants.s
		INCLUDE MSP432xx_tim_constants.s
		AREA main, CODE, READONLY
		;EXPORT __main
		;ENTRY
		
__main 	PROC
		LDR R0,=0x12345678
		MOV R1,#0x20000000
		STR R0, [R1], #4
		ENDP
		
		ALIGN
		AREA allocations, DATA, READWRITE
		END