		;INCLUDE MSP432xx_constants.s
		;INCLUDE MSP432xx_tim_constants.s
		AREA main,CODE,READONLY	
		EXPORT __main
		ENTRY
__main	PROC
		LDR R1, =0x01234ABC 
		MVN R2, R1
 
		ENDP
					
		ALIGN
		AREA allocations, DATA, READWRITE
		END
		