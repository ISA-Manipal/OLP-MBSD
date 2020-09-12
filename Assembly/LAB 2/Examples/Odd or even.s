  
				INCLUDE MSP432xx_constants.s	; Load Constant Definitions
				INCLUDE MSP432xx_tim_constants.s   ; TIM Constants
				AREA    main, CODE, READONLY
				EXPORT	__main		; make __main visible to linker
				ENTRY
				
__main			PROC
	
				LDR	R0, = 0x20000002
				LDR R1, = 0x20000001
				
				LSRS R1,#1
				
				BCC __even
				LDR R3, = 0x20000001
				

here			B here	


__even
				LDR R4, = 0x20000002

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END
