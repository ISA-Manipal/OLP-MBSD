	                        INCLUDE MSP432xx_constants.s	; Load Constant Definitions
				INCLUDE MSP432xx_tim_constants.s   ; TIM Constants
				AREA    main, CODE, READONLY
				EXPORT	__main		; make __main visible to linker
				ENTRY
				
__main			PROC
	
				MOV R0, #08
				MOV R8, #0x0
				MOV R1, #0x01
				
__loop 			
                AND R2, R0, R1
			    CMP R0, R8
				BEQ here

here			MOV R2 , R0


				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END
