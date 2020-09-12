	                        INCLUDE MSP432xx_constants.s	; Load Constant Definitions
				INCLUDE MSP432xx_tim_constants.s   ; TIM Constants
				AREA    main, CODE, READONLY
				EXPORT	__main		; make __main visible to linker
				ENTRY
				
__main			PROC
	
				MOV R1, #4
				TST R1, 0x0
				MOVNE R5, R1 
			        MOVEQ R6, R1


				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END
