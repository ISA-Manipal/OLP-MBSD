				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				;Considering high input to be all 1s(#0xFFFFFFFF)
				;Low input is all 0s(#0x00)
				MOV R0,#0xFFFFFFFF
				MOV R3,#0xFFFFFFFF
				MOV R4,#0xFFFFFFFF
				MOV R5,#0xFFFFFFFF
				
				AND R1, R0, R3			 
				AND R2, R4, R5
				BICS R6, R1, R2		;If even one of the inputs was 0, the Z flag will be 0
				MOVEQ R7,#0xFFFFFFFF ;If the Z flag is 1, R7 will be high
				
stop
				B stop
				
				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END