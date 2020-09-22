				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV R0, #0x0F0F0F0F
				EOR R1, R0, #0xFFFFFFFF ;R1 is the one's complement of the value in R0
				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END