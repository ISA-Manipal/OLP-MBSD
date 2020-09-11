				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				LDR     R0,=87654321
				MOV	    R1, #0xFFFFFFFF
				LDR     R2,= 0x12345678    
				
				LDR     R3,=17122001
				LDR     R4,= 0x55559876
				MOV     R5, #0x17171717    
				
				
				
				ADDS    R6,R0,R3
				ADC     R7,R1,R4
				ADC     R8,R2,R5
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END