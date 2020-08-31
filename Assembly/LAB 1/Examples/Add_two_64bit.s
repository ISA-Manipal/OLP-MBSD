				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV	    R1, #0xFFFFFFFF
				LDR     R2,= 0x12345678    ; R2R1 (MSB,LSB)
				
				LDR     R3,= 0x67154827
				MOV     R4, #0x12121212    ; R4R3 (MSB,LSB)
				
				
				
				ADDS    R5, R1, R3 ; Add the LSBs 
				ADC     R6, R2, R4 ; Add the MSBs => Resulting register = R6R5 (MSB,LSB)
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END