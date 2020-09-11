				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV	    R1, #0xFFFFFFFF
				MOV 	R2, #0x11111111
				LDR     R3,= 0x12345678    ; R3R2R1 (MSB,mid,LSB)
				
				LDR     R4,= 0x67154827
				LDR		R5,= 0x98765432
				MOV     R6, #0x12121212    ; R6R5R4 (MSB,mid,LSB)
				
				
				
				ADDS    R7, R1, R4 ; Add the LSBs
				ADCS	R8, R2, R5 ; Add mids
				ADC     R9, R3, R6 ; Add the MSBs => Resulting register = R9R8R7 (MSB,mid,LSB)
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END