        INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV	    R1, #0xFFFFFFFF
				LDR     R2,= 0x12345678    
				LDR     R3,= 0x32740432    ; R3-R2-R1  MSB-middle-LSB
				
				LDR     R4,= 0x67154827
				MOV     R5, #0x12121212
				LDR     R6,= 0x82742300    ; R6-R5-R4  MSB-MID-LSB 
				
			  ; Register B - Register A 
				
				ADDS    R7, R4, R1     ; ADD the LSWs
				ADDS    R8, R5, R2     ; ADD the Middle USE-Carry
				ADC     R9, R6, R3     ; ADD the MSWs USE-Carry = R9-R8-R7---> MSB-MID-LSB
			
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END