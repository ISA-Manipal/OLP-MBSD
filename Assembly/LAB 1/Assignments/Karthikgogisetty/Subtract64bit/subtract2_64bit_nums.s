				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV	    R1, #0xFFFFFFFF
				LDR     R2,= 0x12345676    ; R2-R1 (MSB-LSB)
				
				LDR     R3,= 0x67154826
				MOV     R4, #0x12121212    ; R4-R3 (MSB-LSB)
				
				
				
				SUBS    R5, R1, R3 ; sub the LSBs 
				SBC     R6, R2, R4 ; (MSB-LSB)
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END