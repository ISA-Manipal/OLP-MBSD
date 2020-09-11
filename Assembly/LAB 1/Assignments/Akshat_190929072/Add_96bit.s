				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				LDR	    R1,= 0x12345678
				LDR     R2,= 0x87654321    
				LDR     R3,= 0x12345678    ; R3R2R1  Register A
				
				LDR     R4,= 0x22233344
				LDR     R5,= 0x45556667
				LDR     R6,= 0x77888999    ; R6R5R4  Register B 
				
			  ; Register B - Register A 
				
				ADDS    R7, R4, R1     ; add the LSWs
				ADCS    R8, R5, R2     ; add the Middle Words with Carry
				ADCS    R9, R6, R3     ; Add the MSWs with Carry  ===> R9R8R7
			
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END
