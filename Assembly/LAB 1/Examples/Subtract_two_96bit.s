				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV	    R1, #0xFFFFFFFF
				LDR     R2,= 0x12345678    
				LDR     R3,= 0x32740432    ; R3R2R1  Register A
				
				LDR     R4,= 0x67154827
				MOV     R5, #0x12121212
				LDR     R6,= 0x82742300    ; R6R5R4  Register B 
				
			  ; Register B - Register A 
				
				SUBS    R7, R4, R1     ; Subtract the LSWs
				SBCS    R8, R5, R2     ; Subtract the Middle Words with Carry
				SBC     R9, R6, R3     ; subtract the MSWs with Carry  ===> R9R8R7
			
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END