				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV	    R1, #0xFFFFFFFF
				LDR     R2,= 0x12345678    
										; R2R1  Register A
				
				LDR     R3,= 0x67154827
				MOV     R4, #0x12121212
										; R4R3  Register B 
				
			  ; Register B - Register A 
				
				SUBS   R5, R3, R1     ; Subtract the LSWs
				SBC    R6, R4, R2     ; subtract the MSWs with Carry  ===> R6R5
			
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END