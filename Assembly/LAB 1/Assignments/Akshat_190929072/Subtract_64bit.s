				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				LDR	    R1,= 0x12436578   
				LDR     R2,= 0x32740432    ; R2R1  Register A
				
				LDR     R3,= 0x67154827
				LDR     R4,= 0x12121212    ; R4R3  Register B 
				
			  ; Register B - Register A 
				
				SUBS    R5, R3, R1     ; Subtract the LSWs
				SBCS    R6, R4, R2     ; subtract the MSWs with Carry  ===> R6R5
			
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END
