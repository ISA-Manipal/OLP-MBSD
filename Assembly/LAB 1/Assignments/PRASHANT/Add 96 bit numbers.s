				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				LDR R2, =0X12345678		;Number 1 -- R2_R1_R0
				LDR R1, =0X13141517
				LDR R0, =0X25032AB1
		
				LDR R5, =0X22FA23B1		;Number 2 -- R5_R4_R3
				LDR R4, =0X11223344
				LDR R3, =0X230ABF12
		
		;Addition of Number 1 and Number 2
		
				ADDS R8, R3, R0			;Add lower 32 bits
				ADCS R7, R4, R1			;Add middle 32 bits	
				ADCS R6, R5, R2			;Add higher 32 bits

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END