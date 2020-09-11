				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				LDR R0, =0X12345678		;Number 1 -- R1_R0
				LDR R1, =0X13141517
		
				LDR R2, =0X22FA23B1		;Number 2 -- R3_R2
				LDR R3, =0X11223344
		
		;Subtraction of Number 2 and Number 1
		
				SUBS R5, R2, R0			;Substract lower 32 bits
				SBCS R4, R3, R1			;Subtract higher 32 bits	

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END