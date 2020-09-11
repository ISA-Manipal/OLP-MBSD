				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV r0, #18
				ADD r0, r0, #20
				MOV r1, #100
				
				RSB r2, r1, r0		;Reverse subtraction operation
				MVN r3, r2			;One's complement
				ADD r3, r3, #01		;Two's complement - Actual answer
				;The actual answer is 3E, 62 in hexadecimal, so Suresh is wrong
				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END