				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
				; Program to find 1's complement of a number
				
__main			PROC
				
				; Moving 15 into R0
				MOV r0, 0xF
				
				; Move with negative. R1 = -R0
				; This 'flips' the bits of the given input 
				; to get 1's complement
				MVN r1, r0
				
				
__stop			B __stop
				ENDP