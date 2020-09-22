				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				LDR R0, =0x41424344	;ASCII value representation of 'ABCD'
				MOV R1, #'A'		;Letter
				MOV R2, #0x00		;Counter
loop
				UBFX R3, R0, #00, #08	;Extract one character
				EORS R3, R1			;Compare to given letter
				ADDEQ R2, #0x01		;Increment counter
				LSRS R0, #08		;Next character
				BNE loop			;If R0 is not empty
				BEQ stop			;If R0 is full of 0s
stop
				B stop
				
				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END