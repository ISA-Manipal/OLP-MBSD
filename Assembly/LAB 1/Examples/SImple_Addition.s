				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV	    R1, #4
				MOV     R2, #7
				ADD     R3, R2, R1
			

				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END