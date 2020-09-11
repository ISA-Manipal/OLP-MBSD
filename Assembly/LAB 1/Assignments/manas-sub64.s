				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
				
				
__main			PROC
				
				
				MOV R0,#0x22222222	
				LDR R1,= 0x12345678	
				
				LDR R2,= 0x87654321	
				MOV R3,#0xFFFFFFFF	
	
				SUBS R4, R2, R0	
				SBC R5, R3, R1	
				
				
				
__stop			B __stop
				ENDP