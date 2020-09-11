				;INCLUDE MSP432xx_constants.s          
				;INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC

				MOV R0,#18
				MOV R1,#20
				ADD R2,R1,R0 ;Total amount spent
				MOV R3,#100
				
				RSB R4, R3, R2
				RSB R5, R4,#0
				; balance is  0x3E i.e Rs. 62
				


__stop			B __stop

				ENDP