				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				MOV R1, #0x00000012
				MOV R2, #0x00000014
				MOV R3, #0x00000064
				ADD R4,R1,R2 ; R4 = R1+R2
				RSB R5,R3,R4 ; R5 = R4-R3, LOWER-HIGHER
				
				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END