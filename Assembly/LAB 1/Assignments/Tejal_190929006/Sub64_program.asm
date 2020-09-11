				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				LDR r0, =0x12233345
				MOV r1, #0xFFFFFFFF
				
				MOV r2, #0x56565656
				LDR r3, =0x99987654
				
				SUBS r4,r0,r2
				SBCS r5,r1,r3
				
				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END