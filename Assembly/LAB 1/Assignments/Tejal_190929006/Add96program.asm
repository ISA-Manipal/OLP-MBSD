				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
__main			PROC
				
				LDR r0, =0x98765432
				MOV r1, #0x11111111
				LDR r2, =0x12345678
				
				MOV r3, #0xFFFFFFFF
				LDR r4, =0x11223344
				LDR r5, =0x55667788
				
				ADDS r6,r0,r3
				ADCS r7,r1,r4
				ADCS r8,r2,r5
			
				ENDP
					
				ALIGN
				AREA allocations, DATA, READWRITE
				END