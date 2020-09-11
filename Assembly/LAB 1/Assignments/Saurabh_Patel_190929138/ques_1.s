  INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   

				AREA main, CODE, READONLY
				EXPORT __main
__main			PROC
				
				;INT 1
				LDR r0,= 0x22222222	;LSB
				LDR r1,= 0x44444444	
				LDR r2,= 0x88888888	;MSB
				
				;INT 2
				LDR r3,= 0xF2222222	;LSB
				LDR r4,= 0xC4444444	
				LDR r5,= 0x66666666	;MSB
				
				;ADD
				ADDS r6, r3, r0	;LSB
				ADC r7, r4, r1	
				ADC r8, r5, r2	;MSB
				;Final int is [R6 R7 R8]
		
__stop			B __stop

				ENDP