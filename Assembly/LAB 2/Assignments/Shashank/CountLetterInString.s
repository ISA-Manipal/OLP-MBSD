				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
				; Program to count a letter in a string
				
__main			PROC
				
				; Setting r0 as base register
				LDR r0, =0x20000000
				
				; =============================================================
				
				; BAD WAY TO DO THIS:
				
				; Writing in hello world
				; MOV r2, #0x48 ; 0x48 => 'H'
				; MOV r3, #0x65 ; 0x65 => 'e'
				; MOV r4, #0x6C ; 0x6C => 'l'
				; MOV r5, #0x6C ; 0x6C => 'l'
				; MOV r6, #0x6F ; 0x6F => 'o'
				; MOV r7, #0x20 ; 0x20 => ' '
				
				; Store 'Hello '
				; STMIA r0, {r2-r7}
				
				; =============================================================
				
				; GOOD WAY TO DO THIS:
				
				; Another way to do the same is using dcb command as shown

				; Store the balue of dcb in a variable called world
helloworld		DCB "Hello world\n",0
				
				; Now to store it
				
				; r1 is a pointer to the string
				LDR r1, =helloworld
				
				; Iterative string copy function
strcpy
				LDRB r2, [r1], #0x1
				STRB r2, [r0], #0x1
				CMP r2, #0x0
				BNE strcpy
				
				; =============================================================
				
				; Count the number of occurences of 'l' in r5
				MOV r3, #0x6c			; Hex value for 'l'
				MOV r5, #0x0
				MOV r0, #0x20000000		; Resetting counter at start
				
checkl
				LDRB r4, [r0], #0x1
				
				CMP r4, #0x0	; Check if string ended
				BEQ __stop		; Stop if string ended
				
				CMP r4, r3		; Check if r4 has 'l'
				BEQ isl			; If r4 is 'l', go to isl
				BNE isnotl		; If not go to isnotl

isnotl
				B checkl			; Go back to checkl
	
isl
				ADD r5, r5, #0x1	; Add 1 to r5
				B checkl			; Go back to chekl
				
__stop			B __stop
				ENDP