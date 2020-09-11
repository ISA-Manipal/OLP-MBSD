				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   
				AREA    main, CODE, READONLY
				EXPORT	__main	      
				ENTRY
				
				; Problem:
				; Suresh has a habit of reverse subtraction, one day he went to retailer store to buy a packet of milk for rupees 18/- and chocolate for rupees 20/-.
				; He gave rupees 100/- to the shopkeeper and asked for rupees 3536/-.
				; Check in debugging whether Suresh's calculations were correct or wrong.
				
__main			PROC
				
				MOV r0, #18			; MILK
				MOV r1, #20			; CHOCOLATE
				ADD r2, r1, r0		; NET EXPENDITURE
				MOV r3, #100		; MONEY GIVEN TO SHOPKEEPER
				
				RSB r4, r3, r2		; REVERSE SUBTRACTION:- EXPENDITURE - MONEY GIVEN
				RSB r5, r4, #0x0	; NEGATING THE RESULT
								
				; The result is stored in R5 is the value that the shopkeeper has to pay back
				; The value is 3E in Hex which is 62 in decimal
				; Suresh should be demanding 62 not 3536
				
__stop			B __stop
				ENDP