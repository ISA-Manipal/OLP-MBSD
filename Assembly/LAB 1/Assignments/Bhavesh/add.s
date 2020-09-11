				INCLUDE MSP432xx_constants.s          
				INCLUDE MSP432xx_tim_constants.s   

				AREA main, CODE, READONLY
				EXPORT __main
__main			PROC
				
				;INTEGER 1
				MOV r0, #0x12	;MILK
				MOV r1, #0x14	;CHOCOLATE
				ADD r2, r1, r0	;NET EXPENDITURE
				MOV r3, #0x64	;MONEY GIVEN TO SHOPKEEPER
				
				RSB r4, r3, r2		;BALANCE BY REVERSE SUBTRACTION:- EXPENDITURE - MONEY GIVEN
				RSB r5, r4, #0x0	;CONVERT THE VALUE TO POSITIVE
				
				;The final value stored in R5 is the value that the shopkeeper has to pay back
				;The value is 3C in Hex which is 62 in decimal
				;Thus the money to be payed back is Rs.62 and not Rs.3536 as it was demanded by Suresh
			
__stop			B __stop

				ENDP