           AREA arm, CODE, READONLY
	     EXPORT __main
	     ENTRY
	
__main   
	     LDR R1,=dst
	     LDR R2,=src
	     MOV R5, #65

check    

           LDRB R3, [R2], #1
	     CMP R3, R5
	     BEQ store
           B check
		 
store      STRB R6, [R1], #1
           B check


	     ALIGN
		
	     AREA Mydata, DATA, READONLY
src          DCB "BANANA", 0
	
	     AREA Copy, DATA, READWRITE
dst          SPACE 20	
	
		
		
	     END
