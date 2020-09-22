		AREA main,CODE,READONLY	
		EXPORT __main
		ENTRY
__main	PROC
	
		MOV R7,#0x0
		MOV R0,#0x1
		MOV R3,#0x1
		MOV R4,#0x0
		MOV R5,#0x1
		AND R4,R5
		AND R3,R4
		AND R0,R3
		ORR R7,R0
		
		ENDP
			
		ALIGN
		AREA allocations, DATA, READWRITE
		END
		
		
		
		