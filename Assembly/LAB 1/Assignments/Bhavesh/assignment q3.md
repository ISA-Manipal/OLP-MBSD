In the usual subtract operation,
      SUB R1, R2, operand2
The R1 assumes the value of R2 - operand2
Here, R2 must necessarily be a register and operand 2 can be either a register or an immediate value.

However, if need to subtract r2 from an immediate value
      MOV R3, #imm16
      SUB R1, R3, R2
Reverse subtraction solves this problem by allowing us to subtract R2 from operand 2
      RSB R1, R2, operand2
Thus the same problem of subtracting R2 from an immediate value can be completed using one command
