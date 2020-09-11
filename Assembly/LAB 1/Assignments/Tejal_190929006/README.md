RSB is useful when you need to perform a shift operation on the first operand.

Example : RSB r0, r1, r2, LSL#2 is equivalent to r0 = (r2 left shifted by 2) - r1

However, in SUB, the same code(but with SUB instead of RSB) would give r0 = r1 - (r2 left shifted by 2)

The code SUB r0, r1, LSL#2, r2 gives an error