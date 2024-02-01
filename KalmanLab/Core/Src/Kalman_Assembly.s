.section .text
.syntax unified
.global kalman


// states order: q, r, x, p, k
kalman:
    VPUSH.32 {s4, s5, s6, s7}
    PUSH {r4}

    // calculate p = p + q
    VLDR.32 s4, [r0, #12]     // load p in s4
    VLDR.32 s5, [r0]         // load q in S5
    VADD.F32 s4, s4, s5        // calculate p = p + q, result in s4
    VSTR.32 s4, [r0, #12]     // save new p in struct

    // calculate k = p / (p + r)
    VLDR.32 s5, [r0, #4]     // load r in s5, new p already in s4
    VADD.F32 s5, s4, s5        // calculate p + r, result in s5
    VCMP.F32 s5, #0.0		// check division by zero
    BEQ division_by_zero
    VDIV.F32 s5, s4, s5        // calculate k = p / (p+r) , result in s5
    VSTR.32 s5, [r0, #16]    // save new k in struct

	// p in S4   k in S5
    // calculate x = x + k * (meas - x)
    VLDR.32 s6, [r0, #8]    // load x in s6
    VSUB.F32 s7, s0, s6		// calculate meas - x, result in s7
    VMLA.F32 s6, s5, s7		// calculate x, result in s6
    VSTR.32 s6, [r0, #8]	// save new x in struct

    // calculate p = (1 - k) * p
    VMOV.F32 s7, #1.0   // load 1.0 in s7
    VSUB.F32 s5, s7, s5		// calculate 1 - k, result in s5
    VMUL.F32 s4, s5, s4		// calculate new p, result in s4
    VSTR.32 s4, [r0, #12]    // save new p in struct

    PUSH {r4}
    VMRS r4, FPSCR
    TST r4, #0x10000000	    // check for overflow
    BEQ overflow
    
    MOV R0, #0     // no errors
    B end_kalman

// error handling
// return 2 if overflow, return 1 if division by zero
overflow:
    MOV R0, #2
    B end_kalman
division_by_zero:
    MOV R0, #1

end_kalman:
    POP {r4}
    VPOP.32 {s4, s5, s6, s7}
    BX LR
