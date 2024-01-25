.section .text
.syntax unified
.global kalman


kalman:
    VPUSH.32 {s4, s5, s6}

    // calculate p
    VLDR.32 s4, [r0, #12]     // load p in s4
    VLDR.32 s5, [r0]         // load q in S5
    VADD.F32 s4, s4, s5        // calculate p = p + q, result in s4
    VSTR.32 s4, [r0, #12]     // save new p in struct

    // calculate k
    VLDR.32 s5, [r0, #4]     // load r in s5, new p already in s4
    VADD.F32 s5, s4, s5        // calculate p + r, result in s5
    VDIV.F32 s5, s4, s5        // calculate k = p / (p+r) , result in s5
    VSTR.32 s5, [r0, #16]    // save new k in struct

    // calculate x
    VLDR.32 s4, [r0, #8]    // load x in s4
    VSUB.F32 s5, s0, s4        // calculate measurement - x, result in s5
    VLDR.32 s6, [r0, #16]    // load k in s6
    VMUL.F32 s5, s6, s5        // calculate k * (measurement-x), result in s5
    VADD.F32 s4, s4, s5        // calculate x + s5, result in s4
    VSTR.32 s4, [r0, #8]    // save new x in struct

    // calculate p again

    VMOV.F32 S4, #1.0   // load 1.0 in s4
    VLDR.32 s5, [r0, #16]   // load k in s5
    VSUB.F32 s5, s4, s5        // 1.0 - k, result in s5
    VLDR.32 s4, [r0, #12]     // load p in s4
    VMUL.F32 s4, s5, s4        // p = (1-k) * p, result in s4
    VSTR.32 s4, [r0, #12]    // save new p in struct

    VPOP.32 {s4, s5, s6}
    BX LR
