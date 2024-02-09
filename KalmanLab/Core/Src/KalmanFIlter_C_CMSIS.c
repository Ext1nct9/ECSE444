#include "main.h"


// CMSIS
  int KalmanFilterCMSIS(float* InputArray, float* OutputArray, struct kalman_state * kstate, int length){
      	for (int i = 0; i<length; i++){ // Iterate through the array

      		arm_add_f32(&kstate->p, &kstate->q,&kstate->p,1); // p+q -> p
      		float add = 0;
      		arm_add_f32(&kstate->p,&kstate->r,&add,1); // p+r -> add
      		kstate->k = kstate->p/add; // p/add -> k
      		float mul = 0;
      		float sub = 0;
      		float constant = 1;
      		arm_sub_f32(&InputArray[i],&kstate->x,&sub,1); // input - x -> sub
      		arm_mult_f32(&sub,&kstate->k,&mul,1); // sub*k -> mul
      		arm_add_f32(&mul,&kstate->x,&kstate->x,1); // mul + x -> x
      		arm_sub_f32(&constant,&kstate->k,&sub,1); // 1 - k -> sub
      		arm_mult_f32(&kstate->p,&sub,&kstate->p,1); // p*sub -> p
      		OutputArray[i] = kstate->x; // Store x in the output array
      		int a = __get_FPSCR(); // Check for overflow
      		if ((a & 268435456) != 0){
      			printf("Overflow.");
      			while (1){}
      		}
      	}
      	return 0; // Return 0 if successful or get stuck in loop
   }
