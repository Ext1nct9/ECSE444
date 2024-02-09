#include "main.h"


// CMSIS
  int KalmanFilterCMSIS(float* InputArray, float* OutputArray, struct kalman_state * kstate, int length){
      	for (int i = 0; i<length; i++){

      		arm_add_f32(&kstate->p, &kstate->q,&kstate->p,1);
      		float add = 0;
      		arm_add_f32(&kstate->p,&kstate->r,&add,1);
      		kstate->k = kstate->p/add;
      		float mul = 0;
      		float sub = 0;
      		float constant = 1;
      		arm_sub_f32(&InputArray[i],&kstate->x,&sub,1);
      		arm_mult_f32(&sub,&kstate->k,&mul,1);
      		arm_add_f32(&mul,&kstate->x,&kstate->x,1);
      		arm_sub_f32(&constant,&kstate->k,&sub,1);
      		arm_mult_f32(&kstate->p,&sub,&kstate->p,1);
      		OutputArray[i] = kstate->x;
      		int a = __get_FPSCR();
      		if ((a & 268435456) != 0){
      			printf("Overflow.");
      			while (1){}
      		}
      	}
      	return 0;
   }
