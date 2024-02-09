#include "main.h"


//C
  int KalmanFilterC(float* InputArray, float* OutputArray, struct kalman_state * kstate, int length){
    	for (int i = 0; i<length; i++){ // Iterate

    		kstate->p = kstate->p + kstate->q;
    		kstate->k = kstate->p/(kstate->p + kstate->r);
    		kstate->x = kstate->x + (kstate->k)*(InputArray[i]-kstate->x);
    		kstate->p = (1-kstate->k)*kstate->p;
    		OutputArray[i] = kstate->x; // Store in output array
    		int a = __get_FPSCR();
    		if (a & 268435456 != 0){ // Check for overflow
    			printf("Overflow.");
    			while (1){}
    		}
    	}
    	return 0; // Return 0 if successful or get stuck in while loop
    }
