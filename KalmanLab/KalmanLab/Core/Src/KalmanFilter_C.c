#include "main.h"


//C
  int KalmanFilterC(float* InputArray, float* OutputArray, struct kalman_state * kstate, int length){
    	for (int i = 0; i<length; i++){

    		kstate->p = kstate->p + kstate->q;
    		kstate->k = kstate->p/(kstate->p + kstate->r);
    		kstate->x = kstate->x + (kstate->k)*(InputArray[i]-kstate->x);
    		kstate->p = (1-kstate->k)*kstate->p;
    		OutputArray[i] = kstate->x;
    	}
    	return 0;
    }
