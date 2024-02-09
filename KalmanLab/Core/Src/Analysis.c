#include "main.h"
#include <math.h>
// Substract data from original
void calculateDiff(float originalArray[], float calculatedArray[], float differenceArray[],int length){
	for(int i = 0; i<length; i++){
		differenceArray[i] = originalArray[i] - calculatedArray[i];
	}
}
// Average Calculation
float calculateAvg(float differenceArray[], int length){
	float sum = 0;
	for (int i = 0; i<length;i++){
		sum += differenceArray[i];
	}
	return sum/length;
}
// Standard Deviation calculation
float calculateStDev(float differenceArray[], float mean, int length){
	float sumSD = 0.0;
	for(int i =0; i<length;i++){
		sumSD += pow(differenceArray[i]-mean,2);
	}

	float var = sumSD/length;

	return sqrt(var);

}
// Correlation between original and tracked vector
void calculateCorrelation(float originalArray[], float calculatedArray[], float correlationArray[], int length){
	for(int i = 0; i<2*length-1;i++){
			correlationArray[i] = 0.0;
	}
	float inputYReversed[length];
	for (int i =0; i<length;i++){
		inputYReversed[i] = calculatedArray[length-i-1];
	}
	for (int i = 0;i<2*length-1;i++){
		for(int j = 0;j<length;j++){
			if (i-j >= 0 && i-j <length){
				correlationArray[i] += originalArray[j]*inputYReversed[i-j];
			}

		}
	}
}
// Convolution Between two vectors
void calculateConvolution(float originalArray[], float calculatedArray[], float resultArray[], int length){
	for(int i = 0; i<2*length-1;i++){
		resultArray[i] = 0.0;
	}
	for(int i = 0; i<2*length-1;i++){
		for(int j = 0; j<length;j++){
			if (i-j >= 0 && i-j <length){
				resultArray[i] += originalArray[j]*calculatedArray[i-j];
			}
		}
	}
}

