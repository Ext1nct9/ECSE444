#include "main.h"

// Substract data from original
float calculateDiff(float originalArray[], float calculatedArray[], float differenceArray[]){
	int len = sizeof(*calculatedArray/sizeof(calculatedArray[0]));
	for(int i = 0; i<len; i++){
		differenceArray[i] = originalArray[i] - calculatedArray[i];
	}
}
// Average Calculation
float calculateAvg(float differenceArray[]){
	int len = sizeof(*differenceArray/sizeof(differenceArray[0]));
	float sum = 0;
	for (int i = 0; i<len;i++){
		sum += differenceArray[i];
	}
	return sum/len;
}
// Standard Deviation calculation
float calculateStDev(float differenceArray[], float mean){
	int len = sizeof(*differenceArray/sizeof(differenceArray[0]));
	float sumSD = 0.0;
	for(int i =0; i<len;i++){
		sumSD += pow(differenceArray[i]-mean,2);
	}

	float var = sumSD/len;
	float SD = sqrt(var);

	return SD;
}
// Correlation between original and tracked vector
float calculateCorrelation(float originalArray[], float calculatedArray[]){
	float mean, mean2,sum1,sum2,sumProduct = 0.0;
	int lenc = sizeof(*calculatedArray/sizeof(calculatedArray[0]));
	for (int i =0; i<lenc;i++){
		mean += originalArray[i];
		mean2 += calculatedArray[i];
		sum1 += pow(originalArray[i], 2);
		sum2 += pow(calculatedArray[i],2);
		sumProduct += originalArray[i]*calculatedArray[i];
	}

	//mean /= lenc;
	//mean2 /= lenc;

	float numerator = lenc*sumProduct - mean*mean2;
	float denominator = sqrt((lenc*sum1-pow(mean,2))*(lenc*sum2-pow(mean2,2)));

	float correlation = numerator/denominator;

	return correlation;
}
// Convolution Between two vectors
void calculateConvolution(float originalArray[], float calculatedArray[], float resultArray[]){
	int len = sizeof(*calculatedArray/sizeof(calculatedArray[0]));
	for(int i = 0; i<2*len-1;i++){
		resultArray[i] = 0.0;
	}
	for(int i = 0; i<2*len-1;i++){
		for(int j = 0; j<len;j++){
			if (i-j >= 0 && i-j <len){
				resultArray[i] += originalArray[j]*calculatedArray[i-j];
			}
		}
	}
}

