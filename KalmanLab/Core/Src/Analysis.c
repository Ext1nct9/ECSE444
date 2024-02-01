#include "main.h"

// Substract data from original
float calculateDiff(float originalArray[], float calculatedArray[], float differenceArray[],float length){
	for(int i = 0; i<length; i++){
		differenceArray[i] = originalArray[i] - calculatedArray[i];
	}
}
// Average Calculation
float calculateAvg(float differenceArray[],float length){
	float sum = 0;
	for (int i = 0; i<length;i++){
		sum += differenceArray[i];
	}
	return sum/length;
}
// Standard Deviation calculation
float calculateStDev(float differenceArray[], float mean, float length){
	float sumSD = 0.0;
	for(int i =0; i<length;i++){
		sumSD += pow(differenceArray[i]-mean,2);
	}

	float var = sumSD/length;
	float SD = sqrt(var);

	return SD;
}
// Correlation between original and tracked vector
float calculateCorrelation(float originalArray[], float calculatedArray[], float length){
	float mean = 0.0;
    float mean2 = 0.0;
    float sum1 = 0.0;
    float sum2 = 0.0;
    float sumProduct = 0.0;
	for (int i =0; i<length;i++){
		mean += originalArray[i];
		mean2 += calculatedArray[i];
		sum1 += pow(originalArray[i], 2);
		sum2 += pow(calculatedArray[i],2);
		sumProduct += originalArray[i]*calculatedArray[i];
	}

	//mean /= lenc;
	//mean2 /= lenc;

	float numerator = length*sumProduct - mean*mean2;
	float denominator = sqrt((length*sum1-pow(mean,2))*(length*sum2-pow(mean2,2)));

	float correlation = numerator/denominator;

	return correlation;
}
// Convolution Between two vectors
void calculateConvolution(float originalArray[], float calculatedArray[], float resultArray[], float length){
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

