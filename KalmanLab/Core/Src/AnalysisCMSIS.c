#include "main.h"

// Substract data from original
void calculateDiffCMSIS(float* originalArray[], float* calculatedArray[], float* differenceArray[],uint32_t length){
		arm_sub_f32(originalArray,calculatedArray, differenceArray,length);
}

// Average Calculation
float calculateAvgCMSIS(float* differenceArray[],uint32_t length){
	float output;
	arm_mean_f32(differenceArray, length, &output);
	return output;
}
// Standard Deviation calculation
float calculateStDevCMSIS(float* differenceArray[],uint32_t length){
	float output;
	arm_std_f32(differenceArray, length, &output);
	return output;

}
// Correlation between original and tracked vector
void calculateCorrelationCMSIS(float originalArray[], float calculatedArray[], float correlationArray[], int length){
	arm_correlate_f32(originalArray, length, calculatedArray, length, correlationArray);
}
// Convolution Between two vectors
void calculateConvolutionCMSIS(float originalArray[], float calculatedArray[], float resultArray[], int length){
	arm_conv_f32(originalArray, length, calculatedArray, length, resultArray);
}
