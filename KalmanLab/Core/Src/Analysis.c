#include "main.h"
#include <math.h>
// Substract data from original
void calculateDiff(float originalArray[], float calculatedArray[], float differenceArray[],int length){
	for(int i = 0; i<length; i++){ // Iterate through the values in both arrays
		differenceArray[i] = originalArray[i] - calculatedArray[i]; // Calculate difference and store in output array
	}
}
// Average Calculation
float calculateAvg(float differenceArray[], int length){
	float sum = 0;
	for (int i = 0; i<length;i++){
		sum += differenceArray[i]; // Sum of the values in the array
	}
	return sum/length; // Return sum/length of the array
}
// Standard Deviation calculation
float calculateStDev(float differenceArray[], float mean, int length){
	float sumSD = 0.0;
	for(int i =0; i<length;i++){
		sumSD += pow(differenceArray[i]-mean,2); // Calculate the difference between the difference between the input and output values and the mean of the difference values, and square tthat value.
	}

	float var = sumSD/length; // Divide by the length

	return sqrt(var); // Return the square root of the result
}
// Correlation between original and tracked vector
void calculateCorrelation(float originalArray[], float calculatedArray[], float correlationArray[], int length){
	for(int i = 0; i<2*length-1;i++){
			correlationArray[i] = 0.0; // Set the values in the array to 0
	}
	float inputYReversed[length];
	for (int i =0; i<length;i++){
		inputYReversed[i] = calculatedArray[length-i-1]; // Reverse values of the calculated array
	}
	for (int i = 0;i<2*length-1;i++){
		for(int j = 0;j<length;j++){
			if (i-j >= 0 && i-j <length){
				correlationArray[i] += originalArray[j]*inputYReversed[i-j]; // Iterate through the output loop and add products of the possible pairs of values.
			}

		}
	}
}
// Convolution Between two vectors
void calculateConvolution(float originalArray[], float calculatedArray[], float resultArray[], int length){
	for(int i = 0; i<2*length-1;i++){
		resultArray[i] = 0.0; // Set array values to 0
	}
	for(int i = 0; i<2*length-1;i++){
		for(int j = 0; j<length;j++){
			if (i-j >= 0 && i-j <length){
				resultArray[i] += originalArray[j]*calculatedArray[i-j]; // Iterate and add product of pairs to the result array
			}
		}
	}
}

