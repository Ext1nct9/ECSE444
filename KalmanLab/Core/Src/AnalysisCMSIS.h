void calculateDiffCMSIS(float* originalArray[], float* calculatedArray[], float* differenceArray[],uint32_t length);

float calculateAvgCMSIS(float* differenceArray[], uint32_t length);

float calculateStDevCMSIS(float* differenceArray[], uint32_t length);

void calculateCorrelationCMSIS(float* originalArray[], float* calculatedArray[], float* correlationArray[],int length);

void calculateConvolutionCMSIS(float* originalArray[], float* calculatedArray[], float* resultArray[], int length);
