
#include "steps.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define STEP_N1 (50)
#define STEP_N2 (25*10)
#define STEP_N3 (300)

#define S1 (0x70)	//判断阈值，默认值适用于机测75rpm-180rpm
#define S2 (0x30)
#define S3 (0x70)
#define S4 (0x30)

static int16_t vSTEP_N1 = STEP_N1, vSTEP_N2 = STEP_N2, vSTEP_N3 = STEP_N3;
static int16_t vcS1 = S1, vcS2 = S2, vcS3 = S3, vcS4 = S4;

int16_t _setTimeout(int16_t n1, int16_t n2, int16_t n3)
{
	vSTEP_N1 = n1;
	vSTEP_N2 = n2;
	vSTEP_N3 = n3;
	return 0;
}

int16_t _setThreshold(int16_t s1, int16_t s2, int16_t s3, int16_t s4)
{
	vcS1 = s1;
	vcS2 = s2;
	vcS3 = s3;
	vcS4 = s4;
	return 0;
}

#define STEP_TIMEOUT1 (8)	//超时时间，应当设置约为每秒采样数的1/3
#define STEP_TIMEOUT2 (6)	//应当设置为略小于TIMEOUT1
// #define STEP_TIMEOUT3

#define MAXACC (32767)
#define MINACC (-32767)

#define DETECTRISE1 (1)
#define DETECTRISE2 (2)
#define DETECTFALL1 (3)
#define DETECTFALL2 (4)
static const int8_t avrIntervalPass_A[] = {0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -6, -7, -8, -9, -10};
static const int8_t avrIntervalPass_B[] = {0, 0, 0, 0, 0, 0, 2, 2, 3, 4, 4, 4, 4, 5, 6, 5, 5, 5, 5, 4, 3, 3, 2, 1,  0};
int8_t calcStep(int16_t* sAxis, uint8_t ctr)	//ctr=0 normal; ctr=1 9steps; ctr=2 sleepClear
{
	static int16_t sSleepAxis[3] = {0, 0, 0};
	static int16_t minAcceleration = MAXACC, maxAcceleration = MINACC, compareAcc;
	static uint8_t stepCounter = 0;
	static uint16_t timeCounter = 0, sleepCounter = 0;
	static int8_t stepInterval = 0, avrInterval = 0;
	static uint16_t vS1 = S1, vS2 = S2, vS3 = S3, vS4 = S4;

	static int16_t sAxisSqEx[4] = {0, 0, 0, 0}, inA = 0;
	static float K = 0.8;

	static uint8_t stepFlag = DETECTRISE1;	//DETECTRISE1,DETECTRISE2,DETECTFALL1,DETECTFALL2
	static uint8_t count = 0;

	register int16_t temp0;
	// int16_t temp1, temp2;
	int16_t* psAxis = sAxis, *psSleepAxis = sSleepAxis;
	// uint8_t deltaA;	//与上次采样相比三轴的变化量之和
	int8_t output = 0;
	int16_t sAxisSq = 0;

	if(ctr == 2) {
		sleepCounter = 0;
		timeCounter = 0;
		stepFlag = DETECTRISE1;
	}

	stepInterval++;
	psAxis = sAxis;
	temp0 = (*psAxis) * (*psAxis);
	psAxis++;
	temp0 += (*psAxis) * (*psAxis);
	psAxis++;
	temp0 += (*psAxis) * (*psAxis);

	inA = inA * 0.75 + temp0 * 0.25;

	sAxisSqEx[count] = K * (2 * sAxisSqEx[(count + 3) & 0x3] - sAxisSqEx[(count + 2) & 0x3]) + (1 - K) * (inA);

	sAxisSq = (sAxisSqEx[0] / 4 + sAxisSqEx[1] / 4 + sAxisSqEx[2] / 4 + sAxisSqEx[3] / 4);

	count = (count + 1) & 0x3;

	switch(stepFlag) {
		case DETECTRISE1:
			if(minAcceleration > sAxisSq) {
				minAcceleration = sAxisSq;
			} else {
				timeCounter++;
				if(sAxisSq - minAcceleration > vS1) {
					compareAcc = sAxisSq;
					stepFlag = DETECTRISE2;
				}
			}
			break;

		case DETECTRISE2:
			if(sAxisSq - compareAcc > vS2) {
				stepFlag = DETECTFALL1;
				maxAcceleration = MINACC;
				timeCounter = 0;
			} else {
				if(timeCounter++ > STEP_TIMEOUT1) {
					stepFlag = DETECTRISE1;
					minAcceleration = MAXACC;
				}
			}
			break;

		case DETECTFALL1:
			if(sAxisSq > maxAcceleration) {
				maxAcceleration = sAxisSq;
			} else {
				if(sAxisSq < maxAcceleration - vS3) {
					compareAcc = sAxisSq;
					stepFlag = DETECTFALL2;
					timeCounter = 0;
				} else {
					if(timeCounter++ > STEP_TIMEOUT1) {
						stepFlag = DETECTRISE1;
						minAcceleration = MAXACC;
					}
				}
			}
			break;

		case DETECTFALL2:
			if(sAxisSq < compareAcc - vS4) {	//step count 1 !!
				stepFlag = DETECTRISE1;
				minAcceleration = MAXACC;
				timeCounter = 0;
				sleepCounter = 0;
				if(ctr == 1) {
					stepCounter++;

					if(stepCounter == 3) {
						avrInterval = stepInterval;

					} else if(stepCounter > 3) {
						temp0 = stepInterval - avrInterval;

						if(avrInterval < 6 || avrInterval > 24) {
							if(stepCounter > 0) {
								stepCounter -= 1;
							}
						} else {
							if((temp0 < avrIntervalPass_A[avrInterval]) || (temp0 > avrIntervalPass_B[avrInterval])) {
								stepCounter = 0;
							}
						}

						if(stepInterval < 6 || stepInterval > 24) {
							stepCounter = 0;    //band pass filter
						}
					}

					if(stepCounter > 0) {
						if(stepInterval <= 8) {
							K = 0.6;
						} else if(stepInterval <= 10) {
							K = 0.7;
						} else if(stepInterval <= 12) {
							K = 0.75;
						} else {
							K = 0.8;
						}
					}

					if(stepCounter >= 5) {
						stepCounter = 0;
						output = 5;
					}
				} else if(ctr == 0) {
					if(stepInterval >= 6) {
						output = 1;
						if(stepInterval <= 8) {
							K = 0.6;
						} else if(stepInterval <= 10) {
							K = 0.7;
						} else if(stepInterval <= 12) {
							K = 0.75;
						} else {
							K = 0.8;
						}
					}
				}
				// avrInterval=avrInterval*0.75+stepInterval*0.25;
				avrInterval = stepInterval;
				if(avrInterval >= 15) {
					vS1 = vcS1, vS2 = vcS2, vS3 = vcS3, vS4 = vcS4;   //<80/min
				} else if(avrInterval >= 12) {
					vS1 = vcS1 + 0x18, vS2 = vcS2 + 0x18, vS3 = vcS3 + 0x18, vS4 = vcS4 + 0x18;   //<100/min
				} else if(avrInterval >= 10) {
					vS1 = vcS1 + 0x38, vS2 = vcS2 + 0x28, vS3 = vcS3 + 0x38, vS4 = vcS4 + 0x28;   //<120/min
				} else if(avrInterval >= 8) {
					vS1 = vcS1 + 0x68, vS2 = vcS2 + 0x60, vS3 = vcS3 + 0x68, vS4 = vcS4 + 0x60;   //<140/min
				} else {
					vS1 = vcS1 + 0xA0, vS2 = vcS2 + 0x90, vS3 = vcS3 + 0xA0, vS4 = vcS4 + 0x90;
				}

				stepInterval = 0;
			} else {
				if(timeCounter++ > STEP_TIMEOUT2) {
					stepFlag = DETECTRISE1;
					minAcceleration = MAXACC;
				}
			}
			break;

		default:
			break;
	}

	sleepCounter++;
	if(sleepCounter == vSTEP_N1) {
		vS1 = vcS1, vS2 = vcS2, vS3 = vcS3, vS4 = vcS4;
		stepFlag = DETECTRISE1;
		minAcceleration = 0xff;
		stepCounter = 0;
		output = -1;
		K = 0.8;
	}
	if(timeCounter > vSTEP_N3 || sleepCounter > vSTEP_N2) {
		output = -2;
		K = 0.8;
	}

	return output;
}
