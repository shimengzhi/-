#ifndef __HIGH_PROCESS__H
#define __HIGH_PROCESS__H

















typedef struct{
	float High;//�����ĸ߶�
	float Speed;//�߶�
	float  sutgbl;
}_st_Height;

extern _st_Height Height;


void Strapdown_INS_High(float high);
#endif



