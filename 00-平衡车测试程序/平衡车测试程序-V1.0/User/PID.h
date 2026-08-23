#ifndef __PID_H
#define __PID_H

typedef struct {
	float Target;
	float Actual;
	float Actual1;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	
	float POut;
	float IOut;
	float DOut;
	
	float OutMax;
	float OutMin;
	
	float OutOffset;
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);

#endif
