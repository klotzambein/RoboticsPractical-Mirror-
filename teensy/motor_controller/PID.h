#ifndef PID_
#define PID_

class PID
{

	float d_target;
	float d_kP;
	float d_kI;
	float d_kD;
	float d_prevError;
	float d_integral;

public:
	PID(float kP, float kI, float kD);
	float update(float current);
	void set_target(float target);
	void set_terms(float kP, float kI, float kD);
};

inline void PID::set_target(float target)
{
	d_target = target;
}

inline void PID::set_terms(float kP, float kI, float kD)
{
	d_kP = kP;
	d_kI = kI;
	d_kD = kD;
}

#endif
