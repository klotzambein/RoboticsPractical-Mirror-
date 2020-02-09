#include "PID.h"

//Constructor
PID::PID(float kP, float kI, float kD)
:
	d_target(0),
	d_kP(kP),
	d_kI(kI),
	d_kD(kD),
	d_prevError(0),
	d_integral(0)
{
}
