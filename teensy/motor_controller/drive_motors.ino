// Students
//Given left and right throttle make sure the motor driver pins are configured correctly
// and the pwm pins are provided with the correct values.
void drive_motors(float left_throttle, float right_throttle)
{
  if(left_throttle >= 0)
  {
    digitalWrite(STBYA, HIGH);
    digitalWrite(APHASE, LOW);
  }else if (left_throttle < 0)
  {
    digitalWrite(STBYA, HIGH);
    digitalWrite(APHASE, HIGH);
  }
  if(right_throttle >= 0)
  {
    digitalWrite(STBYB, HIGH);
    digitalWrite(BPHASE, LOW);
  }else if (right_throttle < 0)
  {
    digitalWrite(STBYB, HIGH);
    digitalWrite(BPHASE, HIGH);
  }
  analogWrite(PWMA,min(abs(left_throttle),4096));
  analogWrite(PWMB,min(abs(right_throttle),4096));
}
