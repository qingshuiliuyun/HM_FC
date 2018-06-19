


#include "Fixedwing.h"
#include "parameter.h"
#include "include.h"

_fixedwingctrl fixedwing;


void FixedWing_ThrottleCtrl(float T,float pThr,float CThr)
{
	
}

void Fixedwing_PitchCtrl(float T,float pPitch,float pPitchRate,float CPitch)
{
	  fixedwing.pitch.error    = pPitch - CPitch;
	
	  fixedwing.pitch.error_i += Parameter_PID.FixedWing.Angle.Pitch.ki * fixedwing.pitch.error * T;
	  fixedwing.pitch.error_i  = LIMIT(fixedwing.pitch.error_i,-6,+6);
	
	  fixedwing.pitch.error_d  = -pPitchRate * Parameter_PID.FixedWing.Angle.Pitch.kd;	
	
	  fixedwing.pitch.out      = fixedwing.pitch.error * Parameter_PID.FixedWing.Angle.Pitch.kp + fixedwing.pitch.error_i + fixedwing.pitch.error_d;
    fixedwing.pitch.out  = LIMIT(fixedwing.pitch.out,-30,+30);
}

void Fixedwing_RollCtrl(float T,float pRoll,float pRollRate,float CRoll)
{
	  fixedwing.roll.error    = pRoll - CRoll;
	
	  fixedwing.roll.error_i += Parameter_PID.FixedWing.Angle.Roll.ki * fixedwing.roll.error * T;
	  fixedwing.roll.error_i  = LIMIT(fixedwing.roll.error_i,-6,+6);
	
	  fixedwing.roll.error_d  = -pRollRate * Parameter_PID.FixedWing.Angle.Roll.kd;	
	
	  fixedwing.roll.out      = fixedwing.roll.error * Parameter_PID.FixedWing.Angle.Roll.kp + fixedwing.roll.error_i + fixedwing.roll.error_d;
    fixedwing.roll.out  = LIMIT(fixedwing.roll.out,-30,+30);
}

void Fixedwing_HeadingCtrl(float T,float pHeading,float pHeadingRate,float CHeading)
{
	  fixedwing.heading.error    = pHeading - CHeading;
	
	  fixedwing.heading.error_i += Parameter_PID.FixedWing.Angle.Yaw.ki * fixedwing.heading.error * T;
	  fixedwing.heading.error_i  = LIMIT(fixedwing.heading.error_i,-6,+6);
	
	
	  fixedwing.heading.error_d  = -pHeadingRate * Parameter_PID.FixedWing.Angle.Yaw.kd;	
	
	  fixedwing.heading.out      = fixedwing.heading.error * Parameter_PID.FixedWing.Angle.Yaw.kp + fixedwing.heading.error_i + fixedwing.heading.error_d;
    fixedwing.heading.out  = LIMIT(fixedwing.heading.out,-30,+30);
}







