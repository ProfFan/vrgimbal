/*************************/
/* RC-Decoder            */
/*************************/

#include "main.h"


// init RC config variables
void initRC() {
  rcLPF_tc = LOWPASS_K_FLOAT(config.rcLPF*0.1);
}

// pinChange Int driven Functions

void intDecodePWM_Ch0_rising();
void intDecodePWM_Ch0_falling();
void intDecodePWM_Ch1_rising();
void intDecodePWM_Ch1_falling();
void intDecodePWM_Ch2_rising();
void intDecodePWM_Ch2_falling();
void intDecodePWM_Ch3_rising();
void intDecodePWM_Ch3_falling();
void intDecodePWM(char chIdx, uint8 pinState);

void intDecodePWM_Ch0_rising()
{
	intDecodePWM(0, HIGH);
	detachInterrupt(BOARD_PWM_IN0);
	attachInterrupt(BOARD_PWM_IN0, &intDecodePWM_Ch0_falling, FALLING);
}
void intDecodePWM_Ch0_falling()
{
	intDecodePWM(0, LOW);
	detachInterrupt(BOARD_PWM_IN0);
	attachInterrupt(BOARD_PWM_IN0, &intDecodePWM_Ch0_rising, RISING);
}

void intDecodePWM_Ch1_rising()
{
	intDecodePWM(1, HIGH);
	detachInterrupt(BOARD_PWM_IN1);
	attachInterrupt(BOARD_PWM_IN1, &intDecodePWM_Ch1_falling, FALLING);
}
void intDecodePWM_Ch1_falling()
{
	intDecodePWM(1, LOW);
	detachInterrupt(BOARD_PWM_IN1);
	attachInterrupt(BOARD_PWM_IN1, &intDecodePWM_Ch1_rising, RISING);
}

void intDecodePWM_Ch2_rising()
{
	intDecodePWM(2, HIGH);
	detachInterrupt(BOARD_PWM_IN2);
	attachInterrupt(BOARD_PWM_IN2, &intDecodePWM_Ch2_falling, FALLING);
}
void intDecodePWM_Ch2_falling()
{
	intDecodePWM(2, LOW);
	detachInterrupt(BOARD_PWM_IN2);
	attachInterrupt(BOARD_PWM_IN2, &intDecodePWM_Ch2_rising, RISING);
}

void intDecodePWM_Ch3_rising()
{
	intDecodePWM(3, HIGH);
	detachInterrupt(BOARD_PWM_IN3);
	attachInterrupt(BOARD_PWM_IN3, &intDecodePWM_Ch3_falling, FALLING);
}
void intDecodePWM_Ch3_falling()
{
	intDecodePWM(3, LOW);
	detachInterrupt(BOARD_PWM_IN3);
	attachInterrupt(BOARD_PWM_IN3, &intDecodePWM_Ch3_rising, RISING);
}

// RC Channel 0
void intDecodePWM(char chIdx, uint8 pinState)
{ 
  uint32_t microsNow = micros();
  uint16_t pulseInPWMtmp;


  if (pinState==HIGH)
  {
    microsRisingEdge[chIdx]= microsNow;
  }
  else
  {
    pulseInPWMtmp = (microsNow - microsRisingEdge[chIdx])/CC_FACTOR;
    if ((pulseInPWMtmp >= MIN_RC) && (pulseInPWMtmp <= MAX_RC)) 
    {
      // update if within expected RC range
      rcRxChannel[chIdx] = pulseInPWMtmp;
      microsLastPWMUpdate[chIdx] = microsNow;
      validRC[chIdx]=true;
      updateRC[chIdx]=true;
    }
  }
}


// check for RC timout
void checkPWMTimeout(char channelNum)
{
#ifdef RC_USE_LIBS
	return;
#endif

  int32_t microsNow = micros();
  int32_t microsLastUpdate;
  cli();
  microsLastUpdate = microsLastPWMUpdate[channelNum];
  sei();
  if (((microsNow - microsLastUpdate)/CC_FACTOR) > RC_TIMEOUT) 
  {
    microsLastPWMUpdate[channelNum] = microsNow;
    rcRxChannel[channelNum] = config.rcMid;
    validRC[channelNum]=false;
    updateRC[channelNum]=true;
  }
}


//******************************************
// PPM Decoder
//******************************************
void intDecodePPM()
{ 
  int32_t microsNow = micros();
  
  static int32_t microsPPMLastEdge = 0;
  uint16_t pulseInPPM;

  static char channel_idx = 0;
  static bool rxPPMvalid = false;

  // PinChangeInt prolog = pin till here = 17 us 
  
  // 45us
  pulseInPPM = (microsNow - microsPPMLastEdge)/CC_FACTOR;
  microsPPMLastEdge = microsNow;

  if (pulseInPPM > RC_PPM_GUARD_TIME) 
  {
    // sync detected 
    channel_idx = 0;
    if (rxPPMvalid)
    {
      microsLastPPMupdate = microsNow;
    }
    rxPPMvalid = false;
  }
  else if (channel_idx < RC_PPM_RX_MAX_CHANNELS)
  {      
    if ((pulseInPPM >= MIN_RC) && (pulseInPPM <= MAX_RC)) 
    {
      rcRxChannel[channel_idx] = pulseInPPM;
      validRC[channel_idx] = true;
      updateRC[channel_idx] = true;
      channel_idx++;
    }  
    else
    {
      rxPPMvalid = false;
    }
  }
}

void checkPPMTimeout()
{
#ifdef RC_USE_LIBS
	return;
#endif
  int32_t microsNow = micros();
  int32_t microsLastUpdate;
  cli();
  microsLastUpdate = microsLastPPMupdate;
  sei();
  if (((microsNow - microsLastUpdate)/CC_FACTOR) > RC_TIMEOUT) 
  {
    for (char i=0; i<RC_PPM_RX_MAX_CHANNELS; i++)
    {
      rcRxChannel[i] = config.rcMid;
      validRC[i] = false;
      updateRC[i] = true;
    }
    microsLastPPMupdate = microsNow;
  }
}


#ifdef RC_USE_LIBS
void updateRCsignals()
{
	if (APM_RC.GetState() > 0){
		for (unsigned char channel_idx = 0; channel_idx < RC_PPM_RX_MAX_CHANNELS; channel_idx++)
		{
			rcRxChannel[channel_idx] = APM_RC.InputCh(channel_idx);
			validRC[channel_idx] = true;
			updateRC[channel_idx] = true;
		}
	}
}
#endif

// initialize RC Pin mode
void initRCPins()
{  

#ifdef RC_USE_LIBS
	if (config.rcModePPM)
	{
		APM_RC.Init(12, NULL, &Serial);
	} else {
		APM_RC.Init(2, NULL, &Serial);
	}
#else
  if (config.rcModePPM)
  {
    pinMode(BOARD_PWM_IN0, INPUT); digitalWrite(BOARD_PWM_IN0, HIGH);
    attachInterrupt(BOARD_PWM_IN0, &intDecodePPM, RISING);
  }
  else
  {
    pinMode(BOARD_PWM_IN0, INPUT);
    digitalWrite(BOARD_PWM_IN0, HIGH);
    attachInterrupt(BOARD_PWM_IN0, &intDecodePWM_Ch0_rising, RISING);

    pinMode(BOARD_PWM_IN1, INPUT);
    digitalWrite(BOARD_PWM_IN1, HIGH);
    attachInterrupt(BOARD_PWM_IN1, &intDecodePWM_Ch1_rising, RISING);

    pinMode(BOARD_PWM_IN2, INPUT);
    digitalWrite(BOARD_PWM_IN2, HIGH);
    attachInterrupt(BOARD_PWM_IN2, &intDecodePWM_Ch2_rising, RISING);

    pinMode(BOARD_PWM_IN3, INPUT);
    digitalWrite(BOARD_PWM_IN3, HIGH);
    attachInterrupt(BOARD_PWM_IN3, &intDecodePWM_Ch3_rising, RISING);
  }
#endif
}

// Proportional RC control
void evaluateRCSignalProportional()
{
#ifdef RC_USE_LIBS
	updateRCsignals();
#endif

	int16_t rxData;

	#define RCSTOP_ANGLE 5.0

	rxData = rcRxChannel[config.rcChannelPitch];
	if(updateRC[config.rcChannelPitch]==true)
	{
		if(rxData >= config.rcMid+RC_DEADBAND)
		{
			pitchRCSpeed = config.rcGain * (float)(rxData - (config.rcMid + RC_DEADBAND))/ (float)(MAX_RC - (config.rcMid + RC_DEADBAND)) + 0.9 * pitchRCSpeed;
		}
		else if(rxData <= config.rcMid-RC_DEADBAND)
		{
			pitchRCSpeed = -config.rcGain * (float)((config.rcMid - RC_DEADBAND) - rxData)/ (float)((config.rcMid - RC_DEADBAND)-MIN_RC) + 0.9 * pitchRCSpeed;
		}
		else
		{
			pitchRCSpeed = 0.0;
		}
		pitchRCSpeed = constrain(pitchRCSpeed, -2 * ANGLE_PRECISION, +2 * ANGLE_PRECISION);  // constrain for max speed
		updateRC[config.rcChannelPitch] = false;
	}

	rxData = rcRxChannel[config.rcChannelRoll];
	if(updateRC[config.rcChannelRoll]==true)
	{

		if(rxData >= config.rcMid+RC_DEADBAND)
		{
			rollRCSpeed = config.rcGain * (float)(rxData - (config.rcMid + RC_DEADBAND))/ (float)(MAX_RC - (config.rcMid + RC_DEADBAND)) + 0.9 * rollRCSpeed;
		}
		else if(rxData <= config.rcMid-RC_DEADBAND)
		{
			rollRCSpeed = -config.rcGain * (float)((config.rcMid - RC_DEADBAND) - rxData)/ (float)((config.rcMid - RC_DEADBAND)-MIN_RC) + 0.9 * rollRCSpeed;
		}
		else
		{
			rollRCSpeed = 0.0;
		}
		rollRCSpeed = constrain(rollRCSpeed, -2 * ANGLE_PRECISION, +2 * ANGLE_PRECISION);  // constrain for max speed
		updateRC[config.rcChannelRoll] = false;
	}

	rxData = rcRxChannel[config.rcChannelYaw];
	if(updateRC[config.rcChannelYaw]==true)
	{
		if(rxData >= config.rcMid+RC_DEADBAND)
		{
			yawRCSpeed = config.rcGain * (float)(rxData - (config.rcMid + RC_DEADBAND))/ (float)(MAX_RC - (config.rcMid + RC_DEADBAND)) + 0.9 * yawRCSpeed;
		}
		else if(rxData <= config.rcMid-RC_DEADBAND)
		{
			yawRCSpeed = -config.rcGain * (float)((config.rcMid - RC_DEADBAND) - rxData)/ (float)((config.rcMid - RC_DEADBAND)-MIN_RC) + 0.9 * yawRCSpeed;
		}
		else
		{
			yawRCSpeed = 0.0;
		}
		yawRCSpeed = constrain(yawRCSpeed, -2 * ANGLE_PRECISION, + 2 * ANGLE_PRECISION);  // constrain for max speed
		updateRC[config.rcChannelYaw] = false;
	}
}

// Absolute RC control
void evaluateRCSignalAbsolute()
{
#ifdef RC_USE_LIBS
	updateRCsignals();
#endif

    int16_t rxData;
    
  // Get Setpoint from RC-Channel if available.
  // LPF on pitchSetpoint
  rxData = rcRxChannel[config.rcChannelPitch];
  if(updateRC[config.rcChannelPitch]==true)
  {
    utilLP_float(&pitchRCSetpoint, (config.minRCPitch + (float)(rxData - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCPitch - config.minRCPitch)), 0.05);
    updateRC[config.rcChannelPitch] = false;
  }

  // LPF on rollSetpoint
  rxData = rcRxChannel[config.rcChannelRoll];
  if(updateRC[config.rcChannelRoll]==true)
  {
    utilLP_float(&rollRCSetpoint, (float)(config.minRCRoll + (float)(rxData - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCRoll - config.minRCRoll)), 0.05);
    updateRC[config.rcChannelRoll] = false;
  }

  // LPF on yawSetpoint
  rxData = rcRxChannel[config.rcChannelYaw];
  if(updateRC[config.rcChannelYaw]==true)
  {
    utilLP_float(&yawRCSetpoint, (float)(config.minRCYaw + (float)(rxData - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCYaw - config.minRCYaw)), 0.05);
    updateRC[config.rcChannelYaw] = false;
  }
}

















