// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <math.h>
#include "AC_PID.h"

const AP_Param::GroupInfo AC_PID::var_info[] PROGMEM = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_PID, _kp, 0),
    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_PID, _ki, 0),
    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    2, AC_PID, _kd, 0),
    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 3, AC_PID, _imax, 0),
    AP_GROUPEND
};

int32_t AC_PID::get_p(int32_t error)
{
	return (int32_t)((float)error * _kp);
}

int32_t AC_PID::get_i(int32_t error, float dt)
{
	if(((_ki < 0.0) || (_ki > 0.0)) && ((dt < 0.0) || (dt > 0.0))){
		_integrator += ((float)error) * _ki * dt;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		return (int32_t)(_integrator);
	}
	return 0;
}

// This is an integrator which tends to decay to zero naturally
// if the error is zero.

int32_t AC_PID::get_leaky_i(int32_t error, float dt, float leak_rate)
{
	if(((_ki < 0.0) || (_ki > 0.0)) && ((dt < 0.0) || (dt > 0.0))){
		_integrator -= (float)_integrator * leak_rate;
		_integrator += ((float)error * _ki) * dt;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}

		return (int32_t)(_integrator);
	}
	return 0;
}

int32_t AC_PID::get_d(int32_t input, float dt)
{
	if (((_kd < 0.0) || (_kd > 0.0)) && ((dt < 0.0) || (dt > 0.0))) {
        float derivative;
		if (isnan(_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			_last_derivative = 0;
		} else {
			// calculate instantaneous derivative
		derivative = (float)(input - _last_input) / dt;
		}

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
        derivative = _last_derivative +
                      (dt / ( _filter + dt)) * (derivative - _last_derivative);

        // update state
        _last_input             = input;
        _last_derivative    = derivative;

		// add in derivative component
		return (int32_t)(_kd * derivative);
	}
	return 0;
}

int32_t AC_PID::get_pi(int32_t error, float dt)
{
	return get_p(error) + get_i(error, dt);
}


int32_t AC_PID::get_pid(int32_t error, float dt)
{
	return get_p(error) + get_i(error, dt) + get_d(error, dt);
}



/*
int32_t AC_PID::get_pid(int32_t error, float dt)
{
	// Compute proportional component
	_output = error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabs(_kd) > 0) && (dt > 0)) {
		_derivative = (error - _last_error) / dt;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = _last_derivative +
		        (dt / ( _filter + dt)) * (_derivative - _last_derivative);

		// update state
		_last_error 		= error;
		_last_derivative    = _derivative;

		// add in derivative component
		_output 	+= _kd * _derivative;
	}

	// Compute integral component if time has elapsed
	if ((fabs(_ki) > 0) && (dt > 0)) {
		_integrator 		+= (error * _ki) * dt;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		_output 	+= _integrator;
	}

	return _output;
}
*/


void
AC_PID::reset_I()
{
    _integrator = 0;
	// mark derivative as invalid
    _last_derivative = NAN;
}

void
AC_PID::load_gains()
{
	_kp.load();
	_ki.load();
	_kd.load();
	_imax.load();
    //_imax = abs(_imax);
}

void
AC_PID::save_gains()
{
	_kp.save();
	_ki.save();
	_kd.save();
	_imax.save();
}
