#ifndef __EKH_INTERFACE_H
#define __EKH_INTERFACE_H

#include "dfc_types.h"
#include "data_types.h"
#include "utils.h"
#include "math.h"

namespace ekh {
    class ekh_interface {


#include "common.h"
#include "RingBuffer.h"
#include "imu_down_sampler/imu_down_sampler.hpp"
#include "output_predictor/output_predictor.h"
#include <lib/atmosphere/atmosphere.h>
#include <lib/lat_lon_alt/lat_lon_alt.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/AlphaFilter.hpp>

using namespace estimator;

class EstimatorInterface
{
public:
	void setIMUData(const imuSample &imu_sample);
	void setMagData(const magSample &mag_sample);
	void setBaroData(const baroSample &baro_sample);

#ifdef OFLOW_IN_USE
	void setOpticalFlowData(const flowSample &flow);
#endif // OFLOW_IN_USE


	void setSystemFlagData(const systemFlagUpdate &system_flags);

	// return a address to the parameters struct
	// in order to give access to the application
	parameters *getParamHandle() { return &_params; }

	// set vehicle landed status data
	void set_in_air_status(bool in_air)
	{
		if (!in_air) {
			if (_control_status.flags.in_air) {
				ECL_DEBUG("no longer in air");
			}

			_time_last_on_ground_us = _time_delayed_us;

		} else {
			if (!_control_status.flags.in_air) {
				ECL_DEBUG("in air");
			}

			_time_last_in_air = _time_delayed_us;
		}

		_control_status.flags.in_air = in_air;
	}

	void set_vehicle_at_rest(bool at_rest)
	{
		if (!_control_status.flags.vehicle_at_rest && at_rest) {
			ECL_DEBUG("at rest");

		} else if (_control_status.flags.vehicle_at_rest && !at_rest) {
			ECL_DEBUG("no longer at rest");
		}

		_control_status.flags.vehicle_at_rest = at_rest;
	}

	void set_constant_pos(bool constant_pos) { _control_status.flags.constant_pos = constant_pos; }

	// return true if the attitude is usable
	bool attitude_valid() const { return _control_status.flags.tilt_align; }

	// get vehicle landed status data
	bool get_in_air_status() const { return _control_status.flags.in_air; }

	// set flag if static pressure rise due to ground effect is expected
	// use _params.gnd_effect_deadzone to adjust for expected rise in static pressure
	// flag will clear after GNDEFFECT_TIMEOUT uSec
	void set_gnd_effect()
	{
		_control_status.flags.gnd_effect = true;
		_time_last_gnd_effect_on = _time_delayed_us;
	}

	// set air density used by the multi-rotor specific drag force fusion
	void set_air_density(float air_density) { _air_density = air_density; }

	const matrix::Quatf &getQuaternion() const { return _output_predictor.getQuaternion(); }
	Vector3f getAngularVelocityAndResetAccumulator() { return _output_predictor.getAngularVelocityAndResetAccumulator(); }
	float getUnaidedYaw() const { return _output_predictor.getUnaidedYaw(); }
	Vector3f getVelocity() const { return _output_predictor.getVelocity(); }

	// get the mean velocity derivative in earth frame since last reset (see `resetVelocityDerivativeAccumulation()`)
	Vector3f getVelocityDerivative() const { return _output_predictor.getVelocityDerivative(); }
	void resetVelocityDerivativeAccumulation() { return _output_predictor.resetVelocityDerivativeAccumulation(); }
	float getVerticalPositionDerivative() const { return _output_predictor.getVerticalPositionDerivative(); }
	Vector3f getPosition() const;


	// Getter for the average EKF update period in s
	float get_dt_ekf_avg() const { return _dt_ekf_avg; }


protected:

	EstimatorInterface() = default;
	virtual ~EstimatorInterface();

	virtual bool init(uint64_t timestamp) = 0;

	parameters _params{};		// filter parameters


	float _dt_ekf_avg{0.010f}; ///< average update rate of the ekf in s


#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	RingBuffer<flowSample> 	*_flow_buffer {nullptr};
	flowSample _flow_sample_delayed{};
#endif // CONFIG_EKF2_OPTICAL_FLOW


// data buffer instances
	static constexpr uint8_t kBufferLengthDefault = 12;
	RingBuffer<imuSample> _imu_buffer{kBufferLengthDefault};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	RingBuffer<magSample> *_mag_buffer {nullptr};
	uint64_t _time_last_mag_buffer_push{0};
#endif // CONFIG_EKF2_MAGNETOMETER


#if defined(CONFIG_EKF2_BAROMETER)
	RingBuffer<baroSample> *_baro_buffer {nullptr};
	uint64_t _time_last_baro_buffer_push{0};
#endif // CONFIG_EKF2_BAROMETER

	uint64_t _time_last_gnd_effect_on{0};

};
#endif // !EKF_ESTIMATOR_INTERFACE_H
    };
}   // namespace ekh

#endif
