#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

class PHState
{
	public:
		PHState(int32_t id)
			: id_(id)
		{
		}

		int32_t  getId(void)                   const { return id_; }
		bool     getCompressorEnabled(void)    const { return compressor_enabled_; }
		bool     getPressureSwitch(void)       const { return pressure_switch_; }
		double   getCompressorCurrent(void)    const { return compressor_current_; }
		bool     getClosedLoopControl(void)    const { return closed_loop_control_; }
		double   getAnalogPressure(size_t channel) const
		{
			if (channel >= analog_pressure_.size())
			{
				ROS_ERROR_STREAM("PH getAnalogPressure channel out of bounds : " << channel);
				return 0.0;
			}
			return analog_pressure_[channel];
		}

		void setCompressorEnabled(bool compressor_enabled)         { compressor_enabled_ = compressor_enabled; }
		void setPressureSwitch(bool pressure_switch)               { pressure_switch_ = pressure_switch; }
		void setCompressorCurrent(double compressor_current)       { compressor_current_ = compressor_current; }
		void setClosedLoopControl(bool closed_loop_control)        { closed_loop_control_ = closed_loop_control; }
		void setAnalogPressure(double analog_pressure, size_t channel)
		{
			if (channel >= analog_pressure_.size())
			{
				ROS_ERROR_STREAM("PH setAnalogPressure channel out of bounds : " << channel);
				return;
			}
			analog_pressure_[channel] = analog_pressure;
		}

	private:
		int32_t  id_;
		bool     compressor_enabled_{false};
		bool     pressure_switch_{false};
		double   compressor_current_{0};
		bool     closed_loop_control_{false};
		std::array<double, 2> analog_pressure_{0, 0};
};

typedef StateHandle<const PHState> PHStateHandle;
typedef StateHandle<PHState> PHWritableStateHandle;
class PHStateInterface       : public HardwareResourceManager<PHStateHandle> {};
class RemotePHStateInterface : public HardwareResourceManager<PHWritableStateHandle, ClaimResources> {};

} // namespace
