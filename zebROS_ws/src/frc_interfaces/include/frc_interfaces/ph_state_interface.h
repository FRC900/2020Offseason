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
		double   getAnalogPressure(void)       const { return analog_pressure_; }
		bool     getClosedLoopControl(void)    const { return closed_loop_control_; }

		void setCompressorEnabled(bool compressor_enabled)         { compressor_enabled_ = compressor_enabled; }
		void setPressureSwitch(bool pressure_switch)               { pressure_switch_ = pressure_switch; }
		void setCompressorCurrent(double compressor_current)       { compressor_current_ = compressor_current; }
		void setAnalogPressure(double analog_pressure)             { analog_pressure_ = analog_pressure; }
		void setClosedLoopControl(bool closed_loop_control)        { closed_loop_control_ = closed_loop_control; }

	private:
		int32_t  id_;
		bool     compressor_enabled_{false};
		bool     pressure_switch_{false};
		double   compressor_current_{0};
		double   analog_pressure_{0};
		bool     closed_loop_control_{false};
};

typedef StateHandle<const PHState> PHStateHandle;
typedef StateHandle<PHState> PHWritableStateHandle;
class PHStateInterface       : public HardwareResourceManager<PHStateHandle> {};
class RemotePHStateInterface : public HardwareResourceManager<PHWritableStateHandle, ClaimResources> {};

} // namespace
