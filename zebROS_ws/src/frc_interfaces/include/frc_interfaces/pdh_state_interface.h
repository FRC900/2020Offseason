#ifndef PDH_STATE_INTERFACE_INC_
#define PDH_STATE_INTERFACE_INC_

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

//holds data from the PDH
class PDHHWState
{
    public:
        //access and set
		PDHHWState(uint32_t module_number)
			: module_number_(module_number)
		{
		}
		int32_t  getModuleNumber(void) const           {return module_number_;}
		bool     getEnabled(void) const                {return enabled_;}
        double   getVoltage(void) const                {return voltage_;}
		bool     getBrownout(void) const               {return brownout_;}
		bool     getStickyBrownout(void) const         {return sticky_brownout_;}
		bool     getCANWarning(void) const             {return can_warning_;}
		bool     getStickyCANWarning(void) const       {return sticky_can_warning_;}
		bool     getStickyCANBusOff(void) const        {return sticky_can_bus_off_;}
		bool     getHardwareFault(void) const          {return hardware_fault_;}
		bool     getStickyHardwareFault(void) const    {return sticky_hardware_fault_;}
		bool     getStickyFirmwareFault(void) const    {return sticky_firmware_fault_;}
		bool     getStickyHasReset(void) const         {return sticky_has_reset_;}
		uint32_t getFirmwareMajor(void) const          {return firmware_major_;}
		uint32_t getFirmwareMinor(void) const          {return firmware_minor_;}
		uint32_t getFirmwareiFix(void) const           {return firmware_fix_;}
		uint32_t getHardwareRev(void) const            {return hardware_rev_;}
		uint32_t getUniqueID(void) const               {return unique_id_;}
        double   getTotalCurrent(void) const           {return total_current_;}
        bool     getSwitchableChannelState(void) const {return switchable_channel_state_;};
        double   getChannelCurrent(size_t channel) const {
            if(channel >= current_.size())
            {
                ROS_WARN_STREAM("Invalid channel. Cannot read current.");
                return 0.0;
            }
            return current_[channel];
        }
        bool  getChannelBrownout(size_t channel) const {
            if(channel >= current_.size())
            {
                ROS_WARN_STREAM("Invalid channel. Cannot read brownout.");
                return false;
            }
            return channel_brownout_[channel];
        }
        bool  getStickyChannelBrownout(size_t channel) const {
            if(channel >= current_.size())
            {
                ROS_WARN_STREAM("Invalid channel. Cannot read brownout.");
                return false;
            }
            return sticky_channel_brownout_[channel];
        }

		void setEnabled(bool enabled)                                 {enabled_ = enabled;}
        void setVoltage(double voltage)                               {voltage_ = voltage;}
        void setBrownout(bool brownout)                               {brownout_ = brownout;}
        void setStickyBrownout(bool sticky_brownout)                  {sticky_brownout_ = sticky_brownout;}
        void setCANWarning(bool can_warning)                          {can_warning_ = can_warning;}
        void setStickyCANWarning(bool sticky_can_warning)             {sticky_can_warning_ = sticky_can_warning;}
        void setStickyCANBusOff(bool sticky_can_bus_off)              {sticky_can_bus_off_ = sticky_can_bus_off;}
        void setHardwareFault(bool hardware_fault)                    {hardware_fault_ = hardware_fault;}
        void setStickyHardwareFault(bool sticky_hardware_fault)       {sticky_hardware_fault_ = sticky_hardware_fault;}
        void setStickyFirmwareFault(bool sticky_firmware_fault)       {sticky_firmware_fault_ = sticky_firmware_fault;}
        void setStickyHasReset(bool sticky_has_reset)                 {sticky_has_reset_ = sticky_has_reset;}
		void setFirmwareMajor(uint32_t firmware_major)                {firmware_major_= firmware_major;}
		void setFirmwareMinor(uint32_t firmware_minor)                {firmware_minor_= firmware_minor;}
		void setFirmwareFix(uint32_t firmware_fix)                    {firmware_fix_= firmware_fix;}
		void setHardwareRev(uint32_t firmware_fix)                    {firmware_fix_= firmware_fix;}
		void setUniqueID(uint32_t unique_id)                          {unique_id_ = unique_id;}
        void setTotalCurrent(double total_current)                    {total_current_ = total_current;}
        void setSwitchableChannelState(bool switchable_channel_state) {switchable_channel_state_ = switchable_channel_state;}
        void setChannelCurrent(double current, size_t channel) {
            if(channel >= current_.size())
            {
                ROS_WARN_STREAM("Invalid channel. Cannot set current.");
                return;
            }
            current_[channel] = current;
        }
        void setChannelBrownout(bool brownout, size_t channel) {
            if(channel >= current_.size())
            {
                ROS_WARN_STREAM("Invalid channel. Cannot set brownout.");
                return;
            }
            channel_brownout_[channel] = brownout;
        }
        void setStickyChannelBrownout(bool brownout, size_t channel) {
            if(channel >= current_.size())
            {
                ROS_WARN_STREAM("Invalid channel. Cannot set brownout.");
                return;
            }
            sticky_channel_brownout_[channel] = brownout;
        }

    private:
		int32_t  module_number_;
		bool     enabled_{false};
        double   voltage_{0.};
		bool     brownout_{false};
		bool     sticky_brownout_{false};
		bool     can_warning_{false};
		bool     sticky_can_warning_{false};
		bool     sticky_can_bus_off_{false};
		bool     hardware_fault_{false};
		bool     sticky_hardware_fault_{false};
		bool     sticky_firmware_fault_{false};
		bool     sticky_has_reset_{false};
		uint32_t firmware_major_{0};
		uint32_t firmware_minor_{0};
		uint32_t firmware_fix_{0};
		uint32_t hardware_rev_{0};
		uint32_t unique_id_{0};
        double   total_current_{0.};
        bool     switchable_channel_state_{false};
        std::array<double, 20> current_{0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};
        std::array<bool, 20>   channel_brownout_{false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
        std::array<bool, 20>   sticky_channel_brownout_{false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

};

typedef StateHandle<const PDHHWState> PDHStateHandle;
typedef StateHandle<PDHHWState> PDHWritableStateHandle;

class PDHStateInterface       : public HardwareResourceManager<PDHStateHandle> {};
class RemotePDHStateInterface : public HardwareResourceManager<PDHWritableStateHandle,   ClaimResources> {};
}

#endif
