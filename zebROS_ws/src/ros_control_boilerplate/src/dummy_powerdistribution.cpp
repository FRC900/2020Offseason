
// WPILIB HAL on the Rio provides functions to access the unique
// set of functions for each power distribution option.  WPI sim
// only simulates the common features described in PowerDistribution.cpp
// and doesn't even provide sim versions of the HAL_REV_*PDH* and HAL_CTRE_*PDP*
// functions. Implement those functions here, using PowerDistribution sim
// code where available and stubs for the remaining functions. This means
// we won't have a high-fidelity simulation but at least it will allow
// us to use common code between hw and sim interfaces
//
//

#include "hal/PowerDistribution.h"
#include "REVPDH.h"

extern "C"
{

HAL_REVPDHHandle HAL_REV_InitializePDH(int32_t module,
                                       const char* allocationLocation,
                                       int32_t* status)
{
	return HAL_InitializePowerDistribution(module, HAL_PowerDistributionType_kRev, allocationLocation, status);
}

int32_t HAL_REV_GetPDHModuleNumber(HAL_REVPDHHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionModuleNumber(handle, status);
}

HAL_Bool HAL_REV_CheckPDHModuleNumber(int32_t module)
{
	return HAL_CheckPowerDistributionModule(module, HAL_PowerDistributionType_kRev);
}

double HAL_REV_GetPDHChannelCurrent(HAL_REVPDHHandle handle, int32_t channel,
                                    int32_t* status)
{
	return HAL_GetPowerDistributionChannelCurrent(handle, channel, status);
}

uint16_t HAL_REV_GetPDHTotalCurrent(HAL_REVPDHHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionTotalCurrent(handle, status);
}

void HAL_REV_SetPDHSwitchableChannel(HAL_REVPDHHandle handle, HAL_Bool enabled,
                                     int32_t* status)
{
	HAL_SetPowerDistributionSwitchableChannel(handle, enabled, status);
}

HAL_Bool HAL_REV_GetPDHSwitchableChannelState(HAL_REVPDHHandle handle,
                                              int32_t* status)
{
	return HAL_GetPowerDistributionSwitchableChannel(handle, status);
}

HAL_Bool HAL_REV_CheckPDHChannelBrownout(HAL_REVPDHHandle /*handle*/,
                                         int32_t /*channel*/, int32_t* status)
{
	*status = 0;
	return false;
}

double HAL_REV_GetPDHSupplyVoltage(HAL_REVPDHHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionVoltage(handle, status);
}

HAL_Bool HAL_REV_IsPDHEnabled(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return true;
}

HAL_Bool HAL_REV_CheckPDHBrownout(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return false;
}

HAL_Bool HAL_REV_CheckPDHCANWarning(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return false;
}
HAL_Bool HAL_REV_CheckPDHHardwareFault(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return false;
}
HAL_Bool HAL_REV_CheckPDHStickyBrownout(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return false;
}

HAL_Bool HAL_REV_CheckPDHStickyCANWarning(HAL_REVPDHHandle /*handle*/,
                                          int32_t* status)
{
	*status = 0;
	return false;
}

HAL_Bool HAL_REV_CheckPDHStickyCANBusOff(HAL_REVPDHHandle /*handle*/,
                                         int32_t* status){
	*status = 0;
	return false;
}

HAL_Bool HAL_REV_CheckPDHStickyHardwareFault(HAL_REVPDHHandle /*handle*/,
                                             int32_t* status)
{
	*status = 0;
	return false;
}

HAL_Bool HAL_REV_CheckPDHStickyFirmwareFault(HAL_REVPDHHandle /*handle*/,
                                             int32_t* status){
	*status = 0;
	return false;
}

HAL_Bool HAL_REV_CheckPDHStickyChannelBrownout(HAL_REVPDHHandle /*handle*/,
                                               int32_t /*channel*/,
                                               int32_t* status)
{
	*status = 0;
	return false;
}
HAL_Bool HAL_REV_CheckPDHStickyHasReset(HAL_REVPDHHandle /*handle*/,
                                        int32_t* status)
{
	*status = 0;
	return false;
}

REV_PDH_Version HAL_REV_GetPDHVersion(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	*status = 0;
	REV_PDH_Version ver;
	ver.firmwareMajor = 900;
	ver.firmwareMinor = 900;
	ver.firmwareFix = 900;
	ver.hardwareRev = 900;
	ver.uniqueId = 900;

	return ver;
}

void HAL_REV_ClearPDHFaults(HAL_REVPDHHandle handle, int32_t* status)
{
	HAL_ClearPowerDistributionStickyFaults(handle, status);
}

void HAL_REV_IdentifyPDH(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	status = 0;
}

HAL_REVPDHHandle HAL_InitializePDP(int32_t module,
                                   const char* allocationLocation,
                                   int32_t* status)
{
	return HAL_InitializePowerDistribution(module, HAL_PowerDistributionType_kCTRE, allocationLocation, status);
}

HAL_Bool HAL_CheckPDPModule(int32_t module)
{
	return HAL_CheckPowerDistributionModule(module, HAL_PowerDistributionType_kRev);
}

double HAL_GetPDPTemperature(HAL_PDPHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionTemperature(handle, status);
}

double HAL_GetPDPVoltage(HAL_PDPHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionVoltage(handle, status);
}

double HAL_GetPDPChannelCurrent(HAL_PDPHandle handle, int32_t channel,
                                int32_t* status)
{
	return HAL_GetPowerDistributionChannelCurrent(handle, channel, status);
}

double HAL_GetPDPTotalCurrent(HAL_PDPHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionTotalCurrent(handle, status);
}

double HAL_GetPDPTotalPower(HAL_PDPHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return 0;
}

double HAL_GetPDPTotalEnergy(HAL_PDPHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return 0;
}

void HAL_ResetPDPTotalEnergy(HAL_PDPHandle /*handle*/, int32_t* status)
{
	*status = 0;
}

void HAL_ClearPDPStickyFaults(HAL_PDPHandle handle, int32_t* status)
{
	HAL_ClearPowerDistributionStickyFaults(handle, status);
}


} // extern "C"
