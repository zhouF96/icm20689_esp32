#pragma once



//static const uint8_t dmp3_image[] = {
//    #include "Invn/icm20789_img.dmp3.h"
//};
//
//#if USE_20789_NOT_20689
//static const uint8_t EXPECTED_WHOAMI[] = { 0x03 };  /* WHOAMI value for ICM20789 */
//#else
//static const uint8_t EXPECTED_WHOAMI[] = { 0x98 };  /* WHOAMI value for ICM20689 */
//#endif
//
///* FSR configurations */
//int32_t cfg_acc_fsr = 4000; // Accel FSR must be set as +/- 4g and cannot be changed. 
//int32_t cfg_gyr_fsr = 2000; // Gyro FSR must be set at +/- 2000dps and cannot be changed.
//
//inv_icm20789_t icm_device;
//inv_invpres_t invpres_device;
//
///*
// * Mask to keep track of enabled sensors
// */
//uint32_t user_enabled_sensor_mask = 0;
