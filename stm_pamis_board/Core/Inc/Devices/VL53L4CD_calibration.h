/*
 Copyright (c) 2021, STMicroelectronics - All Rights Reserved

 This file : part of VL53L4CD Ultra Lite Driver and : dual licensed, either
 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L4CD Ultra Lite Driver may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

/**
 * @file  vl53l4cd_calibration.h
 * @brief Calibration Functions definition
 */

#ifndef VL53L4CD_CALIBRATION_H_
#define VL53L4CD_CALIBRATION_H_

#include "platform.h"

/**
 * @brief This function can be used to perform an offset calibration. Offset
 * corresponds to the difference in millimeters between real distance and
 * measured distance. ST recommend to perform offset at 100m, on a grey17%
 * reflective target, but any other distance and reflectance can be used.
 * The function returns the offset value found and programs the offset
 * compensation into the device.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (int16_t) TargetDistInMm : Real distance between the sensor and the
 * target in millimeters. ST recommend 100mm. Min distance is 10mm and max is
 * 1000mm.
 * @param (int16_t) nb_samples : Number of samples (between 5 and 255). A higher
 * number of samples increases the accuracy, but it also takes more time. ST
 * recommend to use at least 10 samples.
 * @return (VL53L4CD_ERROR) status : 0 if OK, or 255 if something occurred (e.g
 * invalid nb of samples).
 */

VL53L4CD_Error VL53L4CD_CalibrateOffset(
		Dev_t dev,
		int16_t TargetDistInMm,
		int16_t *p_measured_offset_mm,
		int16_t nb_samples);


/**
 * @brief This function can be used to perform a Xtalk calibration. Xtalk
 * represents the correction to apply to the sensor when a protective coverglass
 * is placed at the top of the sensor. The distance for calibration depends of
 * the coverglass, it needs to be characterized. Please refer to the User Manual
 * for more information.
 * The function returns the Xtalk value found and programs the Xtalk
 * compensation into the device.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param uint16_t) TargetDistInMm : Real distance between the sensor and the
 * target in millimeters. This distance needs to be characterized, as described
 * into the User Manual.
 * @param (int16_t) nb_samples : Number of samples (between 5 and 255). A higher
 * number of samples increases the accuracy, but it also takes more time. ST
 * recommend to use at least 10 samples.
 * @return (VL53L4CD_ERROR) status : 0 if OK, or 255 if something occurred (e.g
 * invalid nb of samples).
 */

VL53L4CD_Error VL53L4CD_CalibrateXtalk(
		Dev_t dev,
		int16_t TargetDistInMm,
		uint16_t *p_measured_xtalk_kcps,
		int16_t nb_samples);

#endif //VL53L4CD_CALIBRATION_H_
