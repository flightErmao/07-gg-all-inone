/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "axis.h"
#include "maths.h"
#include "vector.h"
#include "sensor_alignment.h"

typedef struct boardAlignment_s {
    int32_t rollDegrees;
    int32_t pitchDegrees;
    int32_t yawDegrees;
} boardAlignment_t;

// 传感器对齐函数
void alignSensorViaMatrix(vector3_t *dest, matrix33_t *rotationMatrix);
void alignSensorViaRotation(vector3_t *dest, sensor_align_e rotation);

// 初始化板级对齐（可选，用于全局板级旋转）
void initBoardAlignment(const boardAlignment_t *boardAlignment);
