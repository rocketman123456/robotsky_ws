// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace openarm::robstride_motor {

inline constexpr uint8_t DEFAULT_MASTER_CAN_ID = 0xFD;

enum class MotorType : uint8_t { RS00 = 0, RS02 = 1, RS03 = 2, RS04 = 3, RS05 = 4, RS06 = 5, COUNT = 6 };

enum class ControlMode : uint8_t {
    MIT = 0,
    POS = 1,
    SPEED = 2,
    CURRENT = 3,
    CSP = 5,
};

enum class CommunicationType : uint8_t {
    GET_ID = 0x00,
    MOTION_CONTROL = 0x01,
    MOTOR_FEEDBACK = 0x02,
    MOTOR_ENABLE = 0x03,
    MOTOR_STOP = 0x04,
    SET_MECHANICAL_ZERO = 0x06,
    SET_CAN_ID = 0x07,
    GET_SINGLE_PARAMETER = 0x11,
    SET_SINGLE_PARAMETER = 0x12,
    ERROR_FEEDBACK = 0x15,
    SAVE_PARAMETERS = 0x16,
    SET_BAUDRATE = 0x17,
    ACTIVE_REPORT = 0x18,
    SET_PROTOCOL = 0x19,
};

enum class ParameterIndex : uint16_t {
    RUN_MODE = 0x7005,
    IQ_REF = 0x7006,
    SPD_REF = 0x700A,
    LIMIT_TORQUE = 0x700B,
    CUR_KP = 0x7010,
    CUR_KI = 0x7011,
    CUR_FILT_GAIN = 0x7014,
    LOC_REF = 0x7016,
    LIMIT_SPD = 0x7017,
    LIMIT_CUR = 0x7018,
    MECH_POS = 0x7019,
    IQF = 0x701A,
    MECH_VEL = 0x701B,
    VBUS = 0x701C,
    ROTATION = 0x701D,
    LOC_KP = 0x701E,
    SPD_KP = 0x701F,
    SPD_KI = 0x7020,
    SPD_FILT_GAIN = 0x7021,
    ACC_RAD = 0x7022,
    VEL_MAX = 0x7023,
    ACC_SET = 0x7024,
};

struct LimitParam {
    double pMin;
    double pMax;
    double vMin;
    double vMax;
    double kpMin;
    double kpMax;
    double kdMin;
    double kdMax;
    double tMin;
    double tMax;
};

inline constexpr std::array<LimitParam, static_cast<std::size_t>(MotorType::COUNT)> MOTOR_LIMIT_PARAMS = {{
    {-12.57, 12.57, -33.0, 33.0, 0.0, 500.0, 0.0, 5.0, -14.0, 14.0},       // RS00
    {-12.57, 12.57, -44.0, 44.0, 0.0, 500.0, 0.0, 5.0, -17.0, 17.0},       // RS02
    {-12.57, 12.57, -20.0, 20.0, 0.0, 5000.0, 0.0, 100.0, -60.0, 60.0},    // RS03
    {-12.57, 12.57, -15.0, 15.0, 0.0, 5000.0, 0.0, 100.0, -120.0, 120.0},  // RS04
    {-12.57, 12.57, -50.0, 50.0, 0.0, 500.0, 0.0, 5.0, -5.5, 5.5},         // RS05
    {-12.57, 12.57, -50.0, 50.0, 0.0, 5000.0, 0.0, 100.0, -36.0, 36.0},    // RS06
}};

}  // namespace openarm::robstride_motor
