/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2025, Peter Christoffersen
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <numbers>
#include <algorithm>
#include <stdint.h>

namespace feetech_sts_hardware
{


class STSModel {
    public:
        using id_t = uint8_t;
        using pos_t = int16_t;


        static constexpr pos_t STEPS_PER_REVOLUTION = 4096;
        static constexpr int16_t MAX_VELOCITY = 3400;
        static constexpr int16_t MAX_ACCELERATION = 254;
        static constexpr int16_t DEFAULT_VELOCITY = 2400;
        static constexpr int16_t DEFAULT_ACCELERATION = 200;
        static constexpr pos_t DEFAULT_ZERO_OFFSET = 2048;

        
        constexpr STSModel(uint16_t _model_number, const char *_model_name, double kt) :
            model_number_ { _model_number },
            model_name_ { _model_name },
            kt_ { kt }
        {
        }

        uint16_t model_number() const 
        {
            return model_number_;
        }

        const char *model_name() const 
        {
            return model_name_;
        }

        double torque_from_amps(double amps) const
        {
            return amps * kt_;
        }


        double hw_to_pos(pos_t pos) const
        {
            return 2.0 * std::numbers::pi * static_cast<double>(pos) / static_cast<double>(STEPS_PER_REVOLUTION);
        }
        pos_t pos_to_hw(double rad) const
        {
            auto val = rad * static_cast<double>(STEPS_PER_REVOLUTION) / (2.0 * std::numbers::pi);
            return static_cast<int16_t>(val);
        }

        double hw_to_vel(int16_t vel) const
        {
            return 2.0 * std::numbers::pi * static_cast<double>(vel) / static_cast<double>(STEPS_PER_REVOLUTION);
        }
        int16_t vel_to_hw(double rad) const
        {
            auto val = std::abs(rad * static_cast<double>(STEPS_PER_REVOLUTION) / (2.0 * std::numbers::pi));
            return static_cast<uint16_t>(std::abs(val));
        }

        double hw_to_accel(uint8_t accel) const
        {
            return 2.0 * std::numbers::pi * static_cast<double>(accel) / static_cast<double>(STEPS_PER_REVOLUTION);
        }
        uint8_t accel_to_hw(double rad) const
        {
            auto val = rad * static_cast<double>(STEPS_PER_REVOLUTION) / (2.0 * std::numbers::pi);
            return std::clamp<uint16_t>(std::abs(val), 0, MAX_ACCELERATION);
        }
        double hw_to_load(int16_t val) const
        {
            return static_cast<double>(val) * 10.0;
        }
        double hw_to_voltage(uint8_t val) const
        {
            return static_cast<double>(val) * 10.0;
        }
        double hw_to_temp(uint8_t val) const 
        {
            return static_cast<double>(val);
        }
        double hw_to_current(int16_t val) const
        {
            return static_cast<double>(val) * 6.5 / 1000.0;
        }



        static const STSModel *find(uint16_t model_id);

        static const STSModel *unknown();

    private:
        const uint16_t model_number_;
        const char *model_name_;
        const double kt_; // Motor Torque constant in Nm/A


};

};