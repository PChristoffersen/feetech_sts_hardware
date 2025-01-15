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

#include <algorithm>
#include <string>
#include <numbers>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "SCServo.h"
#include "feetech_sts_models.hpp"

namespace feetech_sts_hardware
{

class FeetechSTSHardware : public hardware_interface::SystemInterface {
    
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FeetechSTSHardware)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        using id_t = STSModel::id_t;
        using pos_t = STSModel::pos_t;

        struct Joint {
            struct State {
                double position;
                double velocity;
                double effort;
                double load;
                double voltage;
                double temperature;
                void reset() 
                {
                    position = std::numeric_limits<double>::quiet_NaN();
                    velocity = std::numeric_limits<double>::quiet_NaN();
                    effort = std::numeric_limits<double>::quiet_NaN();
                    load = std::numeric_limits<double>::quiet_NaN();
                    voltage = std::numeric_limits<double>::quiet_NaN();
                    temperature = std::numeric_limits<double>::quiet_NaN();
                }
            };
            struct Command {
                double position;
                double velocity;
                double acceleration;
                void reset()
                {
                    position = std::numeric_limits<double>::quiet_NaN();
                    velocity = std::numeric_limits<double>::quiet_NaN();
                    acceleration = std::numeric_limits<double>::quiet_NaN();
                }
            };
            id_t id;
            pos_t zero_offset;
            const STSModel *model;
            State state;
            Command command;
            Command prev_command;
        };

        SMS_STS sm_st_;

        std::vector<Joint> joints_;

        hardware_interface::return_type set_torque(bool enable);

        void init_command();


};

}