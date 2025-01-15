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

#include "feetech_sts_hardware.hpp"

#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include <chrono>



using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace feetech_sts_hardware
{


CallbackReturn FeetechSTSHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(get_logger(), "Initializing FeetechSTS");

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters.find("port_name") == info_.hardware_parameters.end()) {
        RCLCPP_ERROR(get_logger(), "No port_name found in hardware parameters");
        return CallbackReturn::ERROR;
    }
    if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
        RCLCPP_ERROR(get_logger(), "No baud_rate found in hardware parameters");
        return CallbackReturn::ERROR;
    }

    pos_t default_zero_offset = STSModel::DEFAULT_ZERO_OFFSET;
    if (info_.hardware_parameters.find("default_zero_offset") != info_.hardware_parameters.end()) {
        default_zero_offset = std::stoi(info_.hardware_parameters.at("default_zero_offset"));
    }

    joints_.resize(info_.joints.size());
    for (size_t i=0; i<info_.joints.size(); i++) {
        const auto &joint_info = info_.joints[i];
        auto &joint = joints_[i];
        joint.id = std::stoi(info_.joints[i].parameters.at("id"));
        joint.model = STSModel::unknown();
        if (info_.joints[i].parameters.find("zero_offset") != info_.joints[i].parameters.end()) {
            joint.zero_offset = std::stoi(info_.joints[i].parameters.at("zero_offset"));
        }
        else {
            joint.zero_offset = default_zero_offset;
        }
        joint.state.reset();
        joint.command.reset();
        joint.prev_command.reset();
        if (joint_info.command_interfaces.size() != 3) {
            RCLCPP_ERROR(get_logger(), "Joint '%s' has %zu command interfaces found. 3 expected.", joint_info.name.c_str(), joint_info.command_interfaces.size());
            return CallbackReturn::ERROR;
        }
        if (joint_info.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_ERROR(get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.", joint_info.name.c_str(), joint_info.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }
        if (joint_info.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_ERROR(get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.", joint_info.name.c_str(), joint_info.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }
        if (joint_info.command_interfaces[2].name != hardware_interface::HW_IF_ACCELERATION) {
            RCLCPP_ERROR(get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.", joint_info.name.c_str(), joint_info.command_interfaces[2].name.c_str(), hardware_interface::HW_IF_ACCELERATION);
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Joint '%s' has ID %d", joint_info.name.c_str(), joint.id);
    }

    return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface::ConstSharedPtr> FeetechSTSHardware::on_export_state_interfaces() 
{
    RCLCPP_INFO(get_logger(), "Exporting FeetechSTS state interfaces");
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

    state_interfaces.reserve(info_.joints.size() * 3);

    for (size_t i=0; i<info_.joints.size(); i++) {
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> FeetechSTSHardware::on_export_command_interfaces() 
{
    RCLCPP_INFO(get_logger(), "Exporting FeetechSTS command interfaces");
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

    for (size_t i=0; i<info_.joints.size(); i++) {
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &joints_[i].command.acceleration));
    }

    return command_interfaces;
}


CallbackReturn FeetechSTSHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) 
{
    RCLCPP_INFO(get_logger(), "Configuring FeetechSTS");

    auto port_name = info_.hardware_parameters.at("port_name");
    auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

    // connect
    if (!sm_st_.begin(baud_rate, port_name.c_str())) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to FeetechSTS");
        return CallbackReturn::ERROR;
    }

    // Ping servos to make sure they are responding
    for (auto &joint : joints_) {
        if (sm_st_.Ping(joint.id)<0) {
            RCLCPP_ERROR(get_logger(), "Failed to ping servo with ID %d", joint.id);
            return CallbackReturn::ERROR;
        }
    }

    // Determine servo model for torque calculation
    std::vector<id_t> ids(joints_.size());
    std::transform(joints_.begin(), joints_.end(), ids.begin(), [](const auto &j) { return j.id; });
    uint8_t rxPacket[2];
	sm_st_.syncReadBegin(ids.size(), sizeof(rxPacket));
    sm_st_.syncReadPacketTx(ids.data(), ids.size(), SMS_STS_MODEL_L, sizeof(rxPacket));
    for (size_t i=0; i<ids.size(); i++) {
        auto &joint = joints_[i];
        const auto &joint_info = info_.joints[i];

        if (!sm_st_.syncReadPacketRx(joint.id, rxPacket)) {
            RCLCPP_ERROR(get_logger(), "Failed to read servo model with ID %d", joint.id);
            return CallbackReturn::ERROR;
        }
        uint16_t model_id = sm_st_.syncReadRxPacketToWrod(0);

        auto model = STSModel::find(model_id);
        if (model) {
            RCLCPP_INFO(get_logger(), "Joint '%s' found servo (id=%d, model=%04x, model_name=%s", joint_info.name.c_str(), joint.id, model_id, model->model_name());
        }
        else {
            model = STSModel::unknown();
            RCLCPP_WARN(get_logger(), "Joint '%s' found servo (id=%d, model=%04x, model_name=%s", joint_info.name.c_str(), joint.id, model_id, model->model_name());
        }
        joint.model = model;
    }
    sm_st_.syncReadEnd();


    return CallbackReturn::SUCCESS;
}


CallbackReturn FeetechSTSHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up FeetechSTS");

    sm_st_.end();

    return CallbackReturn::SUCCESS;
}


CallbackReturn FeetechSTSHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) 
{
    RCLCPP_INFO(get_logger(), "Activating FeetechSTS");

    read(rclcpp::Time{}, rclcpp::Duration(0, 0));
    init_command();
    write(rclcpp::Time{}, rclcpp::Duration(0, 0));

    // Enable torque on all servos
    if (set_torque(true) != return_type::OK) {
        RCLCPP_ERROR(get_logger(), "Failed to turn on servo torque");
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechSTSHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) 
{
    RCLCPP_INFO(get_logger(), "Deactivating FeetechSTS");

    // Turn off torque on all servos
    if (set_torque(false) != return_type::OK) {
        RCLCPP_ERROR(get_logger(), "Failed to turn off servo torque");
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}


CallbackReturn FeetechSTSHardware::on_error(const rclcpp_lifecycle::State & previous_state) 
{
    RCLCPP_ERROR(get_logger(), "Error in FeetechSTS");

    if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_ERROR(get_logger(), "Error in FeetechSTS while active. Turning off torque on all servos");
        if (set_torque(false) != return_type::OK) {
            RCLCPP_WARN(get_logger(), "Failed to turn off servo torque");
        }
    }

    return hardware_interface::SystemInterface::on_error(previous_state);
}

return_type FeetechSTSHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{
    #if 0
    auto start = std::chrono::high_resolution_clock::now();
    #endif

    RCLCPP_DEBUG(get_logger(), "Reading FeetechSTS");

    bool success = true;
    std::vector<id_t> ids(joints_.size());
    uint8_t rxPacket[8];

    std::transform(joints_.begin(), joints_.end(), ids.begin(), [](const auto &j) { return j.id; });

	sm_st_.syncReadBegin(ids.size(), sizeof(rxPacket));
    
    sm_st_.syncReadPacketTx(ids.data(), ids.size(), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));
    for (size_t i=0; i<ids.size(); i++) {
        auto &joint = joints_[i];
        if (!sm_st_.syncReadPacketRx(joint.id, rxPacket)) {
            RCLCPP_ERROR(get_logger(), "Failed to read servo with ID %d", joint.id);
            success = false;
            continue;
        }

        joint.state.position = joint.model->hw_to_pos(sm_st_.syncReadRxPacketToWrod(15)-joint.zero_offset);
        joint.state.velocity = joint.model->hw_to_vel(sm_st_.syncReadRxPacketToWrod(15));
        joint.state.load = joint.model->hw_to_load(sm_st_.syncReadRxPacketToWrod(10));
        joint.state.voltage = joint.model->hw_to_voltage(sm_st_.syncReadRxPacketToByte());
        joint.state.temperature = joint.model->hw_to_temp(sm_st_.syncReadRxPacketToByte());

    }
    sm_st_.syncReadEnd();

	sm_st_.syncReadBegin(ids.size(), 2);
    sm_st_.syncReadPacketTx(ids.data(), ids.size(), SMS_STS_PRESENT_CURRENT_L, 2);
    for (size_t i=0; i<ids.size(); i++) {
        auto &joint = joints_[i];
        if (!sm_st_.syncReadPacketRx(joint.id, rxPacket)) {
            RCLCPP_ERROR(get_logger(), "Failed to read servo with ID %d", joint.id);
            success = false;
            continue;
        }

        auto current = joint.model->hw_to_current(sm_st_.syncReadRxPacketToWrod(15));
        joint.state.effort = joint.model->torque_from_amps(current);
    }
    sm_st_.syncReadEnd();


    for (auto &joint : joints_) {
        RCLCPP_INFO(get_logger(), "Read from servo with ID %d   pos=%.2f  vel=%.2f  eff=%.2f", joint.id, joint.state.position, joint.state.velocity, joint.state.effort);
    }

    #if 0
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    RCLCPP_INFO(get_logger(), "Read function took %ld milliseconds", duration.count());
    #endif

    return success ? return_type::OK : return_type::ERROR;
}

return_type FeetechSTSHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{
    RCLCPP_DEBUG(get_logger(), "Writing FeetechSTS");

    std::vector<id_t> ids(joints_.size());
    std::vector<int16_t> positions(joints_.size());
    std::vector<uint16_t> speeds(joints_.size());
    std::vector<uint8_t> accels(joints_.size());
    size_t n_updates = 0;

    for (size_t i=0; i<joints_.size(); i++) {
        auto &joint = joints_[i];

        if (joint.command.position != joint.prev_command.position) {
            ids[n_updates] = joint.id;
            positions[n_updates] = joint.model->pos_to_hw(joint.command.position)+joint.zero_offset;
            speeds[n_updates] = joint.model->vel_to_hw(joint.command.velocity);
            accels[n_updates] = joint.model->accel_to_hw(joint.command.acceleration);
            n_updates++;
            RCLCPP_INFO(get_logger(), "Writing to servo with ID %d   pos=%.2f (%d)  vel=%.2f (%d)  accel=%.2f (%d)", 
                joint.id, 
                joint.command.position, positions[n_updates-1],
                joint.command.velocity, speeds[n_updates-1],
                joint.command.acceleration, accels[n_updates-1]);
            joint.prev_command = joint.command;
        }
    }

    if (n_updates > 0) {
        sm_st_.SyncWritePosEx(ids.data(), n_updates, positions.data(), speeds.data(), accels.data());
    }

    return return_type::OK;
}


hardware_interface::return_type FeetechSTSHardware::set_torque(bool enable)
{
    RCLCPP_INFO(get_logger(), "Setting torque %s FeetechSTS", enable ? "ON" : "OFF");

    for (auto &joint : joints_) {
        if (!sm_st_.EnableTorque(joint.id, enable ? 1 : 0)) {
            RCLCPP_ERROR(get_logger(), "Failed to turn %s servo torque for servo with ID %d", enable ? "on" : "off",  joint.id);
            return return_type::ERROR;
        }
    }

    return return_type::OK;
}


void FeetechSTSHardware::init_command()
{
    for (auto &joint : joints_) {
        joint.command.position = joint.state.position;
        joint.command.velocity = joint.model->hw_to_vel(STSModel::DEFAULT_VELOCITY);
        joint.command.acceleration = joint.model->hw_to_accel(STSModel::DEFAULT_ACCELERATION);
        joint.prev_command.reset();
    }
}


};

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_sts_hardware::FeetechSTSHardware, hardware_interface::SystemInterface)
