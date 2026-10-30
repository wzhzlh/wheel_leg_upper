/**
* @file mujoco_ros2_control_plugin.cpp
*
* @brief This file contains the implementation of the Mujoco ROS2 Control plugin.
*
* @author Adrian Danzglock
* @date 2023
* @license BSD 3-Clause License
* @copyright Copyright (c) 2023, DFKI GmbH
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this list of conditions
*    and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
*    and the following disclaimer in the documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of DFKI GmbH nor the names of its contributors may be used to endorse or promote
*    products derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
* FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
* THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
* The init_controller_manager method contains a modified version of the original code from Cyberbotics Ltd.
* https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_control/src/Ros2Control.cpp
*
* Original code licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
        *
        *     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
        * limitations under the License.
*/

#include "mujoco_ros2_control/mujoco_ros2_control_plugin.hpp"

namespace mujoco_ros2_control {
    MujocoRos2Control::MujocoRos2Control(rclcpp::Node::SharedPtr &node) : nh_(node) {
        // set up the parameter listener
        param_listener_ = std::make_shared<ParamListener>(nh_);
        param_listener_->refresh_dynamic_parameters();

        params_ = param_listener_->get_params();

        // Check that ROS has been initialized
        if (!rclcpp::ok())
        {
            RCLCPP_FATAL(nh_->get_logger(), "Unable to initialize Mujoco node.");
            return;
        }

        // create publisher for the Clock
        publisher_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SystemDefaultsQoS());
        clock_publisher_ = std::make_unique<ClockPublisher>(publisher_);

        // 启动门控状态（latched）：sim_bridge 据此决定何时给轮子下发力矩，
        // 避免"挂架还夹着机身、轮子已经出大力"导致释放瞬间被弹飞。
        start_gate_publisher_ = nh_->create_publisher<std_msgs::msg::Bool>(
            "/mujoco/start_gate_open", rclcpp::QoS(1).transient_local().reliable());
        {
            std_msgs::msg::Bool gate_msg;
            gate_msg.data = false;
            start_gate_publisher_->publish(gate_msg);
        }

        // mujoco related parameters
        show_gui_ = params_.show_gui;
        real_time_factor_ = params_.real_time_factor;
        pub_clock_frequency_ = params_.clock_publisher_frequency;

        init_mujoco();

        init_controller_manager();

        // Start MuJoCo
        // 有 keyframe 就用 keyframe 0（"home"）：hinge 的 qpos0 恒为 0，机身高度又只由
        // base_link 的 pos 决定，想要"两轮同时着地"这类初始姿态只能通过 keyframe 给。
        if (mujoco_model_->nkey > 0) {
            mj_resetDataKeyframe(mujoco_model_, mujoco_data_, 0);
        } else {
            mj_resetData(mujoco_model_, mujoco_data_);
        }

        // compute forward kinematics for new pos
        // 注意：这里不再先跑一步 mj_step。控制还没起来的时候积分没有任何意义，
        // 反而会把机器人往地上砸（闭链 + 接触求解很容易把模型弹飞）。
        // 第一个物理步由 update() 里的启动门控决定什么时候执行。
        mj_forward(mujoco_model_, mujoco_data_);
        mujoco_start_time_ = mujoco_data_->time;

        // 记录挂架要保持的机身位姿（自由关节 qpos 前 7 个）
        for (int j = 0; j < mujoco_model_->njnt; j++) {
            if (mujoco_model_->jnt_type[j] != mjJNT_FREE) {
                continue;
            }
            const int qpos_adr = mujoco_model_->jnt_qposadr[j];
            for (int k = 0; k < 7; k++) {
                hold_base_qpos_[k] = mujoco_data_->qpos[qpos_adr + k];
            }
            break;
        }

        if (params_.start_gate_enabled) {
            std::string required;
            for (const auto &name : params_.start_gate_controllers) {
                if (!required.empty()) {
                    required += ", ";
                }
                required += name;
            }
            RCLCPP_INFO(nh_->get_logger(),
                        "启动门控已启用：等 [%s] 全部 active 后再保持 %.2f s 才放开物理步进",
                        required.c_str(), params_.start_gate_hold);
        }

        clock_gettime(CLOCK_MONOTONIC, &startTime_);
#ifdef USE_LIBSIMULATE
        mj_vis_.init(mujoco_model_, mujoco_data_);
#else
        mj_vis_.init(mujoco_model_, mujoco_data_, show_gui_);
#endif

        registerSensors();
        RCLCPP_INFO(nh_->get_logger(), "Sim environment setup complete");
    }

    MujocoRos2Control::~MujocoRos2Control()
    {
        stop_.store(true);
        if (start_gate_thread_.joinable()) {
            start_gate_thread_.join();
        }
        for (auto &thread : camera_threads_) {
            thread.join();
        }
        thread_executor_spin_.join();
        // deallocate existing mjModel
        mj_deleteModel(mujoco_model_);

        // deallocate existing mjData
        mj_deleteData(mujoco_data_);

        mj_vis_.terminate();
    }

    void MujocoRos2Control::update() {
        mjtNum simstart = mujoco_data_->time;
        timespec currentTime{};
        param_listener_->refresh_dynamic_parameters();
        params_ = param_listener_->get_params();
        // run until the next frame must be rendered with 60Hz
        if (mj_vis_.sim->run) {
            while( mujoco_data_->time - simstart < 1.0/60.0 ) {
                // check that mujoco is not faster than the expected realtime factor
                clock_gettime(CLOCK_MONOTONIC, &currentTime);
                if (double (currentTime.tv_sec-startTime_.tv_sec) + double (currentTime.tv_nsec-startTime_.tv_nsec) / 1e9 >= (mujoco_data_->time-mujoco_start_time_)*params_.real_time_factor) {
                    publish_sim_time();
                    rclcpp::Time sim_time_ros = rclcpp::Time((int64_t) (mujoco_data_->time * 1e+9), RCL_ROS_TIME);
                    rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

                    // check if we should update the controllers
                    if (sim_period >= control_period_) {
                        // store simulation time
                        last_update_sim_time_ros_ = sim_time_ros;
                        // update the robot simulation with the state of the mujoco model
                        controller_manager_->read(sim_time_ros, sim_period);
                        // compute the controller commands
                        controller_manager_->update(sim_time_ros, sim_period);
                        // update the mujoco model with the result of the controller
                        controller_manager_->write(sim_time_ros, sim_period);
                    }
                    // Calculate the next mujoco step（启动门控没放开时只推进时钟，不积分）
                    step_simulation();
                }
            }
        } else {
            mj_forward(mujoco_model_, mujoco_data_);
            mj_vis_.sim->speed_changed = true;
        }
        mj_vis_.update();
    }

    void MujocoRos2Control::step_simulation() {
        if (start_gate_opened_) {
            mj_step(mujoco_model_, mujoco_data_);
            return;
        }

        // ---- 启动门控：挂架模式 ----
        // 正常积分一步，然后每一步把机身（free joint）的水平位移/姿态按回初始值、竖直方向放开，
        // 相当于把躯干挂在试验台的竖直导轨上：
        //   * 机身不会倒、不会漂 → 控制器起来之前不会自由落体、也不会被接触求解弹飞；
        //   * 腿、轮子仍按控制器指令真实运动，竖直方向还能被 VMC 顶到目标高度
        //     → 释放瞬间不会因为"指令一直在涨、状态却一直是 0"而猛冲；
        //   * /clock、joint_states、IMU 都照常发布 → leg_control 能走完启动自检。
        mj_step(mujoco_model_, mujoco_data_);
        clamp_base_to_hold_pose();
        mj_forward(mujoco_model_, mujoco_data_);

        update_start_gate();
    }

    void MujocoRos2Control::clamp_base_to_hold_pose() {
        for (int j = 0; j < mujoco_model_->njnt; j++) {
            if (mujoco_model_->jnt_type[j] != mjJNT_FREE) {
                continue;
            }
            const int qpos_adr = mujoco_model_->jnt_qposadr[j];
            const int dof_adr = mujoco_model_->jnt_dofadr[j];

            // 只夹住水平位移和姿态，竖直方向放开着：
            // 相当于把机器人挂在"竖直导轨"上——机身不会倒、不会漂，
            // 但腿仍然能按 VMC 的指令把机身顶到目标高度，轮子也能自由滚动。
            // 这样放开时腿已经处在控制器想要的姿态，不会因为"腿被冻住、指令却一直在涨"而抖一下。
            mujoco_data_->qpos[qpos_adr + 0] = hold_base_qpos_[0];
            mujoco_data_->qpos[qpos_adr + 1] = hold_base_qpos_[1];
            for (int k = 3; k < 7; k++) {
                mujoco_data_->qpos[qpos_adr + k] = hold_base_qpos_[k];
            }
            mujoco_data_->qvel[dof_adr + 0] = 0.0;
            mujoco_data_->qvel[dof_adr + 1] = 0.0;
            for (int k = 3; k < 6; k++) {
                mujoco_data_->qvel[dof_adr + k] = 0.0;
            }
        }
    }

    bool MujocoRos2Control::start_gate_controllers_active() {
        if (!controller_manager_) {
            return false;
        }
        const auto controllers = controller_manager_->get_loaded_controllers();
        for (const auto &required : params_.start_gate_controllers) {
            const auto it = std::find_if(
                    controllers.begin(), controllers.end(),
                    [&required](const controller_manager::ControllerSpec &spec) {
                        return spec.info.name == required;
                    });
            if (it == controllers.end() || !it->c) {
                return false;
            }
            if (it->c->get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                return false;
            }
        }
        return true;
    }

    void MujocoRos2Control::update_start_gate() {
        if (start_gate_opened_) {
            return;
        }

        const double sim_now = mujoco_data_->time;

        auto publish_open = [this]() {
            if (!start_gate_publisher_) {
                return;
            }
            std_msgs::msg::Bool gate_msg;
            gate_msg.data = true;
            start_gate_publisher_->publish(gate_msg);
        };

        if (!params_.start_gate_enabled) {
            start_gate_opened_ = true;
            publish_open();
            RCLCPP_INFO(nh_->get_logger(), "启动门控已关闭，立即开始物理步进");
            return;
        }

        if (start_gate_ready_time_ < 0.0) {
            if (!start_gate_controllers_active_.load()) {
                if (!start_gate_timeout_warned_ && sim_now > params_.start_gate_timeout) {
                    start_gate_timeout_warned_ = true;
                    start_gate_opened_ = true;
                    publish_open();
                    RCLCPP_WARN(nh_->get_logger(),
                                "等待控制器 active 超过 %.1f s（仿真时间），强制放开启动门控",
                                params_.start_gate_timeout);
                }
                return;
            }
            start_gate_ready_time_ = sim_now;
            RCLCPP_INFO(nh_->get_logger(),
                        "控制器已全部 active（sim t=%.3f），挂架保持 %.2f s 后放开物理步进",
                        sim_now, params_.start_gate_hold);
        }

        if (sim_now - start_gate_ready_time_ >= params_.start_gate_hold) {
            start_gate_opened_ = true;
            publish_open();
            RCLCPP_INFO(nh_->get_logger(),
                        "启动门控放开：开始物理步进（sim t=%.3f，挂架保持 %.2f s）",
                        sim_now, sim_now - start_gate_ready_time_);
        }
    }

    void MujocoRos2Control::publish_sim_time() {
        double sim_time = mujoco_data_->time;
        if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_) < 1.0/pub_clock_frequency_)
            return;
        if (clock_publisher_->trylock()) {
            clock_publisher_->msg_.clock.sec = std::floor(sim_time);
            clock_publisher_->msg_.clock.nanosec = std::floor((sim_time-std::floor(sim_time))*1e9);
            clock_publisher_->unlockAndPublish();
            last_pub_clock_time_ = sim_time;
        }
    }

    void MujocoRos2Control::init_mujoco() {
        char error[1000];

        // create mjModel
        mujoco_model_ = mj_loadXML(params_.robot_model_path.c_str(), NULL, error, 1000);

        if (!mujoco_model_) {
            RCLCPP_FATAL(nh_->get_logger(), "Could not load mujoco model with error: %s.\n", error);
            return;
        } else {
            // No problem with margins
            RCLCPP_INFO(nh_->get_logger(), "loaded mujoco model");
        }

        // Set simulation frequency
        mujoco_model_->opt.timestep = 1.0 / params_.simulation_frequency;

        // create mjData corresponding to mjModel
        mujoco_data_ = mj_makeData(mujoco_model_);
        if (!mujoco_data_)
        {
            RCLCPP_FATAL(nh_->get_logger(), "Could not create mujoco data from model.");
            return;
        }else {
            RCLCPP_INFO(nh_->get_logger(), "Created mujoco data");
        }

        // get the Mujoco simulation period as ros duration
        mujoco_period_ = rclcpp::Duration::from_seconds(mujoco_model_->opt.timestep);
    }

    void MujocoRos2Control::init_controller_manager() {
        RCLCPP_INFO(nh_->get_logger(), "init controller manager");
        try {
            robot_hw_sim_loader_.reset(
                    new pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>
                            ("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"));
        } catch (pluginlib::LibraryLoadException &ex) {
            RCLCPP_FATAL(nh_->get_logger() , "Failed to create robot sim interface loader: %s", ex.what());
        }

        std::string urdf_string;
        urdf::Model urdf_model;
        std::vector<hardware_interface::HardwareInfo> control_hardware;
        resource_manager_ = std::make_unique<hardware_interface::ResourceManager>();

        try {
            urdf_string = params_.robot_description;
            urdf_model.initString(urdf_string);
            control_hardware = hardware_interface::parse_control_resources_from_urdf(urdf_string);
        } catch (const std::runtime_error & ex) {
            RCLCPP_ERROR(nh_->get_logger(), "Error parsing URDF in mujoco_ros2_control plugin: %s",
                         ex.what());
            rclcpp::shutdown();
        }

        try {
            resource_manager_->load_urdf(urdf_string, false, false);
        } catch (...) {
            // This error should be normal as the resource manager is not supposed to load and initialize
            // them
            RCLCPP_ERROR(nh_->get_logger(), "Error initializing URDF to resource manager!");
        }

        for (auto & hw_info : control_hardware) {
            const std::string hardware_type = hw_info.hardware_class_type;
            auto system = std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface>(robot_hw_sim_loader_->createUnmanagedInstance(hardware_type));
            system->initSim(mujoco_model_, mujoco_data_, hw_info, &urdf_model);
            resource_manager_->import_component(std::move(system), hw_info);
            // activate all components
            rclcpp_lifecycle::State state(
                    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                    hardware_interface::lifecycle_state_names::ACTIVE);
            resource_manager_->set_component_state(hw_info.name, state);
        }

        if (resource_manager_->is_urdf_already_loaded()) {
            RCLCPP_DEBUG(nh_->get_logger(), "URDF is already loaded");
        }

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        controller_manager_.reset(
            new controller_manager::ControllerManager(
                std::move(resource_manager_), 
                executor_, 
                "controller_manager", 
                nh_->get_namespace()));
        
        executor_->add_node(controller_manager_);
        
        if (!controller_manager_->has_parameter("update_rate")) {
            RCLCPP_ERROR(nh_->get_logger(), "controller manager doesn't have an update_rate parameter");
            return;
        }

        long cm_update_rate = controller_manager_->get_parameter("update_rate").as_int();
        control_period_ = rclcpp::Duration(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(1.0 / static_cast<double>(cm_update_rate))));
        // Check the period against the simulation period
        if (control_period_ < mujoco_period_) {
            RCLCPP_ERROR(nh_->get_logger(), "The controller period (%f) is faster than the simulation period (%f).",
                         control_period_.seconds(), mujoco_period_.seconds());
            control_period_ = mujoco_period_;
        } else if (control_period_ > mujoco_period_) {
            if (control_period_ < mujoco_period_) {
                RCLCPP_WARN(nh_->get_logger(), "The controller period (%f) is slower than the simulation period (%f).",
                            control_period_.seconds(), mujoco_period_.seconds());
            }
        }

        // Force setting of use_sime_time parameter
        controller_manager_->set_parameter(
        rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

        stop_ = false;
        auto spin = [this]() {
            while(rclcpp::ok() && !stop_.load()) {
                executor_->spin_once();
            }
        };
        thread_executor_spin_ = std::thread(spin);

        // 启动门控的控制器状态轮询：必须放在独立线程里，
        // 因为 switch_controller 期间 controller_manager 会持锁等 update() 把切换做完，
        // 在仿真主循环里查询会互相卡死（spawner 5 s 超时）。
        if (params_.start_gate_enabled && !params_.start_gate_controllers.empty()) {
            start_gate_thread_ = std::thread(&MujocoRos2Control::start_gate_poll_loop, this);
        } else {
            start_gate_controllers_active_.store(true);
        }
    }

    void MujocoRos2Control::start_gate_poll_loop() {
        while (rclcpp::ok() && !stop_.load()) {
            if (start_gate_controllers_active()) {
                start_gate_controllers_active_.store(true);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }


    void MujocoRos2Control::registerSensors() {
        // Add sensors
        if (mujoco_model_->nsensor > 0) {
            std::map<std::string, mujoco_ros2_sensors::MujocoRos2Sensors::Sensors> sensors;
            for (int id = 0; id < mujoco_model_->nsensor; id++) {
                mujoco_ros2_sensors::MujocoRos2Sensors::Sensors sensor;
                int obj_id = mujoco_model_->sensor_objid[id];
                int obj_type = mujoco_model_->sensor_objtype[id];
                std::string obj_name = mj_id2name(mujoco_model_, obj_type, obj_id);
                int sensor_type = mujoco_model_->sensor_type[id];
                std::string sensor_name = mj_id2name(mujoco_model_, mjOBJ_SENSOR, id);
                int sensor_adr = mujoco_model_->sensor_adr[id];
                int sensor_dim = mujoco_model_->sensor_dim[id];
                if (sensors.find(obj_name) == sensors.end()) {
                    sensors.insert(std::make_pair(obj_name, sensor));
                    sensors.at(obj_name).obj_type = obj_type;
                }
                sensors.at(obj_name).sensor_ids.push_back(id);
                sensors.at(obj_name).sensor_types.push_back(sensor_type);
                sensors.at(obj_name).sensor_names.push_back(sensor_name);
                sensors.at(obj_name).sensor_addresses.push_back(sensor_adr);
                sensors.at(obj_name).sensor_dimensions.push_back(sensor_dim);
            }
            mujoco_ros2_sensors_ = std::make_shared<mujoco_ros2_sensors::MujocoRos2Sensors>(executor_, mujoco_model_, mujoco_data_, sensors);
        }

        // Add cameras
        if (mujoco_model_->ncam > 0) {
            cameras_.resize(mujoco_model_->ncam);
            for (int id = 0; id < mujoco_model_->ncam; id++) {
                std::string name = mj_id2name(mujoco_model_, mjOBJ_CAMERA, id);
                auto node = camera_nodes_.emplace_back(rclcpp::Node::make_shared(name, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
                executor_->add_node(node);
                cameras_.at(id).reset(new mujoco_rgbd_camera::MujocoDepthCamera(node, mujoco_model_, mujoco_data_, id,
                                                                                name, &stop_));
                camera_threads_.emplace_back([ObjectPtr = cameras_.at(id)] {ObjectPtr->update();});
            }
        }
    }

}  // namespace mujoco_ros_control


/**
 * @brief Main function for the Mujoco ROS2 Control plugin.
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return Exit code of the program.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("mujoco_ros2_control");
    // create the mujoco_ros2_control_plugin
    mujoco_ros2_control::MujocoRos2Control mujoco_ros2_control_plugin(node);

    // create an executor and spin the created node with it
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([ObjectPtr = &executor] { ObjectPtr->spin(); });

    while ( rclcpp::ok())
    {
        mujoco_ros2_control_plugin.update();
    }
    executor_thread.join();

    return 0;
}
