#ifndef INDYDCP3_H
#define INDYDCP3_H

#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
#else
    #include <thread>
    #include <chrono>
#endif

#include <grpcpp/grpcpp.h>
#include "google/protobuf/util/json_util.h"

#include "config.grpc.pb.h"
#include "control.grpc.pb.h"
#include "device.grpc.pb.h"
#include "rtde.grpc.pb.h"
#include "cri.grpc.pb.h"
#include "boot.grpc.pb.h"
// #include "proto/hri.grpc.pb.h"

using google::protobuf::util::MessageToJsonString;
using google::protobuf::util::JsonStringToMessage;

using Nrmk::IndyFramework::OpState;
using Nrmk::IndyFramework::JointBaseType;
using Nrmk::IndyFramework::TaskBaseType;
using Nrmk::IndyFramework::DigitalState;
using Nrmk::IndyFramework::ProgramState;
using Nrmk::IndyFramework::EndtoolState;
using Nrmk::IndyFramework::StopCategory;
using Nrmk::IndyFramework::BlendingType_Type;
using Nrmk::IndyFramework::MotionCondition_ConditionType;
using Nrmk::IndyFramework::MotionCondition_ReactionType;
using Nrmk::IndyFramework::CircularSettingType;
using Nrmk::IndyFramework::CircularMovingType;
using Nrmk::IndyFramework::TeleMethod;
using Nrmk::IndyFramework::TrajCondition;

#define LIMIT_LEVEL_MIN  1
#define LIMIT_LEVEL_MAX  9
#define LIMIT_JOG_LEVEL_MIN 1
#define LIMIT_JOG_LEVEL_MAX 3
#define LIMIT_JOG_VEL_LEVEL_DEFAULT 2
#define LIMIT_JOG_ACC_LEVEL_DEFAULT 2

#define LIMIT_JOG_VEL_RATIO_DEFAULT 15  // %
#define LIMIT_JOG_ACC_RATIO_DEFAULT 100  // %
#define LIMIT_VEL_RATIO_MAX 100  // %
#define LIMIT_VEL_RATIO_MIN 1  // %
#define LIMIT_ACC_RATIO_MAX 900  // %
#define LIMIT_ACC_RATIO_MIN 1  // %

#define LIMIT_JOG_VEL_RATIO_MIN 5  // %
#define LIMIT_JOG_VEL_RATIO_MAX 25  // %
#define LIMIT_VEL_AUTO_LEVEL_VALUE (LIMIT_VEL_RATIO_MAX - LIMIT_JOG_VEL_RATIO_MAX)/(LIMIT_LEVEL_MAX - LIMIT_JOG_LEVEL_MAX)  // %
#define LIMIT_VEL_MANUAL_LEVEL_VALUE (LIMIT_JOG_VEL_RATIO_MAX - LIMIT_JOG_VEL_RATIO_MIN)/(LIMIT_JOG_LEVEL_MAX - LIMIT_JOG_LEVEL_MIN)  // %

#define LIMIT_TASK_DISP_VEL_VALUE_DEFAULT 250  // 250mm/s
#define LIMIT_Task_DISP_ACC_VALUE_DEFAULT 250  // 250mm/s^2
#define LIMIT_TASK_DISP_VEL_VALUE_MAX 1000  // mm/s
#define LIMIT_TASK_ROT_VEL_VALUE_MAX 120  // deg/s
#define LIMIT_EXTERNAL_MOTOR_SPEED_MAX 250;  // mm/s : 3000rpm -> 50 rev/sec * 5 mm/rev -> 250 mm/s


struct DCPDICond{
    std::map<unsigned int, int> di;
    std::map<unsigned int, int> end_di;
};

struct DCPVarCond{
    std::map<unsigned int, int> int_vars;
    std::map<unsigned int, float> float_vars;
    std::map<unsigned int, bool> bool_vars;
    std::map<unsigned int, int> modbus_vars;
    std::map<unsigned int, std::vector<float>> joint_vars;
    std::map<unsigned int, std::array<float, 6>> task_vars;
};

class IndyDCP3
{
    public:
        IndyDCP3(const std::string& robot_ip = "127.0.0.1", int index = 0);
        ~IndyDCP3();

        //----------------------------------
        bool get_robot_data(Nrmk::IndyFramework::ControlData &control_data);
        bool get_control_data(Nrmk::IndyFramework::ControlData &control_data);
        bool get_control_state(Nrmk::IndyFramework::ControlData2 &control_state);
        bool get_motion_data(Nrmk::IndyFramework::MotionData &motion_data);

        bool get_servo_data(Nrmk::IndyFramework::ServoData &servo_data);
        bool get_violation_data(Nrmk::IndyFramework::ViolationData &violation_data);
        bool get_program_data(Nrmk::IndyFramework::ProgramData &program_data);

        bool get_collision_model_state(Nrmk::IndyFramework::CollisionModelState& state);
        bool get_reserved_data(Nrmk::IndyFramework::ReservedData& data);

        bool get_di(Nrmk::IndyFramework::DigitalList &di_data);
        bool get_do(Nrmk::IndyFramework::DigitalList &do_data);
        bool set_do(const Nrmk::IndyFramework::DigitalList &do_signal_list);

        bool get_ai(Nrmk::IndyFramework::AnalogList &ai_data);
        bool get_ao(Nrmk::IndyFramework::AnalogList &ao_data);
        bool set_ao(const Nrmk::IndyFramework::AnalogList &ao_signal_list);

        bool get_endtool_di(Nrmk::IndyFramework::EndtoolSignalList &endtool_di_data);
        bool get_endtool_do(Nrmk::IndyFramework::EndtoolSignalList &endtool_do_data);
        bool set_endtool_do(const Nrmk::IndyFramework::EndtoolSignalList &end_do_signal_list);

        bool get_endtool_ai(Nrmk::IndyFramework::AnalogList &endtool_ai_data);
        bool get_endtool_ao(Nrmk::IndyFramework::AnalogList &endtool_ao_data);
        bool set_endtool_ao(const Nrmk::IndyFramework::AnalogList& end_ao_signal_list);

        bool get_device_info(Nrmk::IndyFramework::DeviceInfo& device_info);
        bool get_ft_sensor_data(Nrmk::IndyFramework::FTSensorData& ft_sensor_data);
        bool stop_motion(const StopCategory stop_category);

        bool get_home_pos(Nrmk::IndyFramework::JointPos &home_jpos);

        bool commit_violation(const Nrmk::IndyFramework::ViolationRequest& request);
        bool get_rt_task_times(Nrmk::IndyFramework::TaskTimes& task_times);
        bool set_conveyor_locked_joint(int index);
        bool set_conveyor_tool_link(int index);

        bool set_locked_joint(int index);
        bool set_tool_link(int index);

        //----------------------------------
        bool movej(const std::vector<float>& jtarget,
                    const int base_type=JointBaseType::ABSOLUTE_JOINT,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float vel_ratio=LIMIT_JOG_VEL_RATIO_DEFAULT,
                    const float acc_ratio=LIMIT_JOG_ACC_RATIO_DEFAULT,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={},
                    const bool teaching_mode=false);
        
        bool movej_time(const std::vector<float>& jtarget,
                    const int base_type=JointBaseType::ABSOLUTE_JOINT,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float move_time=5.0,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={});
        
        bool movel(const std::array<float, 6>& ttarget,
                    const int base_type=TaskBaseType::ABSOLUTE_TASK,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float vel_ratio=LIMIT_JOG_VEL_RATIO_DEFAULT,
                    const float acc_ratio=LIMIT_JOG_ACC_RATIO_DEFAULT,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={},
                    const bool teaching_mode=false,
                    const bool bypass_singular=false);

        bool movel_time(const std::array<float, 6>& ttarget,
                    const int base_type=TaskBaseType::ABSOLUTE_TASK,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float move_time=5.0,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={});

        bool movec(const std::array<float, 6>& tpos1,
                    const std::array<float, 6>& tpos2,
                    const float angle=0.0,
                    const int setting_type=CircularSettingType::POINT_SET,
                    const int move_type=CircularMovingType::CONSTANT,
                    const int base_type=TaskBaseType::ABSOLUTE_TASK,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float vel_ratio=LIMIT_JOG_VEL_RATIO_DEFAULT,
                    const float acc_ratio=LIMIT_JOG_ACC_RATIO_DEFAULT,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={},
                    const bool teaching_mode=false,
                    const bool bypass_singular=false);

        bool movec_time(const std::array<float, 6>& tpos1,
                    const std::array<float, 6>& tpos2,
                    const float angle=0.0,
                    const int setting_type=CircularSettingType::POINT_SET,
                    const int move_type=CircularMovingType::CONSTANT,
                    const int base_type=TaskBaseType::ABSOLUTE_TASK,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float move_time=5.0,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={});

        bool move_gcode(const std::string& gcode_file, 
                          const bool is_smooth_mode=false, 
                          const float smooth_radius=0.0, 
                          const float vel_ratio=LIMIT_JOG_VEL_RATIO_DEFAULT, 
                          const float acc_ratio=LIMIT_JOG_ACC_RATIO_DEFAULT);

        //----------------------------------
        bool add_joint_waypoint(const std::vector<float>& waypoint);
        bool get_joint_waypoint(std::vector<std::vector<float>>& waypoints) const;
        bool clear_joint_waypoint();
        bool move_joint_waypoint(float move_time = -1);

        bool add_task_waypoint(const std::array<float, 6>& waypoint);
        bool get_task_waypoint(std::vector<std::array<float, 6>>& waypoints) const;
        bool clear_task_waypoint();
        bool move_task_waypoint(float move_time = -1);

        //----------------------------------
        bool move_home();
        bool start_teleop(const TeleMethod method);
        bool stop_teleop();

        bool movetelej(const std::vector<float>& jpos, 
                        const float vel_ratio=1.0, 
                        const float acc_ratio=1.0,
                        const TeleMethod method=TeleMethod::TELE_JOINT_ABSOLUTE);

        bool movetelel(const std::array<float, 6>& tpos, 
                        const float vel_ratio=1.0, 
                        const float acc_ratio=1.0,
                        const TeleMethod method=TeleMethod::TELE_TASK_ABSOLUTE);

        //----------------------------------
        bool inverse_kin(const std::array<float, 6>& tpos, 
                        const std::vector<float>& init_jpos, 
                        std::vector<float>& jpos);

        bool inverse_kin(const Nrmk::IndyFramework::InverseKinematicsReq& request,
                                Nrmk::IndyFramework::InverseKinematicsRes& response);

        bool set_direct_teaching(bool enable);
        bool set_simulation_mode(bool enable);
        bool recover();
        bool set_manual_recovery(bool enable);

        bool calculate_relative_pose(const std::array<float, 6>& start_pos, 
                                    const std::array<float, 6>& end_pos, 
                                    int base_type, 
                                    std::array<float, 6>& relative_pose);

        bool calculate_current_pose_rel(const std::array<float, 6>& current_pos,
                                        const std::array<float, 6>& relative_pos,
                                        int base_type,
                                        std::array<float, 6>& calculated_pose);

        //----------------------------------
        bool play_program(const std::string& prog_name = "", int prog_idx = -1);
        bool pause_program();
        bool resume_program();
        bool stop_program();

        bool get_boot_status(Nrmk::IndyFramework::BootStatus& status);

        //----------------------------------        
        bool get_bool_variable(std::vector<Nrmk::IndyFramework::BoolVariable>& bool_variables);
        bool get_int_variable(std::vector<Nrmk::IndyFramework::IntVariable>& int_variables);
        bool get_float_variable(std::vector<Nrmk::IndyFramework::FloatVariable>& float_variables);
        bool get_jpos_variable(std::vector<Nrmk::IndyFramework::JPosVariable>& jpos_variables);
        bool get_tpos_variable(std::vector<Nrmk::IndyFramework::TPosVariable>& tpos_variables);

        bool set_bool_variable(const std::vector<Nrmk::IndyFramework::BoolVariable>& bool_variables);
        bool set_int_variable(const std::vector<Nrmk::IndyFramework::IntVariable>& int_variables);
        bool set_float_variable(const std::vector<Nrmk::IndyFramework::FloatVariable>& float_variables);
        bool set_jpos_variable(const std::vector<Nrmk::IndyFramework::JPosVariable>& jpos_variables);
        bool set_tpos_variable(const std::vector<Nrmk::IndyFramework::TPosVariable>& tpos_variables);

        // Plugin Variables
        bool set_plugin_bool_variable(const std::string& name, bool value);
        bool get_plugin_bool_variable(const std::string& name, bool& value);

        bool set_plugin_int_variable(const std::string& name, int64_t value);
        bool get_plugin_int_variable(const std::string& name, int64_t& value);

        bool set_plugin_float_variable(const std::string& name, float value);
        bool get_plugin_float_variable(const std::string& name, float& value);

        bool set_plugin_jpos_variable(const std::string& name, const std::vector<float>& jpos);
        bool get_plugin_jpos_variable(const std::string& name, std::vector<float>& jpos);

        bool set_plugin_tpos_variable(const std::string& name, const std::vector<float>& tpos);
        bool get_plugin_tpos_variable(const std::string& name, std::vector<float>& tpos);

        bool set_speed_ratio(unsigned int speed_ratio);
        bool get_speed_ratio(unsigned int& speed_ratio);
        bool set_home_pos(const Nrmk::IndyFramework::JointPos& home_jpos);

        // Path/Tool/Vision/Modbus Config APIs
        bool get_path_config(Nrmk::IndyFramework::PathConfig& path_config);
        bool set_tool_list(const Nrmk::IndyFramework::ToolList& tool_list);
        bool get_tool_list(Nrmk::IndyFramework::ToolList& tool_list);
        bool set_vision_server_list(const Nrmk::IndyFramework::VisionServerList& vision_server_list);
        bool get_vision_server_list(Nrmk::IndyFramework::VisionServerList& vision_server_list);
        bool set_modbus_server_list(const Nrmk::IndyFramework::ModbusServerList& modbus_server_list);
        bool get_modbus_server_list(Nrmk::IndyFramework::ModbusServerList& modbus_server_list);

        // Environment list
        bool set_environment_list(const Nrmk::IndyFramework::EnvironmentList& environment_list);
        bool get_environment_list(Nrmk::IndyFramework::EnvironmentList& environment_list);

        // Conveyor list
        bool get_conveyor_list(Nrmk::IndyFramework::ConveyorList& conveyor_list);
        bool set_conveyor_list(const Nrmk::IndyFramework::ConveyorList& conveyor_list);

        // Compliance control joint gain
        bool set_compliance_control_joint_gain(const Nrmk::IndyFramework::ComplianceGainSet& gains);
        bool get_compliance_control_joint_gain(Nrmk::IndyFramework::ComplianceGainSet& gains);

        // Tool/Ref frame lists and custom positions
        bool get_tool_frame_list(Nrmk::IndyFramework::ToolFrameList& list);
        bool set_tool_frame_list(const Nrmk::IndyFramework::ToolFrameList& list);
        bool get_ref_frame_list(Nrmk::IndyFramework::RefFrameList& list);
        bool set_ref_frame_list(const Nrmk::IndyFramework::RefFrameList& list);
        bool get_custom_pos_list(Nrmk::IndyFramework::CustomPosList& list);
        bool set_custom_pos_list(const Nrmk::IndyFramework::CustomPosList& list);

        // Tool shape list
        bool set_tool_shape_list(const Nrmk::IndyFramework::ToolShapeList& list);
        bool get_tool_shape_list(Nrmk::IndyFramework::ToolShapeList& list);

        //----------------------------------
        bool get_ref_frame(std::array<float, 6>& fpos);
        bool set_ref_frame(const std::array<float, 6>& fpos);

        bool load_reference_frame(Nrmk::IndyFramework::RefFrameList& list);
        bool save_reference_frame(const Nrmk::IndyFramework::RefFrameList& list);

        bool set_ref_frame_planar(std::array<float, 6>& fpos_out, 
                            const std::array<float, 6>& fpos0,
                            const std::array<float, 6>& fpos1, 
                            const std::array<float, 6>& fpos2);

        bool set_tool_frame(const std::array<float, 6>& fpos);
        //----------------------------------

        bool get_friction_comp(Nrmk::IndyFramework::FrictionCompSet& friction_comp);
        bool set_friction_comp(const Nrmk::IndyFramework::FrictionCompSet& friction_comp);

        bool get_tool_property(Nrmk::IndyFramework::ToolProperties& tool_properties);
        bool set_tool_property(const Nrmk::IndyFramework::ToolProperties& tool_properties);

        bool set_mount_pos(float rot_y=0.0, float rot_z=0.0);
        bool set_mount_pos(const Nrmk::IndyFramework::MountingAngles& mounting_angles);

        bool get_mount_pos(float &rot_y, float &rot_z);
        bool get_mount_pos(Nrmk::IndyFramework::MountingAngles& mounting_angles);

        bool get_coll_sens_level(unsigned int &level);
        bool set_coll_sens_level(unsigned int level);

        bool get_coll_sens_param(Nrmk::IndyFramework::CollisionThresholds& coll_sens_param);
        bool set_coll_sens_param(const Nrmk::IndyFramework::CollisionThresholds& coll_sens_param);
        bool get_default_coll_sens_param(Nrmk::IndyFramework::CollisionThresholds& coll_sens_param);

        bool get_coll_policy(Nrmk::IndyFramework::CollisionPolicy& coll_policy);
        bool set_coll_policy(const Nrmk::IndyFramework::CollisionPolicy& coll_policy);

        bool set_simple_coll_threshold();
        bool get_collison_model_margin(Nrmk::IndyFramework::CollisionModelMargin& margin);
        bool set_collison_model_margin(const Nrmk::IndyFramework::CollisionModelMargin& margin);

        // Sensorless parameters
        bool set_sensorless_params(const Nrmk::IndyFramework::SensorlessParams& params);
        bool get_sensorless_params(Nrmk::IndyFramework::SensorlessParams& params);

        // On-start program configuration
        bool set_on_start_program_config(const Nrmk::IndyFramework::OnStartProgramConfig& config);
        bool get_on_start_program_config(Nrmk::IndyFramework::OnStartProgramConfig& config);

        bool get_safety_limits(Nrmk::IndyFramework::SafetyLimits& safety_limits);
        bool set_safety_limits(const Nrmk::IndyFramework::SafetyLimits& safety_limits);

        bool activate_sdk(const Nrmk::IndyFramework::SDKLicenseInfo& request, 
                                Nrmk::IndyFramework::SDKLicenseResp& response);

        //----------------------------------
        bool set_custom_control_mode(const int mode);
        bool get_custom_control_mode(int& mode);

        bool get_custom_control_gain(Nrmk::IndyFramework::CustomGainSet& custom_gains);
        bool set_custom_control_gain(const Nrmk::IndyFramework::CustomGainSet& custom_gains);

        bool wait_time(float time);
        bool wait_progress(int progress);
        bool wait_traj(const Nrmk::IndyFramework::TrajCondition& traj_condition);
        bool wait_radius(int radius);

        // bool wait_for_time(float wait_time = -1.0);
        bool wait_for_operation_state(int wait_op_state = -1);
        bool wait_for_motion_state(const std::string& wait_motion_state = "");

        bool start_log();
        bool end_log();

        bool get_violation_message_queue(Nrmk::IndyFramework::ViolationMessageQueue& violation_queue);
        bool get_stop_state(Nrmk::IndyFramework::StopState& stop_state);

        bool set_endtool_rs485_rx(const Nrmk::IndyFramework::EndtoolRS485Rx& request);
        // bool set_endtool_rs485_rx(const uint32_t word1, const uint32_t word2);

        bool get_endtool_rs485_rx(Nrmk::IndyFramework::EndtoolRS485Rx& rx_data);
        bool get_endtool_rs485_tx(Nrmk::IndyFramework::EndtoolRS485Tx& tx_data);
        
        // bool set_end_led_dim(const uint32_t led_dim);
        bool set_end_led_dim(const Nrmk::IndyFramework::EndLedDim& request);

        bool get_conveyor(Nrmk::IndyFramework::Conveyor& conveyor_data);

        // bool set_conveyor_by_name(const std::string& name);
        bool set_conveyor_by_name(const Nrmk::IndyFramework::Name& request);

        bool get_conveyor_state(Nrmk::IndyFramework::ConveyorState& conveyor_state);
        bool set_sander_command(const Nrmk::IndyFramework::SanderCommand::SanderType& sander_type, 
                                  const std::string& ip, 
                                  const float speed, 
                                  const bool state);
        bool get_sander_command(Nrmk::IndyFramework::SanderCommand& sander_command);

        bool get_load_factors(Nrmk::IndyFramework::GetLoadFactorsRes& load_factors_res);
        bool set_auto_mode(const bool on);
        bool check_auto_mode(Nrmk::IndyFramework::CheckAutoModeRes& check_auto_mode_res);

        bool check_reduced_mode(Nrmk::IndyFramework::CheckReducedModeRes& reduced_mode_res);
        bool get_safety_function_state(Nrmk::IndyFramework::SafetyFunctionState& safety_function_state);
        bool request_safety_function(const Nrmk::IndyFramework::SafetyFunctionState& request);
        bool get_safety_control_data(Nrmk::IndyFramework::SafetyControlData& safety_control_data);

        bool get_gripper_data(Nrmk::IndyFramework::GripperData& gripper_data);
        bool set_gripper_command(const Nrmk::IndyFramework::GripperCommand& gripper_command);

        bool activate_cri(const bool on);
        bool is_cri_active(bool& is_active);
        bool login_cri_server(const Nrmk::IndyFramework::SFDAccount& account);
        bool is_cri_login(bool& is_logged_in);

        bool set_cri_target(const Nrmk::IndyFramework::SFDTarget& target);
        bool set_cri_option(const Nrmk::IndyFramework::State& option);
        bool get_cri_proj_list(Nrmk::IndyFramework::SFDProjectList& project_list);
        bool get_cri(Nrmk::IndyFramework::CriData& cri_data);

        bool movelf(const std::array<float, 6>& ttarget,
                    const std::vector<bool>& enabledaxis,
                    const std::vector<float>& desforce,
                    const int base_type=TaskBaseType::ABSOLUTE_TASK,
                    const int blending_type=BlendingType_Type::BlendingType_Type_NONE,
                    const float blending_radius=0.0,
                    const float vel_ratio=LIMIT_JOG_VEL_RATIO_DEFAULT,
                    const float acc_ratio=LIMIT_JOG_ACC_RATIO_DEFAULT,
                    const bool const_cond=true,
                    const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                    const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                    const DCPDICond di_condition={},
                    const DCPVarCond var_condition={},
                    const bool teaching_mode=false);

        bool get_transformed_ft_sensor_data(Nrmk::IndyFramework::TransformedFTSensorData& ft_sensor_data);

        bool move_joint_traj(const std::vector<std::vector<float>>& q_list, 
                            const std::vector<std::vector<float>>& qdot_list, 
                            const std::vector<std::vector<float>>& qddot_list);
        bool move_task_traj(const std::vector<std::vector<float>>& p_list, 
                              const std::vector<std::vector<float>>& pdot_list, 
                              const std::vector<std::vector<float>>& pddot_list);

        bool move_conveyor(const bool teaching_mode, 
                            const bool bypass_singular, 
                            const float acc_ratio,
                            const bool const_cond=true,
                            const int cond_type=MotionCondition_ConditionType::MotionCondition_ConditionType_CONST_COND,
                            const int react_type=MotionCondition_ReactionType::MotionCondition_ReactionType_NONE_COND,
                            const DCPDICond di_condition={},
                            const DCPVarCond var_condition={});

        bool move_axis(const std::array<float, 3>& start_mm,
                         const std::array<float, 3>& target_mm,
                         const bool is_absolute=true,
                         const float vel_ratio=5.0f,
                         const float acc_ratio=100.0f,
                         const bool teaching_mode=false);
        
        bool forward_kin(const Nrmk::IndyFramework::ForwardKinematicsReq& request, 
                           Nrmk::IndyFramework::ForwardKinematicsRes& response);

        bool set_tact_time(const Nrmk::IndyFramework::TactTime& tact_time);
        bool get_tact_time(Nrmk::IndyFramework::TactTime& tact_time);

        bool set_ft_sensor_config(const Nrmk::IndyFramework::FTSensorDevice& sensor_config);
        bool get_ft_sensor_config(Nrmk::IndyFramework::FTSensorDevice& sensor_config);

        bool set_do_config_list(const Nrmk::IndyFramework::DOConfigList& do_config_list);
        bool get_do_config_list(Nrmk::IndyFramework::DOConfigList& do_config_list);

        bool move_recover_joint(const std::vector<float>& jtarget, 
                                const int base_type=JointBaseType::ABSOLUTE_JOINT);
        bool get_control_info(Nrmk::IndyFramework::ControlInfo& control_info);

        // Control: compliance, bus event, force mode, ping, FT zero
        bool set_compliance_mode(const Nrmk::IndyFramework::ComplianceMode& mode);
        bool get_compliance_mode(Nrmk::IndyFramework::ComplianceMode& mode);
        bool push_bus_event(const Nrmk::IndyFramework::BusEvent& event);
        bool catch_bus_event(const Nrmk::IndyFramework::CatchBusEventReq& request,
                    Nrmk::IndyFramework::BusEvent& event);
        bool set_force_mode(const Nrmk::IndyFramework::ForceModeReq& request);
        bool get_force_mode(Nrmk::IndyFramework::ForceModeReq& response);
        bool ping_from_conty();
        bool get_ft_zero();

        // Control inference data
        bool set_inference_data(const Nrmk::IndyFramework::ControlInferenceDataSet& data);
        bool get_inference_data(Nrmk::IndyFramework::ControlInferenceDataSet& data);

        bool check_aproach_retract_valid(const std::array<float, 6>& tpos, 
                                           const std::vector<float>& init_jpos, 
                                           const std::array<float, 6>& pre_tpos, 
                                           const std::array<float, 6>& post_tpos, 
                                           Nrmk::IndyFramework::CheckAproachRetractValidRes& response);

        bool get_pallet_point_list(const std::array<float, 6>& tpos, 
                                     const std::vector<float>& jpos, 
                                     const std::array<float, 6>& pre_tpos, 
                                     const std::array<float, 6>& post_tpos, 
                                     const int pallet_pattern, 
                                     const int width, 
                                     const int height, 
                                     Nrmk::IndyFramework::GetPalletPointListRes& response);

        bool play_tuning_program(const std::string& prog_name, 
                                   const int prog_idx,
                                   const Nrmk::IndyFramework::TuningSpace tuning_space, 
                                   const Nrmk::IndyFramework::TuningPrecision precision, 
                                   const uint32_t vel_level_max, 
                                   Nrmk::IndyFramework::CollisionThresholds& response);

        bool set_di_config_list(const Nrmk::IndyFramework::DIConfigList& di_config_list) ;
        bool get_di_config_list(Nrmk::IndyFramework::DIConfigList& di_config_list);

        bool set_auto_servo_off(const Nrmk::IndyFramework::AutoServoOffConfig& config);
        bool get_auto_servo_off(Nrmk::IndyFramework::AutoServoOffConfig& config);

        bool set_safety_stop_config(const Nrmk::IndyFramework::SafetyStopConfig& config);
        bool get_safety_stop_config(Nrmk::IndyFramework::SafetyStopConfig& config);

        bool get_reduced_ratio(float& ratio);
        bool get_reduced_speed(float& speed);
        bool set_reduced_speed(const float speed);

        bool set_teleop_params(const Nrmk::IndyFramework::TeleOpParams& request);
        bool get_teleop_params(Nrmk::IndyFramework::TeleOpParams& response);

        bool get_kinematics_params(Nrmk::IndyFramework::KinematicsParams& response);
        bool get_io_data(Nrmk::IndyFramework::IOData& response);

        bool wait_io(const std::vector<Nrmk::IndyFramework::DigitalSignal>& di_signal_list,
            const std::vector<Nrmk::IndyFramework::DigitalSignal>& do_signal_list,
            const std::vector<Nrmk::IndyFramework::DigitalSignal>& end_di_signal_list,
            const std::vector<Nrmk::IndyFramework::DigitalSignal>& end_do_signal_list,
            const int conjunction);

        bool set_friction_comp_state(const bool enable);
        bool get_friction_comp_state();

        bool get_teleop_device(Nrmk::IndyFramework::TeleOpDevice& device);
        bool get_teleop_state(Nrmk::IndyFramework::TeleOpState& state);
        bool connect_teleop_device(const std::string& name, 
                                    const Nrmk::IndyFramework::TeleOpDevice_TeleOpDeviceType type, 
                                    const std::string& ip, 
                                    const uint32_t port);
        bool disconnect_teleop_device();
        bool read_teleop_input(Nrmk::IndyFramework::TeleP& teleop_input);

        bool set_play_rate(const float rate);
        bool get_play_rate(float& rate);

        bool get_tele_file_list(std::vector<std::string>& files);
        bool save_tele_motion(const std::string& name);
        bool load_tele_motion(const std::string& name);
        bool delete_tele_motion(const std::string& name);

        bool enable_tele_key(const bool enable);
        bool get_pack_pos(std::vector<float>& jpos);

        bool set_joint_control_gain(const std::vector<float>& kp, const std::vector<float>& kv, const std::vector<float>& kl2);
        bool get_joint_control_gain(std::vector<float>& kp, std::vector<float>& kv, std::vector<float>& kl2);
        bool set_task_control_gain(const std::vector<float>& kp, const std::vector<float>& kv, const std::vector<float>& kl2);
        bool get_task_control_gain(std::vector<float>& kp, std::vector<float>& kv, std::vector<float>& kl2);

        bool set_impedance_control_gain(const std::vector<float>& mass, 
                                          const std::vector<float>& damping, 
                                          const std::vector<float>& stiffness, 
                                          const std::vector<float>& kl2);

        bool get_impedance_control_gain(std::vector<float>& mass, 
                                        std::vector<float>& damping, 
                                        std::vector<float>& stiffness, 
                                        std::vector<float>& kl2);

        bool set_force_control_gain(const std::vector<float>& kp, 
                                    const std::vector<float>& kv, 
                                    const std::vector<float>& kl2, 
                                    const std::vector<float>& mass, 
                                    const std::vector<float>& damping, 
                                    const std::vector<float>& stiffness, 
                                    const std::vector<float>& kpf, 
                                    const std::vector<float>& kif);

        bool get_force_control_gain(std::vector<float>& kp, 
                                    std::vector<float>& kv, 
                                    std::vector<float>& kl2, 
                                    std::vector<float>& mass, 
                                    std::vector<float>& damping, 
                                    std::vector<float>& stiffness, 
                                    std::vector<float>& kpf, 
                                    std::vector<float>& kif);

        // bool set_mount_pos(const float rot_y, const float rot_z);
        // bool get_mount_pos(float& rot_y, float& rot_z);

        bool set_brakes(const std::vector<bool>& brake_state_list);
        bool set_servo_all(const bool enable);
        bool set_servo(const uint32_t index, const bool enable);

        bool set_endtool_led_dim(const uint32_t led_dim);
        bool execute_tool(const std::string& name);
        // bool get_el5001(int& status, int& value, int& delta, float& average);
        // bool get_el5101(int& status, int& value, int& latch, int& delta, float& average);

        bool get_brake_control_style(int& style);

        bool set_conveyor_name(const std::string& name);
        bool set_conveyor_encoder(int encoder_type, int64_t channel1, int64_t channel2, int64_t sample_num,
                                float mm_per_tick, float vel_const_mmps, bool reversed);
        bool set_conveyor_trigger(int trigger_type, int64_t channel, bool detect_rise);
        bool set_conveyor_offset(float offset_mm);

        bool set_conveyor_starting_pose(const std::vector<float>& jpos, const std::vector<float>& tpos);
        bool set_conveyor_terminal_pose(const std::vector<float>& jpos, const std::vector<float>& tpos);

        bool add_photoneo_calib_point(const std::string& vision_name, double px, double py, double pz);
        bool get_photoneo_detection(const Nrmk::IndyFramework::VisionServer& vision_server, 
                                    const std::string& object, 
                                    const Nrmk::IndyFramework::VisionFrameType frame_type, 
                                    Nrmk::IndyFramework::VisionResult& result);
        bool get_photoneo_retrieval(const Nrmk::IndyFramework::VisionServer& vision_server, 
                                    const std::string& object, 
                                    const Nrmk::IndyFramework::VisionFrameType frame_type, 
                                    Nrmk::IndyFramework::VisionResult& result);

    private:
        bool _isConnected;
        unsigned int _cobotDOF;

        std::vector<std::vector<float>> _joint_waypoint;
        std::vector<std::array<float, 6>> _task_waypoint;

        std::shared_ptr<grpc::Channel> control_channel;
        std::shared_ptr<grpc::Channel> device_channel;
        std::shared_ptr<grpc::Channel> config_channel;
        std::shared_ptr<grpc::Channel> rtde_channel;
        std::shared_ptr<grpc::Channel> cri_channel;
        std::shared_ptr<grpc::Channel> boot_channel;

        std::unique_ptr<Nrmk::IndyFramework::Control::Stub> control_stub;
        std::unique_ptr<Nrmk::IndyFramework::Device::Stub> device_stub;
        std::unique_ptr<Nrmk::IndyFramework::Config::Stub> config_stub;
        std::unique_ptr<Nrmk::IndyFramework::RTDataExchange::Stub> rtde_stub;
        std::unique_ptr<Nrmk::IndyFramework::CRI::Stub> cri_stub;
        std::unique_ptr<Nrmk::IndyFramework::Boot::Stub> boot_stub;

        const std::vector<int> CONTROL_SOCKET_PORT  = {20001, 30001};
        const std::vector<int> DEVICE_SOCKET_PORT   = {20002, 30002};
        const std::vector<int> CONFIG_SOCKET_PORT   = {20003, 30003};
        const std::vector<int> RTDE_SOCKET_PORT     = {20004, 30004};
        const std::vector<int> CRI_SOCKET_PORT      = {20181, 30181};
        const std::vector<int> BOOT_SOCKET_PORT     = {20010, 30010};
};

#endif