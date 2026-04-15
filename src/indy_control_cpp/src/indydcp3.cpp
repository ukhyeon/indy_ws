// #include "indydcp3.h"
#include "indy_control_cpp/indydcp3.h"
#include <vector>

IndyDCP3::IndyDCP3(const std::string& robot_ip, int index) 
:_isConnected(false)
{
    if (index != 0 && index != 1) {
        throw std::invalid_argument("Index must be 0 or 1");
    }

    if (!_isConnected) {

        device_channel  = grpc::CreateChannel(robot_ip + ":" + std::to_string(DEVICE_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        control_channel = grpc::CreateChannel(robot_ip + ":" + std::to_string(CONTROL_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        config_channel  = grpc::CreateChannel(robot_ip + ":" + std::to_string(CONFIG_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        rtde_channel    = grpc::CreateChannel(robot_ip + ":" + std::to_string(RTDE_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        cri_channel     = grpc::CreateChannel(robot_ip + ":" + std::to_string(CRI_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());
        boot_channel    = grpc::CreateChannel(robot_ip + ":" + std::to_string(BOOT_SOCKET_PORT[index]), grpc::InsecureChannelCredentials());

        device_stub     = Nrmk::IndyFramework::Device::NewStub(device_channel);
        control_stub    = Nrmk::IndyFramework::Control::NewStub(control_channel);
        config_stub     = Nrmk::IndyFramework::Config::NewStub(config_channel);
        rtde_stub       = Nrmk::IndyFramework::RTDataExchange::NewStub(rtde_channel);
        cri_stub        = Nrmk::IndyFramework::CRI::NewStub(cri_channel);
        boot_stub       = Nrmk::IndyFramework::Boot::NewStub(boot_channel);

        Nrmk::IndyFramework::Empty request;
        Nrmk::IndyFramework::DeviceInfo response;
        grpc::ClientContext context;
        grpc::Status status = device_stub->GetDeviceInfo(&context, request, &response);
        if(status.ok()){
            _cobotDOF = response.num_joints();
            std::cout << "Cobot" << index << " DOF:" << _cobotDOF << std::endl;
            _isConnected = true;
        }
    }
}

IndyDCP3::~IndyDCP3() {}


bool IndyDCP3::get_robot_data(Nrmk::IndyFramework::ControlData &control_data) {
    /*
    Control Data:
        running_hours   -> uint32
        running_mins   -> uint32
        running_secs  -> uint32
        op_state  -> OpState
        sim_mode  -> bool
        q  -> float[6]
        qdot  -> float[6]
        p  -> float[6]
        pdot  -> float[6]
        ref_frame  -> float[6]
        tool_frame  -> float[6]
        response  -> Response
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ControlData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetControlData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetControlData/RobotData RPC failed." << std::endl;
        return false;
    }

    control_data.set_running_hours(response.running_hours());
    control_data.set_running_mins(response.running_mins());
    control_data.set_running_secs(response.running_secs());

    control_data.set_op_state(response.op_state());
    control_data.set_sim_mode(response.sim_mode());

    control_data.clear_q();
    control_data.clear_qdot();
    for(unsigned int i=0; i<_cobotDOF; i++){
        control_data.add_q(response.q(i));
        control_data.add_qdot(response.qdot(i));
    }

    for(int i=0; i<6; i++) {
        control_data.add_p(response.p(i));
        control_data.add_pdot(response.pdot(i));

        control_data.add_ref_frame(response.ref_frame(i));
        control_data.add_tool_frame(response.tool_frame(i));
    }

    return true;
}

// Environment list
bool IndyDCP3::set_environment_list(const Nrmk::IndyFramework::EnvironmentList& environment_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetEnvironmentList(&context, environment_list, &response);
    if (!status.ok()) {
        std::cerr << "SetEnvironmentList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_environment_list(Nrmk::IndyFramework::EnvironmentList& environment_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetEnvironmentList(&context, request, &environment_list);
    if (!status.ok()) {
        std::cerr << "GetEnvironmentList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_control_data(Nrmk::IndyFramework::ControlData &control_data) {
    return get_robot_data(control_data);
}

bool IndyDCP3::get_control_state(Nrmk::IndyFramework::ControlData2 &control_state) {
    /*
    Control Data:
        q  -> float[]
        qdot  -> float[]
        qddot  -> float[]
        qdes  -> float[]
        qdotdes  -> float[]
        qddotdes  -> float[]
        p  -> float[]
        pdot  -> float[]
        pddot  -> float[]
        pdes  -> float[]
        pdotdes  -> float[]
        pddotdes  -> float[]
        tau  -> float[]
        tau_act  -> float[]
        tau_ext  -> float[]
        tau_jts  -> float[]
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ControlData2 response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetControlState(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetControlState RPC failed." << std::endl;
        return false;
    }

    control_state.clear_q();
    control_state.clear_qdot();
    control_state.clear_qddot();
    control_state.clear_qdes();
    control_state.clear_qdotdes();
    control_state.clear_qddotdes();
    control_state.clear_tau();
    control_state.clear_tau_act();
    control_state.clear_tau_ext();
    control_state.clear_tau_jts();
    for(unsigned int i=0; i<_cobotDOF; i++){
        control_state.add_q(response.q(i));
        control_state.add_qdot(response.qdot(i));
        control_state.add_qddot(response.qddot(i));
        control_state.add_qdes(response.qdes(i));
        control_state.add_qdotdes(response.qdotdes(i));
        control_state.add_qddotdes(response.qddotdes(i));
        control_state.add_tau(response.tau(i));
        control_state.add_tau_act(response.tau_act(i));
        control_state.add_tau_ext(response.tau_ext(i));
        control_state.add_tau_jts(response.tau_jts(i));
    }

    for(int i=0; i<6; i++){
        control_state.add_p(response.p(i));
        control_state.add_pdot(response.pdot(i));
        control_state.add_pddot(response.pddot(i));
        control_state.add_pdes(response.pdes(i));
        control_state.add_pdotdes(response.pdotdes(i));
        control_state.add_pddotdes(response.pddotdes(i));
    }
    
    return true;
}

bool IndyDCP3::get_motion_data(Nrmk::IndyFramework::MotionData &motion_data) {
    /*
    Motion Data:
        traj_state   -> TrajState
        traj_progress   -> int32
        is_in_motion  -> bool
        is_target_reached  -> bool
        is_pausing  -> bool
        is_stopping  -> bool
        has_motion  -> bool
        speed_ratio  -> int32
        motion_id  -> int32
        remain_distance  -> float
        motion_queue_size  -> uint32
        cur_traj_progress  -> int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::MotionData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetMotionData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetMotionData RPC failed." << std::endl;
        return false;
    }

    motion_data.set_traj_state(static_cast<Nrmk::IndyFramework::TrajState>(response.traj_state()));
    motion_data.set_traj_progress(response.traj_progress());

    motion_data.set_is_in_motion(response.is_in_motion());
    motion_data.set_is_target_reached(response.is_target_reached());
    motion_data.set_is_pausing(response.is_pausing());
    motion_data.set_is_stopping(response.is_stopping());
    motion_data.set_has_motion(response.has_motion());

    motion_data.set_speed_ratio(response.speed_ratio());
    motion_data.set_motion_id(response.motion_id());
    motion_data.set_remain_distance(response.remain_distance());
    motion_data.set_motion_queue_size(response.motion_queue_size());
    motion_data.set_cur_traj_progress(response.cur_traj_progress());

    return true;
}

bool IndyDCP3::get_servo_data(Nrmk::IndyFramework::ServoData &servo_data) {
    /*
    Servo Data:
        status_codes   -> string[]
        temperatures   -> float[]
        voltages  -> float[]
        currents  -> float[]
        servo_actives  -> bool[]
        brake_actives  -> bool[]
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ServoData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetServoData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetServoData RPC failed." << std::endl;
        return false;
    }

    servo_data.clear_status_codes();
    servo_data.clear_temperatures();
    servo_data.clear_voltages();
    servo_data.clear_currents();
    servo_data.clear_servo_actives();
    servo_data.clear_brake_actives();
    for(unsigned int i=0; i<_cobotDOF; i++){
        servo_data.add_status_codes(response.status_codes(i));
        servo_data.add_temperatures(response.temperatures(i));
        servo_data.add_voltages(response.voltages(i));
        servo_data.add_currents(response.currents(i));
        servo_data.add_servo_actives(response.servo_actives(i));
        servo_data.add_brake_actives(response.brake_actives(i));
    }

    return true;
}

bool IndyDCP3::get_violation_data(Nrmk::IndyFramework::ViolationData &violation_data) {
    /*
    Violation Data:
        violation_code   -> uint64
        j_index   -> uint32
        i_args  -> int32[]
        f_args  -> float[]
        violation_str  -> string
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ViolationData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetViolationData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetViolationData RPC failed." << std::endl;
        return false;
    }

    violation_data.set_violation_code(response.violation_code());
    violation_data.set_j_index(response.j_index());

    violation_data.clear_i_args();
    for(int i=0; i<response.i_args_size(); i++){
        violation_data.add_i_args(response.i_args(i)); 
    }

    violation_data.clear_f_args();
    for(int i=0; i<response.f_args_size(); i++){
        violation_data.add_f_args(response.f_args(i)); 
    }
    violation_data.set_violation_str(response.violation_str());

    return true;
}

bool IndyDCP3::get_program_data(Nrmk::IndyFramework::ProgramData &program_data) {
    /*
    Program Data:
        program_state   -> ProgramState
        cmd_id   -> int32
        sub_cmd_id  -> int32
        running_hours  -> int32
        running_mins  -> int32
        running_secs  -> int32
        program_name  -> string
        program_alarm  -> string
        program_annotation  -> string
        speed_ratio -> int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ProgramData response;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetProgramData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetProgramData RPC failed." << std::endl;
        return false;
    }

    program_data.set_program_state(static_cast<ProgramState>(response.program_state()));
    program_data.set_cmd_id(response.cmd_id());
    program_data.set_sub_cmd_id(response.sub_cmd_id());
    program_data.set_running_hours(response.running_hours());
    program_data.set_running_mins(response.running_mins());
    program_data.set_running_secs(response.running_secs());
    program_data.set_program_name(response.program_name());
    program_data.set_program_alarm(response.program_alarm());
    program_data.set_program_annotation(response.program_annotation());
    program_data.set_speed_ratio(response.speed_ratio());

    return true;
}

bool IndyDCP3::get_collision_model_state(Nrmk::IndyFramework::CollisionModelState& state) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = rtde_stub->GetCollisionModelState(&context, request, &state);
    if (!status.ok()) {
        std::cerr << "GetCollisionModelState RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_reserved_data(Nrmk::IndyFramework::ReservedData& data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = rtde_stub->GetReservedData(&context, request, &data);
    if (!status.ok()) {
        std::cerr << "GetReservedData RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_di(Nrmk::IndyFramework::DigitalList &di_data) {
    /*
        address = uint32
        state = DigitalState 
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::DigitalList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetDI(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetDI RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    di_data = response;
    return true;
}

bool IndyDCP3::get_do(Nrmk::IndyFramework::DigitalList &do_data) {
    /*
    signals = index
    address = uint32
    state = DigitalState
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::DigitalList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetDO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetDO RPC failed." << std::endl;
        return false;
    }

    do_data = response;

    return true;
}

bool IndyDCP3::set_do(const Nrmk::IndyFramework::DigitalList &do_signal_list) {

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetDO(&context, do_signal_list, &response);
    if (!status.ok()) {
        std::cerr << "SetDO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_ai(Nrmk::IndyFramework::AnalogList &ai_data) {
    /*
        address = uint32
        voltage = int32
    */    
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::AnalogList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetAI(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetAI RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    ai_data = response;
    return true;
}

bool IndyDCP3::get_ao(Nrmk::IndyFramework::AnalogList &ao_data) {
    /*
        address = uint32
        voltage = int32
    */    
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::AnalogList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetAO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetAO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    ao_data = response;
    return true;
}

bool IndyDCP3::set_ao(const Nrmk::IndyFramework::AnalogList &ao_signal_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetAO(&context, ao_signal_list, &response);
    if (!status.ok()) {
        std::cerr << "SetAO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_endtool_di(Nrmk::IndyFramework::EndtoolSignalList &endtool_di_data) {
    /*
        state = EndtoolState
        port = char value [A,B,C]
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::EndtoolSignalList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndDI(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetEndDI RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    endtool_di_data = response;
    return true;
}

bool IndyDCP3::get_endtool_do(Nrmk::IndyFramework::EndtoolSignalList &endtool_do_data) {
    /*
        state = EndtoolState
        port = char value [A,B,C]
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::EndtoolSignalList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndDO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetEndDO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    endtool_do_data = response;
    return true;
}

bool IndyDCP3::set_endtool_do(const Nrmk::IndyFramework::EndtoolSignalList &end_do_signal_list) {
    /*
        state = EndtoolState
        port = string
    */
    grpc::ClientContext context;
    Nrmk::IndyFramework::EndtoolSignalList request = end_do_signal_list;
    Nrmk::IndyFramework::Response response;

    grpc::Status status = device_stub->SetEndDO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetEndDO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_endtool_ai(Nrmk::IndyFramework::AnalogList &endtool_ai_data) {
    /*
        address = uint32
        voltage = int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::AnalogList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndAI(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetEndAI RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    endtool_ai_data = response;
    return true;
}

bool IndyDCP3::get_endtool_ao(Nrmk::IndyFramework::AnalogList &endtool_ao_data) {
    /*
        address = uint32
        voltage = int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::AnalogList response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndAO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetEndAO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    endtool_ao_data = response;
    return true;
}

bool IndyDCP3::set_endtool_ao(const Nrmk::IndyFramework::AnalogList& end_ao_signal_list) {
    /*
        address = uint32
        voltage = int32
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetEndAO(&context, end_ao_signal_list, &response);
    if (!status.ok()) {
        std::cerr << "SetEndAO RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_device_info(Nrmk::IndyFramework::DeviceInfo& device_info) {
    /*
        Device Info:
            num_joints          -> uint32
            robot_serial        -> string
            payload             -> float
            io_board_fw_ver     -> string
            core_board_fw_vers  -> string[]
            endtool_board_fw_ver-> string
            controller_ver      -> string
            controller_detail   -> string
            controller_date     -> string
            teleop_loaded       -> bool
            calibrated          -> bool
            response            -> Response
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::DeviceInfo response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetDeviceInfo(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetDeviceInfo RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    device_info = response;
    return true;
}

bool IndyDCP3::get_ft_sensor_data(Nrmk::IndyFramework::FTSensorData& ft_sensor_data) {
    /*
    FTSensorData:
        ft_Fx   -> float ;
        ft_Fy   -> float ;
        ft_Fz   -> float ;
        ft_Tx   -> float ;
        ft_Ty   -> float ;
        ft_Tz   -> float ;
    */
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::FTSensorData response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetFTSensorData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetFTSensorData RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    ft_sensor_data = response;
    return true;
}

bool IndyDCP3::stop_motion(const StopCategory stop_category) {
    /*
        stop_category -> StopCategory
            CAT0  = 0
            CAT1  = 1
            CAT2  = 2
    */
    Nrmk::IndyFramework::StopCat request;
    request.set_category(stop_category);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->StopMotion(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "StopMotion RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_home_pos(Nrmk::IndyFramework::JointPos &home_jpos)
{
    /*
        Joint Home Position
        jpos -> double[]
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::JointPos response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetHomePosition(&context, request, &response);

    if (!status.ok()){
        std::cerr << "Get Home Pos RPC failed." << std::endl;
        return false;
    }

    home_jpos = response;
    return true;
}

bool IndyDCP3::commit_violation(const Nrmk::IndyFramework::ViolationRequest& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = device_stub->CommitViolation(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "CommitViolation RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_rt_task_times(Nrmk::IndyFramework::TaskTimes& task_times) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = device_stub->GetRTTaskTimes(&context, request, &task_times);
    if (!status.ok()) {
        std::cerr << "GetRTTaskTimes RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_locked_joint(int index) {
    Nrmk::IndyFramework::Int request;
    request.set_value(index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = device_stub->SetConveyorLockedJoint(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorLockedJoint RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_tool_link(int index) {
    Nrmk::IndyFramework::Int request;
    request.set_value(index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = device_stub->SetConveyorToolLink(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorToolLink RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_locked_joint(int index) {
    Nrmk::IndyFramework::Int request;
    request.set_value(index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetLockedJoint(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetLockedJoint RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_tool_link(int index) {
    Nrmk::IndyFramework::Int request;
    request.set_value(index);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetToolLink(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetToolLink RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::movej(const std::vector<float>& jtarget,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float vel_ratio,
                     const float acc_ratio,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition,
                     const bool teaching_mode)
{
    /*
        Joint Move:
            blending_type -> BlendingType.Type
                NONE
                OVERRIDE
                DUPLICATE
            base_type -> JointBaseType
                ABSOLUTE
                RELATIVE
            vel_ratio (0-100) -> float
            acc_ratio (0-100) -> float
            teaching_mode -> bool
    */

    Nrmk::IndyFramework::MoveJReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // tarJ.set_j_start();
        if(jtarget.size()!=_cobotDOF) {
            return false;
        }

        // set target
        // for (std::size_t i=0;i<jtarget.size();i++){
        //     request.mutable_target()->add_j_target(jtarget[i]);
        // }
        for (const auto& pos : jtarget){
            request.mutable_target()->add_j_target(pos);
        }

        // set base type
        // request.mutable_target()->set_base_type(static_cast<JointBaseType>(base_type));
        request.mutable_target()->set_base_type(
            base_type == 1 ? JointBaseType::RELATIVE_JOINT : JointBaseType::ABSOLUTE_JOINT);

        // set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        //    BlendingType_Type_NONE      = 0
        //    BlendingType_Type_OVERRIDE  = 1
        //    BlendingType_Type_DUPLICATE = 2

        request.mutable_blending()->set_blending_radius(blending_radius);

        // set post_condition
        request.mutable_post_condition()->set_const_cond(const_cond);

        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        //    CONST_COND = 0;
        //    IO_COND    = 1;
        //    VAR_COND   = 2;

        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));
        //    NONE_COND  = 0;
        //    STOP_COND  = 1;
        //    PAUSE_COND = 2;

        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_vel_ratio(vel_ratio);
        request.set_acc_ratio(acc_ratio);
        request.set_teaching_mode(teaching_mode);

        grpc::Status status = control_stub->MoveJ(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveJ RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch(std::string &err)
    {
        std::cout<<"error occur"<<err<<std::endl;
        return false;
    }
}

bool IndyDCP3::movej_time(const std::vector<float>& jtarget,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float move_time,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition)
{
    /*
        Joint Move Time:
            blending_type -> BlendingType.Type
                NONE
                OVERRIDE
                DUPLICATE
            base_type -> JointBaseType
                ABSOLUTE
                RELATIVE
            move_time -> float
    */

    Nrmk::IndyFramework::MoveJTReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // tarJ.set_j_start();
        if(jtarget.size()!=_cobotDOF) {
            return false;
        }

        // set target
        for (const auto& pos : jtarget){
            request.mutable_target()->add_j_target(pos);
        }

        // set base type
        request.mutable_target()->set_base_type(
            base_type == 1 ? JointBaseType::RELATIVE_JOINT : JointBaseType::ABSOLUTE_JOINT);

        // set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        //    BlendingType_Type_NONE      = 0
        //    BlendingType_Type_OVERRIDE  = 1
        //    BlendingType_Type_DUPLICATE = 2

        request.mutable_blending()->set_blending_radius(blending_radius);

        // set post_condition
        request.mutable_post_condition()->set_const_cond(const_cond);

        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        //    CONST_COND = 0;
        //    IO_COND    = 1;
        //    VAR_COND   = 2;

        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));
        //    NONE_COND  = 0;
        //    STOP_COND  = 1;
        //    PAUSE_COND = 2;

        // set post_condition - io condition
        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_time(move_time);

        grpc::Status status = control_stub->MoveJT(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveJT RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch(std::string &err)
    {
        std::cout<<"error occur"<<err<<std::endl;
        return false;
    }
}


bool IndyDCP3::movel(const std::array<float, 6>& ttarget,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float vel_ratio,
                     const float acc_ratio,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition,
                     const bool teaching_mode,
                     const bool bypass_singular)
{
    /*
        ttarget = [mm, mm, mm, deg, deg, deg]
        base_tye -> TaskBaseType
            ABSOLUTE
            RELATIVE
            TCP
    */

    Nrmk::IndyFramework::MoveLReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target
        // for (std::size_t i = 0; i < ttarget.size(); i++){
        //     request.mutable_target()->add_t_target(ttarget[i]);
        // }
        for (const auto& pos : ttarget){
            request.mutable_target()->add_t_target(pos);
        }

        // Set base type
        // request.mutable_target()->set_base_type(static_cast<TaskBaseType>(base_type));
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set post_condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set post_condition - IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_vel_ratio(vel_ratio);
        request.set_acc_ratio(acc_ratio);
        request.set_teaching_mode(teaching_mode);
        request.set_bypass_singular(bypass_singular);

        grpc::Status status = control_stub->MoveL(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveL RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::string& err)
    {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}


bool IndyDCP3::movel_time(const std::array<float, 6>& ttarget,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float move_time,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition)
{
    /*
        ttarget = [mm, mm, mm, deg, deg, deg]
        base_tye -> TaskBaseType
            ABSOLUTE
            RELATIVE
            TCP
    */

    Nrmk::IndyFramework::MoveLTReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target
        for (const auto& pos : ttarget){
            request.mutable_target()->add_t_target(pos);
        }

        // Set base type
        // request.mutable_target()->set_base_type(static_cast<TaskBaseType>(base_type));
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set post_condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set post_condition - IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_time(move_time);

        grpc::Status status = control_stub->MoveLT(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveLT RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::string& err)
    {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}                  

bool IndyDCP3::movec(const std::array<float, 6>& tpos1,
                     const std::array<float, 6>& tpos2,
                     const float angle,
                     const int setting_type,
                     const int move_type,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float vel_ratio,
                     const float acc_ratio,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition,
                     const bool teaching_mode,
                     const bool bypass_singular)
{
    /*
        tstart = [mm, mm, mm, deg, deg, deg]
        ttarget = [mm, mm, mm, deg, deg, deg]
    */
    Nrmk::IndyFramework::MoveCReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target positions
        // for (int i = 0; i < 6; i++)
        // {
        //     request.mutable_target()->add_t_pos0(tpos1[i]);
        //     request.mutable_target()->add_t_pos1(tpos2[i]);
        // }
        for (const auto& pos : tpos1){
            request.mutable_target()->add_t_pos0(pos);
        }
        for (const auto& pos : tpos2){
            request.mutable_target()->add_t_pos1(pos);
        }

        // Set base type
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set post condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            auto* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }
        for (const auto& [address, state] : di_condition.end_di)
        {
            auto* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set variable condition
        for (const auto& [address, value] : var_condition.bool_vars)
        {
            auto* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(address);
            bool_buff->set_value(value);
        }
        for (const auto& [address, value] : var_condition.int_vars)
        {
            auto* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(address);
            int_buff->set_value(value);
        }
        for (const auto& [address, value] : var_condition.float_vars)
        {
            auto* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(address);
            float_buff->set_value(value);
        }
        for (const auto& [address, jpos] : var_condition.joint_vars)
        {
            if (_cobotDOF != jpos.size())
                return false;

            auto* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(address);
            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, jpos[i]);
        }
        for (const auto& [address, tpos] : var_condition.task_vars)
        {
            auto* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(address);
            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, tpos[i]);
        }
        for (const auto& [address, value] : var_condition.modbus_vars)
        {
            auto* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(address);
            modbus_buff->set_value(value);
        }

        request.set_vel_ratio(vel_ratio);
        request.set_acc_ratio(acc_ratio);
        request.set_teaching_mode(teaching_mode);
        request.set_setting_type(static_cast<CircularSettingType>(setting_type));
        request.set_move_type(static_cast<CircularMovingType>(move_type));
        request.set_bypass_singular(bypass_singular);

        grpc::Status status = control_stub->MoveC(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveC RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return false;
    }
}

bool IndyDCP3::movec_time(const std::array<float, 6>& tpos1,
                     const std::array<float, 6>& tpos2,
                     const float angle,
                     const int setting_type,
                     const int move_type,
                     const int base_type,
                     const int blending_type,
                     const float blending_radius,
                     const float move_time,
                     const bool const_cond,
                     const int cond_type,
                     const int react_type,
                     DCPDICond di_condition,
                     DCPVarCond var_condition)
{
    /*
        tstart = [mm, mm, mm, deg, deg, deg]
        ttarget = [mm, mm, mm, deg, deg, deg]
    */
    Nrmk::IndyFramework::MoveCTReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target positions
        for (const auto& pos : tpos1){
            request.mutable_target()->add_t_pos0(pos);
        }
        for (const auto& pos : tpos2){
            request.mutable_target()->add_t_pos1(pos);
        }

        // Set base type
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending
        request.mutable_blending()->set_type(static_cast<BlendingType_Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set post condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            auto* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }
        for (const auto& [address, state] : di_condition.end_di)
        {
            auto* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set variable condition
        for (const auto& [address, value] : var_condition.bool_vars)
        {
            auto* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(address);
            bool_buff->set_value(value);
        }
        for (const auto& [address, value] : var_condition.int_vars)
        {
            auto* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(address);
            int_buff->set_value(value);
        }
        for (const auto& [address, value] : var_condition.float_vars)
        {
            auto* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(address);
            float_buff->set_value(value);
        }
        for (const auto& [address, jpos] : var_condition.joint_vars)
        {
            if (_cobotDOF != jpos.size())
                return false;

            auto* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(address);
            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, jpos[i]);
        }
        for (const auto& [address, tpos] : var_condition.task_vars)
        {
            auto* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(address);
            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, tpos[i]);
        }
        for (const auto& [address, value] : var_condition.modbus_vars)
        {
            auto* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(address);
            modbus_buff->set_value(value);
        }

        request.set_time(move_time);
        request.set_setting_type(static_cast<CircularSettingType>(setting_type));
        request.set_move_type(static_cast<CircularMovingType>(move_type));

        grpc::Status status = control_stub->MoveCT(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveC RPC failed." << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return false;
    }
}

bool IndyDCP3::move_gcode(const std::string& gcode_file, 
                          const bool is_smooth_mode, 
                          const float smooth_radius, 
                          const float vel_ratio, 
                          const float acc_ratio) 
{
    Nrmk::IndyFramework::MoveGcodeReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_gcode_file(gcode_file);
    request.set_is_smooth_mode(is_smooth_mode);
    request.set_smooth_radius(smooth_radius);
    request.set_vel_ratio(vel_ratio);
    request.set_acc_ratio(acc_ratio);

    grpc::Status status = control_stub->MoveGcode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "MoveGcode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::add_joint_waypoint(const std::vector<float>& waypoint) {
    _joint_waypoint.push_back(waypoint);
    return true;
}

bool IndyDCP3::get_joint_waypoint(std::vector<std::vector<float>>& waypoints) const {
    if (_joint_waypoint.empty()) {
        return false;
    }
    waypoints = _joint_waypoint;
    return true;
}

bool IndyDCP3::clear_joint_waypoint() {
    _joint_waypoint.clear();
    return true;
}

bool IndyDCP3::move_joint_waypoint(float move_time) 
{
    for (const auto& wp : _joint_waypoint) {
        if (move_time < 0) {
            movej(wp, 
                JointBaseType::ABSOLUTE_JOINT,
                BlendingType_Type::BlendingType_Type_OVERRIDE);
        } else {
            movej_time(wp, 
                JointBaseType::ABSOLUTE_JOINT,
                BlendingType_Type::BlendingType_Type_OVERRIDE,
                0.0,
                move_time);
        }
        wait_progress(100);
    }
    return true;
}

bool IndyDCP3::add_task_waypoint(const std::array<float, 6>& waypoint) {
    _task_waypoint.push_back(waypoint);
    return true;
}

bool IndyDCP3::get_task_waypoint(std::vector<std::array<float, 6>>& waypoints) const{
    if (_task_waypoint.empty()) {
        return false;
    }
    waypoints = _task_waypoint;
    return true;
}

bool IndyDCP3::clear_task_waypoint() {
    _task_waypoint.clear();
    return true;
}

bool IndyDCP3::move_task_waypoint(float move_time) 
{
    for (const auto& wp : _task_waypoint) {
        if (move_time < 0) {
            movel(wp, 
                TaskBaseType::ABSOLUTE_TASK,
                BlendingType_Type::BlendingType_Type_OVERRIDE);
        } else {
            movel_time(wp, 
                TaskBaseType::ABSOLUTE_TASK,
                BlendingType_Type::BlendingType_Type_OVERRIDE,
                0.0,
                move_time);
        }
        wait_progress(100);
    }
    return true;
}

bool IndyDCP3::move_home(){
    Nrmk::IndyFramework::JointPos home_pos;
    bool is_success = false;
    is_success = get_home_pos(home_pos);
    if (!is_success) {
        return false;
    }

    std::vector<float> j_pos;
    for (int i = 0; i < home_pos.jpos_size(); i++) {
        j_pos.push_back(home_pos.jpos(i));
    }

    is_success = movej(j_pos);
    if (is_success) {
        return true;
    } else {
        return false;
    }
}

bool IndyDCP3::start_teleop(const TeleMethod method)
{
    /*
        method:
        TELE_TASK_ABSOLUTE = 0
        TELE_TASK_RELATIVE = 1
        TELE_JOINT_ABSOLUTE = 10
        TELE_JOINT_RELATIVE = 11
    */

    Nrmk::IndyFramework::TeleOpState request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_method(method);
    request.set_mode(Nrmk::IndyFramework::TeleMode::TELE_RAW);

    grpc::Status status = control_stub->StartTeleOp(&context, request, &response);
    if (!status.ok()){
        std::cerr << "Start Tele RPC failed." << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::stop_teleop()
{
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->StopTeleOp(&context, request, &response);
    if (!status.ok()){
        std::cerr << "Stop Tele RPC failed." << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::movetelej(const std::vector<float>& jpos, 
                        const float vel_ratio, 
                        const float acc_ratio, 
                        const TeleMethod method)
{
    /*
        Joint Teleoperation
        jpos = [deg, deg, deg, deg, deg, deg]
    */

    Nrmk::IndyFramework::MoveTeleJReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    if(_cobotDOF != jpos.size()){
        return false;
    }

    request.clear_jpos();
    // for (unsigned int i=0; i<_cobotDOF; i++) {
    //    request.add_jpos(jpos[i]);
    //     // std::cout << "Joint " << jpos[i] << std::endl;
    // }
    for (const auto& pos : jpos){
        request.add_jpos(pos);
    }

    request.set_vel_ratio(vel_ratio);
    request.set_acc_ratio(acc_ratio);
    request.set_method(method);

    grpc::Status status = control_stub->MoveTeleJ(&context, request, &response);
    if (!status.ok()){
        std::cerr << "MoveTeleJ RPC failed." << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::movetelel(const std::array<float, 6>& tpos, 
                        const float vel_ratio, 
                        const float acc_ratio, 
                        const TeleMethod method)
{
    /*
        Task Teleoperation
        jpos = [mm, mm, mm, deg, deg, deg] 
    */

    Nrmk::IndyFramework::MoveTeleLReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.clear_tpos();
    // for(int i=0; i<6;i++){
    //     request.add_tpos(tpos[i]);
    // }
    for (const auto& pos : tpos){
        request.add_tpos(pos);
    }
    
    request.set_vel_ratio(vel_ratio);
    request.set_acc_ratio(acc_ratio);
    request.set_method(method);

    grpc::Status status = control_stub->MoveTeleL(&context, request, &response);
    if (!status.ok()){
        std::cerr << "MoveTeleL RPC failed." << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::inverse_kin(const std::array<float, 6>& tpos, 
                           const std::vector<float>& init_jpos, 
                           std::vector<float>& jpos)
{
    /*
        Inverse Kinematics
        tpos -> float[6]
        init_jpos -> float[]
        jpos -> float[]
    */
    
    Nrmk::IndyFramework::InverseKinematicsReq request;
    Nrmk::IndyFramework::InverseKinematicsRes response;
    grpc::ClientContext context;

    if(_cobotDOF!=init_jpos.size()){
        return false;
    }

    for (size_t i = 0; i < 6; ++i) {
        request.add_tpos(tpos[i]);
    }
    for (size_t i = 0; i < init_jpos.size(); ++i) {
        request.add_init_jpos(init_jpos[i]);
    }

    grpc::Status status = control_stub->InverseKinematics(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "InverseKinematics RPC failed." << std::endl;
        return false;
    }

    jpos.clear();
    for (int i = 0; i < response.jpos_size(); ++i) {
        jpos.push_back(response.jpos(i));
    }

    return true;
}

bool IndyDCP3::inverse_kin(const Nrmk::IndyFramework::InverseKinematicsReq& request,
                           Nrmk::IndyFramework::InverseKinematicsRes& response) 
{
    /*
        Inverse Kinematics
        request  -> InverseKinematicsReq
        response -> InverseKinematicsRes
    */
    grpc::ClientContext context;

    if (_cobotDOF != request.init_jpos_size()) {
        return false;
    }

    grpc::Status status = control_stub->InverseKinematics(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "InverseKinematics RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}


bool IndyDCP3::set_direct_teaching(bool enable)
{
    /*
        Direct Teaching
        enable -> bool
    */

    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetDirectTeaching(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "SetDirectTeaching RPC failed." << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_simulation_mode(bool enable)
{
    /*
        Simulation Mode
        enable -> bool
    */

    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetSimulationMode(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "SetSimulationMode RPC failed." << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::recover()
{
    /*
        Recover from Violation
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->Recover(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "Recover RPC failed." << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_manual_recovery(bool enable)
{
    /*
        Manual Recovery Mode
        enable -> bool
    */

    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetManualRecovery(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "SetManualRecovery RPC failed." << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::calculate_relative_pose(const std::array<float, 6>& start_pos, 
                                       const std::array<float, 6>& end_pos, 
                                       int base_type, 
                                       std::array<float, 6>& relative_pose)
{
    /*
        Calculate Relative Pose
        start_pos -> float[6]
        end_pos -> float[6]
        base_type -> int
        relative_pose -> float[6]
    */

    Nrmk::IndyFramework::CalculateRelativePoseReq request;
    Nrmk::IndyFramework::CalculateRelativePoseRes response;
    grpc::ClientContext context;

    for (const auto& sp : start_pos) {
        request.add_start_pos(sp);
    }
    for (const auto& ep : end_pos) {
        request.add_end_pos(ep);
    }

    request.set_base_type(static_cast<TaskBaseType>(base_type));

    grpc::Status status = control_stub->CalculateRelativePose(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "CalculateRelativePose RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        relative_pose[i] = response.relative_pos(i);
    }

    return true;
}

bool IndyDCP3::calculate_current_pose_rel(const std::array<float, 6>& current_pos,
                                          const std::array<float, 6>& relative_pos,
                                          int base_type,
                                          std::array<float, 6>& calculated_pose)
{
    /*
        Calculate Current Pose Relative
        current_pos -> float[6]
        relative_pos -> float[6]
        base_type -> int
        calculated_pose -> float[6]
    */

    Nrmk::IndyFramework::CalculateCurrentPoseRelReq request;
    Nrmk::IndyFramework::CalculateCurrentPoseRelRes response;
    grpc::ClientContext context;

    for (const auto& cp : current_pos) {
        request.add_current_pos(cp);
    }
    for (const auto& rp : relative_pos) {
        request.add_relative_pos(rp);
    }

    request.set_base_type(static_cast<TaskBaseType>(base_type));
    grpc::Status status = control_stub->CalculateCurrentPoseRel(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "CalculateCurrentPoseRel RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        calculated_pose[i] = response.calculated_pos(i);
    }

    return true;
}

bool IndyDCP3::play_program(const std::string& prog_name, int prog_idx)
{
    /*
        Play Program
        prog_name -> std::string
        prog_idx -> int
    */

    Nrmk::IndyFramework::Program request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_prog_name(prog_name);
    request.set_prog_idx(prog_idx);

    grpc::Status status = control_stub->PlayProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "PlayProgram RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::pause_program()
{
    /*
        Pause Program
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->PauseProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "PauseProgram RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::resume_program()
{
    /*
        Resume Program
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->ResumeProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ResumeProgram RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::stop_program()
{
    /*
        Stop Program
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->StopProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "StopProgram RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_boot_status(Nrmk::IndyFramework::BootStatus& status) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status rpc = boot_stub->GetBootStatus(&context, request, &status);
    if (!rpc.ok()) {
        std::cerr << "GetBootStatus RPC failed: " << rpc.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_speed_ratio(unsigned int speed_ratio)
{
    /*
        Set Speed Ratio
        speed_ratio -> int (0 ~ 100)
    */

    Nrmk::IndyFramework::Ratio request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_ratio(speed_ratio);

    grpc::Status status = config_stub->SetSpeedRatio(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetSpeedRatio RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_speed_ratio(unsigned int& speed_ratio)
{
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Ratio response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetSpeedRatio(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetSpeedRatio RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    speed_ratio = static_cast<unsigned int>(response.ratio());
    return true;
}

bool IndyDCP3::get_bool_variable(std::vector<Nrmk::IndyFramework::BoolVariable>& bool_variables)
{
    /*
        Get Bool Variables
        bool_variables -> std::vector<BoolVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::BoolVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetBoolVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetBoolVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    bool_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::get_int_variable(std::vector<Nrmk::IndyFramework::IntVariable>& int_variables)
{
    /*
        Get Integer Variables
        int_variables -> std::vector<IntVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::IntVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetIntVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetIntVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    int_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::get_float_variable(std::vector<Nrmk::IndyFramework::FloatVariable>& float_variables)
{
    /*
        Get Float Variables
        float_variables -> std::vector<FloatVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::FloatVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetFloatVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetFloatVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    float_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::get_jpos_variable(std::vector<Nrmk::IndyFramework::JPosVariable>& jpos_variables)
{
    /*
        Get JPos Variables
        jpos_variables -> std::vector<JPosVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::JPosVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetJPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetJPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    jpos_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::get_tpos_variable(std::vector<Nrmk::IndyFramework::TPosVariable>& tpos_variables)
{
    /*
        Get TPos Variables
        tpos_variables -> std::vector<TPosVariable>
    */

    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::TPosVars response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetTPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    tpos_variables.assign(response.variables().begin(), response.variables().end());
    return true;
}

bool IndyDCP3::set_bool_variable(const std::vector<Nrmk::IndyFramework::BoolVariable>& bool_variables)
{
    /*
        Set Bool Variables
        bool_variables -> std::vector<BoolVariable>
    */

    Nrmk::IndyFramework::BoolVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : bool_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetBoolVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetBoolVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_int_variable(const std::vector<Nrmk::IndyFramework::IntVariable>& int_variables)
{
    /*
        Set Int Variables
        int_variables -> std::vector<IntVariable>
    */

    Nrmk::IndyFramework::IntVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : int_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetIntVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetIntVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_float_variable(const std::vector<Nrmk::IndyFramework::FloatVariable>& float_variables)
{
    /*
        Set Float Variables
        float_variables -> std::vector<FloatVariable>
    */

    Nrmk::IndyFramework::FloatVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : float_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetFloatVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetFloatVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_jpos_variable(const std::vector<Nrmk::IndyFramework::JPosVariable>& jpos_variables)
{
    /*
        Set JPos Variables
        jpos_variables -> std::vector<JPosVariable>
    */

    Nrmk::IndyFramework::JPosVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : jpos_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetJPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetJPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_tpos_variable(const std::vector<Nrmk::IndyFramework::TPosVariable>& tpos_variables)
{
    /*
        Set TPos Variables
        tpos_variables -> std::vector<TPosVariable>
    */

    Nrmk::IndyFramework::TPosVars request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    for (const auto& var : tpos_variables) {
        *request.add_variables() = var;
    }

    grpc::Status status = control_stub->SetTPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetTPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_plugin_bool_variable(const std::string& name, bool value)
{
    Nrmk::IndyFramework::NamedBool request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    request.set_value(value);

    grpc::Status status = control_stub->SetPluginBoolVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginBoolVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_bool_variable(const std::string& name, bool& value)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedBool response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginBoolVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginBoolVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    value = response.value();
    return true;
}

bool IndyDCP3::set_plugin_int_variable(const std::string& name, int64_t value)
{
    Nrmk::IndyFramework::NamedInt request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    request.set_value(value);

    grpc::Status status = control_stub->SetPluginIntVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginIntVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_int_variable(const std::string& name, int64_t& value)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedInt response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginIntVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginIntVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    value = response.value();
    return true;
}

bool IndyDCP3::set_plugin_float_variable(const std::string& name, float value)
{
    Nrmk::IndyFramework::NamedFloat request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    request.set_value(value);

    grpc::Status status = control_stub->SetPluginFloatVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginFloatVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_float_variable(const std::string& name, float& value)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedFloat response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginFloatVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginFloatVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    value = response.value();
    return true;
}

bool IndyDCP3::set_plugin_jpos_variable(const std::string& name, const std::vector<float>& jpos)
{
    Nrmk::IndyFramework::NamedJointPosition request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    for (float v : jpos) request.add_jpos(v);

    grpc::Status status = control_stub->SetPluginJPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginJPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_jpos_variable(const std::string& name, std::vector<float>& jpos)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedJointPosition response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginJPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginJPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    jpos.assign(response.jpos().begin(), response.jpos().end());
    return true;
}

bool IndyDCP3::set_plugin_tpos_variable(const std::string& name, const std::vector<float>& tpos)
{
    Nrmk::IndyFramework::NamedTaskPosition request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    request.set_name(name);
    for (float v : tpos) request.add_tpos(v);

    grpc::Status status = control_stub->SetPluginTPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPluginTPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_plugin_tpos_variable(const std::string& name, std::vector<float>& tpos)
{
    Nrmk::IndyFramework::Name request;
    Nrmk::IndyFramework::NamedTaskPosition response;
    grpc::ClientContext context;

    request.set_name(name);
    grpc::Status status = control_stub->GetPluginTPosVariable(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPluginTPosVariable RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    tpos.assign(response.tpos().begin(), response.tpos().end());
    return true;
}

bool IndyDCP3::set_home_pos(const Nrmk::IndyFramework::JointPos& home_jpos) 
{
    /*
        Joint Home Position
        jpos -> double[]
    */

    Nrmk::IndyFramework::JointPos request = home_jpos;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetHomePosition(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Home Pos RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_ref_frame(std::array<float, 6>& fpos) 
{
    /*
        Reference Frame
        fpos -> float[6]
    */

    Nrmk::IndyFramework::Frame response;
    grpc::ClientContext context;
    Nrmk::IndyFramework::Empty request;

    grpc::Status status = config_stub->GetRefFrame(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetRefFrame RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        fpos[i] = response.fpos(i);
    }

    return true;
}

bool IndyDCP3::set_ref_frame(const std::array<float, 6>& fpos) 
{
    /*
        Reference Frame
        fpos -> float[6]
    */

    Nrmk::IndyFramework::Frame request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : fpos) {
        request.add_fpos(value);
    }

    grpc::Status status = config_stub->SetRefFrame(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetRefFrame RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::load_reference_frame(Nrmk::IndyFramework::RefFrameList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetRefFrameList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetRefFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::save_reference_frame(const Nrmk::IndyFramework::RefFrameList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetRefFrameList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetRefFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::set_ref_frame_planar(std::array<float, 6>& fpos_out, const std::array<float, 6>& fpos0,
                                    const std::array<float, 6>& fpos1, const std::array<float, 6>& fpos2) 
{
    /*
        Reference Frame (Planar)
        fpos_out -> float[6]
        fpos0 -> float[6]
        fpos1 -> float[6]
        fpos2 -> float[6]
    */

    Nrmk::IndyFramework::PlanarFrame request;
    Nrmk::IndyFramework::FrameResult response;
    grpc::ClientContext context;

    for (int i = 0; i < 6; ++i) {
        request.add_fpos0(fpos0[i]);
        request.add_fpos1(fpos1[i]);
        request.add_fpos2(fpos2[i]);
    }

    grpc::Status status = config_stub->SetRefFramePlanar(&context, request, &response);

    for (int i = 0; i < 6; ++i) {
        fpos_out[i] = response.fpos(i);
    }

    if (!status.ok()) {
        std::cerr << "SetRefFramePlanar RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}


bool IndyDCP3::set_tool_frame(const std::array<float, 6>& fpos) 
{
    /*
        Tool Frame
        fpos -> float[6]
    */

    Nrmk::IndyFramework::Frame request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : fpos) {
        request.add_fpos(value);
    }

    grpc::Status status = config_stub->SetToolFrame(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetToolFrame RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_path_config(Nrmk::IndyFramework::PathConfig& path_config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetPathConfig(&context, request, &path_config);
    if (!status.ok()) {
        std::cerr << "GetPathConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_friction_comp(Nrmk::IndyFramework::FrictionCompSet& friction_comp) 
{
    /*
        Friction Compensation Set:
        joint_idx   -> uint32
        control_comp_enable   -> bool
        control_comp_levels   -> int32[6]
        teaching_comp_enable   -> bool
        teaching_comp_levels   -> int32[6]
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetFrictionComp(&context, request, &friction_comp);
    if (!status.ok()) {
        std::cerr << "GetFrictionComp RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_friction_comp(const Nrmk::IndyFramework::FrictionCompSet& friction_comp) 
{
    /*
        Friction Compensation Set:
        control_comp_enable   -> bool
        control_comp_levels   -> int32[]
        teaching_comp_enable   -> bool
        teaching_comp_levels   -> int32[]
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    if ((_cobotDOF != friction_comp.control_comp_levels_size()) ||
        (_cobotDOF != friction_comp.teaching_comp_levels_size())) {
        return false;
    }

    grpc::Status status = config_stub->SetFrictionComp(&context, friction_comp, &response);
    if (!status.ok()) {
        std::cerr << "SetFrictionComp RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_tool_property(Nrmk::IndyFramework::ToolProperties& tool_properties) 
{
    /*
        Tool Properties:
        mass   -> float
        center_of_mass   -> float[3]
        inertia   -> float[6]
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetToolProperty(&context, request, &tool_properties);
    if (!status.ok()) {
        std::cerr << "GetToolProperty RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_tool_list(const Nrmk::IndyFramework::ToolList& tool_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetToolList(&context, tool_list, &response);
    if (!status.ok()) {
        std::cerr << "SetToolList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tool_list(Nrmk::IndyFramework::ToolList& tool_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetToolList(&context, request, &tool_list);
    if (!status.ok()) {
        std::cerr << "GetToolList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_vision_server_list(const Nrmk::IndyFramework::VisionServerList& vision_server_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetVisionServerList(&context, vision_server_list, &response);
    if (!status.ok()) {
        std::cerr << "SetVisionServerList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_vision_server_list(Nrmk::IndyFramework::VisionServerList& vision_server_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetVisionServerList(&context, request, &vision_server_list);
    if (!status.ok()) {
        std::cerr << "GetVisionServerList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_modbus_server_list(const Nrmk::IndyFramework::ModbusServerList& modbus_server_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetModbusServerList(&context, modbus_server_list, &response);
    if (!status.ok()) {
        std::cerr << "SetModbusServerList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_modbus_server_list(Nrmk::IndyFramework::ModbusServerList& modbus_server_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetModbusServerList(&context, request, &modbus_server_list);
    if (!status.ok()) {
        std::cerr << "GetModbusServerList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_conveyor_list(Nrmk::IndyFramework::ConveyorList& conveyor_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetConveyorList(&context, request, &conveyor_list);
    if (!status.ok()) {
        std::cerr << "GetConveyorList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_list(const Nrmk::IndyFramework::ConveyorList& conveyor_list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetConveyorList(&context, conveyor_list, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_compliance_control_joint_gain(const Nrmk::IndyFramework::ComplianceGainSet& gains) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetComplianceControlJointGain(&context, gains, &response);
    if (!status.ok()) {
        std::cerr << "SetComplianceControlJointGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_compliance_control_joint_gain(Nrmk::IndyFramework::ComplianceGainSet& gains) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetComplianceControlJointGain(&context, request, &gains);
    if (!status.ok()) {
        std::cerr << "GetComplianceControlJointGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tool_frame_list(Nrmk::IndyFramework::ToolFrameList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetToolFrameList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetToolFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_tool_frame_list(const Nrmk::IndyFramework::ToolFrameList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetToolFrameList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetToolFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_ref_frame_list(Nrmk::IndyFramework::RefFrameList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetRefFrameList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetRefFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_ref_frame_list(const Nrmk::IndyFramework::RefFrameList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetRefFrameList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetRefFrameList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_custom_pos_list(Nrmk::IndyFramework::CustomPosList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetCustomPosList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetCustomPosList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_custom_pos_list(const Nrmk::IndyFramework::CustomPosList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetCustomPosList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetCustomPosList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_tool_shape_list(const Nrmk::IndyFramework::ToolShapeList& list) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetToolShapeList(&context, list, &response);
    if (!status.ok()) {
        std::cerr << "SetToolShapeList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tool_shape_list(Nrmk::IndyFramework::ToolShapeList& list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetToolShapeList(&context, request, &list);
    if (!status.ok()) {
        std::cerr << "GetToolShapeList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::set_tool_property(const Nrmk::IndyFramework::ToolProperties& tool_properties)
{
    /*
        Tool Properties:
        mass -> float
        center_of_mass -> float[3]
        inertia -> float[6]
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetToolProperty(&context, tool_properties, &response);

    if (!status.ok()) {
        std::cerr << "SetToolProperty RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_mount_pos(float rot_y, float rot_z)
{
    Nrmk::IndyFramework::MountingAngles request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_ry(rot_y);
    request.set_rz(rot_z);

    grpc::Status status = config_stub->SetMountPos(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "SetMountPos RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_mount_pos(const Nrmk::IndyFramework::MountingAngles& mounting_angles)
{
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetMountPos(&context, mounting_angles, &response);

    if (!status.ok()) {
        std::cerr << "SetMountPos RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_mount_pos(float &rot_y, float &rot_z)
{
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::MountingAngles response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetMountPos(&context, request, &response);

    rot_y = response.ry();
    rot_z = response.rz();

    if (!status.ok()) {
        std::cerr << "GetMountPos RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_mount_pos(Nrmk::IndyFramework::MountingAngles& mounting_angles)
{
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetMountPos(&context, request, &mounting_angles);

    if (!status.ok()) {
        std::cerr << "GetMountPos RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::get_coll_sens_level(unsigned int &level) 
{
    /*
        Collision Sensitivity Level:
        level -> uint32
    */

    Nrmk::IndyFramework::CollisionSensLevel response;
    grpc::ClientContext context;
    Nrmk::IndyFramework::Empty request;

    grpc::Status status = config_stub->GetCollSensLevel(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetCollSensLevel RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    level = response.level();
    return true;
}

bool IndyDCP3::set_coll_sens_level(unsigned int level) 
{
    /*
        Collision Sensitivity Level:
        level -> uint32
    */

    Nrmk::IndyFramework::CollisionSensLevel request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_level(level);

    grpc::Status status = config_stub->SetCollSensLevel(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetCollSensLevel RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}


bool IndyDCP3::get_coll_sens_param(Nrmk::IndyFramework::CollisionThresholds& coll_sens_param) 
{
    /*
        Collision Params:
        j_torque_bases                  -> double[6]
        j_torque_tangents               -> double[6]
        t_torque_bases                  -> double[6]
        t_torque_tangents               -> double[6]
        error_bases                     -> double[6]
        error_tangents                  -> double[6]
        t_constvel_torque_bases         -> double[6]
        t_constvel_torque_tangents      -> double[6]
        t_conveyor_torque_bases         -> double[6]
        t_conveyor_torque_tangents      -> double[6]
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetCollSensParam(&context, request, &coll_sens_param);

    if (!status.ok()) {
        std::cerr << "GetCollSensParam RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_coll_sens_param(const Nrmk::IndyFramework::CollisionThresholds& coll_sens_param) 
{
    /*
        Collision Params:
        j_torque_bases                  -> double[6]
        j_torque_tangents               -> double[6]
        t_torque_bases                  -> double[6]
        t_torque_tangents               -> double[6]
        error_bases                     -> double[6]
        error_tangents                  -> double[6]
        t_constvel_torque_bases         -> double[6]
        t_constvel_torque_tangents      -> double[6]
        t_conveyor_torque_bases         -> double[6]
        t_conveyor_torque_tangents      -> double[6]
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetCollSensParam(&context, coll_sens_param, &response);

    if (!status.ok()) {
        std::cerr << "SetCollSensParam RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_default_coll_sens_param(Nrmk::IndyFramework::CollisionThresholds& coll_sens_param) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetDefaultCollSensParam(&context, request, &coll_sens_param);
    if (!status.ok()) {
        std::cerr << "GetDefaultCollSensParam RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_coll_policy(Nrmk::IndyFramework::CollisionPolicy& coll_policy) 
{
    /*
        Collision Policy:
        policy -> uint32
        sleep_time -> float
        gravity_time -> float
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetCollPolicy(&context, request, &coll_policy);

    if (!status.ok()) {
        std::cerr << "GetCollPolicy RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_coll_policy(const Nrmk::IndyFramework::CollisionPolicy& coll_policy) 
{
    /*
        Collision Policies:
        policy -> uint32
        sleep_time -> float
        gravity_time -> float
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetCollPolicy(&context, coll_policy, &response);

    if (!status.ok()) {
        std::cerr << "SetCollPolicy RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_simple_coll_threshold() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetSimpleCollThreshold(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetSimpleCollThreshold RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_collison_model_margin(Nrmk::IndyFramework::CollisionModelMargin& margin) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetCollisonModelMargin(&context, request, &margin);
    if (!status.ok()) {
        std::cerr << "GetCollisonModelMargin RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_collison_model_margin(const Nrmk::IndyFramework::CollisionModelMargin& margin) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetCollisonModelMargin(&context, margin, &response);
    if (!status.ok()) {
        std::cerr << "SetCollisonModelMargin RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::set_sensorless_params(const Nrmk::IndyFramework::SensorlessParams& params) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetSensorlessParams(&context, params, &response);
    if (!status.ok()) {
        std::cerr << "SetSensorlessParams RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_sensorless_params(Nrmk::IndyFramework::SensorlessParams& params) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetSensorlessParams(&context, request, &params);
    if (!status.ok()) {
        std::cerr << "GetSensorlessParams RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_on_start_program_config(const Nrmk::IndyFramework::OnStartProgramConfig& config) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = config_stub->SetOnStartProgramConfig(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "SetOnStartProgramConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_on_start_program_config(Nrmk::IndyFramework::OnStartProgramConfig& config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = config_stub->GetOnStartProgramConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "GetOnStartProgramConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_safety_limits(Nrmk::IndyFramework::SafetyLimits& safety_limits) 
{
    /*
        Safety Limits:
        power_limit             -> float
        power_limit_ratio       -> float
        tcp_force_limit         -> float
        tcp_force_limit_ratio   -> float
        tcp_speed_limit         -> float
        tcp_speed_limit_ratio   -> float
        joint_upper_limits      -> float[]
        joint_lower_limits      -> float[]
    */

    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetSafetyLimits(&context, request, &safety_limits);

    if (!status.ok()) {
        std::cerr << "GetSafetyLimits RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}


bool IndyDCP3::set_safety_limits(const Nrmk::IndyFramework::SafetyLimits& safety_limits) 
{
    /*
        Safety Limits:
        power_limit             -> float
        power_limit_ratio       -> float
        tcp_force_limit         -> float
        tcp_force_limit_ratio   -> float
        tcp_speed_limit         -> float
        tcp_speed_limit_ratio   -> float
        joint_upper_limits      -> float[]
        joint_lower_limits      -> float[]
    */

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetSafetyLimits(&context, safety_limits, &response);

    if (!status.ok()) {
        std::cerr << "SetSafetyLimits RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::activate_sdk(const Nrmk::IndyFramework::SDKLicenseInfo& request, 
                                Nrmk::IndyFramework::SDKLicenseResp& response) 
{
    /*
        Activate SDK
        request -> SDKLicenseInfo (input)
            - license_key -> std::string
            - expire_date -> std::string
        response -> SDKLicenseResp (output)
            - activated -> bool, True if activated
            - response (code, msg)
                - 0, 'Activated'                -> SDK Activated
                - 1, 'Invalid'                  -> Wrong key or expire date
                - 2, 'No Internet Connection'   -> Need Internet for License Verification
                - 3, 'Expired'                  -> License Expired
                - 4, 'HW_FAILURE'               -> Failed acquire HW ID to verify licens
    */

    grpc::ClientContext context;

    // Call the gRPC method
    grpc::Status status = control_stub->ActivateIndySDK(&context, request, &response);

    if (!status.ok()) {
        std::cerr << "Activate SDK RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_custom_control_mode(const int mode) {
    /*
        Set Custom Control Mode
        mode -> int
    */
    Nrmk::IndyFramework::IntMode request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_mode(mode);

    grpc::Status status = control_stub->SetCustomControlMode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Custom Control Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::get_custom_control_mode(int& mode) {
    /*
        Get Custom Control Mode
        mode -> int (output)
    */
    Nrmk::IndyFramework::IntMode response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetCustomControlMode(&context, Nrmk::IndyFramework::Empty(), &response);
    if (!status.ok()) {
        std::cerr << "Get Custom Control Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    // mode = response.mode();
    mode = static_cast<int>(response.mode());
    return true;
}

bool IndyDCP3::get_custom_control_gain(Nrmk::IndyFramework::CustomGainSet& custom_gains) {
    /*
        Get Custom Control Gain
        custom_gains -> Nrmk::IndyFramework::CustomGainSet
    */
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetCustomControlGain(&context, Nrmk::IndyFramework::Empty(), &custom_gains);
    if (!status.ok()) {
        std::cerr << "Get Custom Control Gain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_custom_control_gain(const Nrmk::IndyFramework::CustomGainSet& custom_gains) {
    /*
        Set Custom Control Gain
        custom_gains -> Nrmk::IndyFramework::CustomGainSet
    */
    grpc::ClientContext context;
    Nrmk::IndyFramework::Response response;

    grpc::Status status = config_stub->SetCustomControlGain(&context, custom_gains, &response);
    if (!status.ok()) {
        std::cerr << "Set Custom Control Gain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::wait_time(float time)
{
    Nrmk::IndyFramework::WaitTimeReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_time(time);

    // Note: set_* signal lists have been removed from WaitTimeReq in latest API.

    grpc::Status status = control_stub->WaitTime(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Wait Time RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::wait_progress(int progress)
{
    Nrmk::IndyFramework::WaitProgressReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_progress(progress);

    // Note: set_* signal lists have been removed from WaitProgressReq in latest API.

    grpc::Status status = control_stub->WaitProgress(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Wait Progress RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::wait_traj(const Nrmk::IndyFramework::TrajCondition& traj_condition)
{
    Nrmk::IndyFramework::WaitTrajReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_traj_condition(traj_condition);

    // Note: set_* signal lists have been removed from WaitTrajReq in latest API.

    grpc::Status status = control_stub->WaitTraj(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Wait Traj RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::wait_radius(int radius)
{
    Nrmk::IndyFramework::WaitRadiusReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_radius(radius);

    // Note: set_* signal lists have been removed from WaitRadiusReq in latest API.

    grpc::Status status = control_stub->WaitRadius(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Wait Radius RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

// bool IndyDCP3::wait_for_time(float wait_time) {
//     if (wait_time != -1.0) {
//         std::this_thread::sleep_for(std::chrono::duration<float>(wait_time));
//     } 
//     return true;
// }

bool IndyDCP3::wait_for_operation_state(int wait_op_state) {
    if (wait_op_state != -1) {
        bool isDone = false;
        while (!isDone) {
            Nrmk::IndyFramework::ControlData crr_control_data;
            bool is_success = get_robot_data(crr_control_data);
            if (is_success){
                if (crr_control_data.op_state() == wait_op_state){
                    std::cout << "Wait finish" << std::endl;
                    isDone = true;
                }
            }
            else{
                std::cout << "Failed to get_robot_data" << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return true;
}

bool IndyDCP3::wait_for_motion_state(const std::string& wait_motion_state) {
    std::vector<std::string> motion_list = {"is_in_motion", "is_target_reached", "is_pausing", "is_stopping", "has_motion"};

    if (!wait_motion_state.empty() && std::find(motion_list.begin(), motion_list.end(), wait_motion_state) != motion_list.end()) {
        
        bool isDone = false;
        while (!isDone) {
            Nrmk::IndyFramework::MotionData crr_motion_data;
            bool is_success = get_motion_data(crr_motion_data);

            if (wait_motion_state == "is_in_motion") {
                isDone = crr_motion_data.is_in_motion();
            } else if (wait_motion_state == "is_target_reached") {
                isDone = crr_motion_data.is_target_reached();
            } else if (wait_motion_state == "is_pausing") {
                isDone = crr_motion_data.is_pausing();
            } else if (wait_motion_state == "is_stopping") {
                isDone = crr_motion_data.is_stopping();
            } else if (wait_motion_state == "has_motion") {
                isDone = crr_motion_data.has_motion();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    else {
        std::cerr << "Invalid or empty motion state: " << wait_motion_state << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::start_log() {
    /*
        Start realtime data logging
    */
    std::vector<Nrmk::IndyFramework::IntVariable> int_vars_to_set;
    Nrmk::IndyFramework::IntVariable int_var;
    int_var.set_addr(300);
    int_var.set_value(1);
    int_vars_to_set.push_back(int_var);

    return set_int_variable(int_vars_to_set);
}

bool IndyDCP3::end_log() {
    /*
        Finish realtime data logging and save the realtime data in STEP
        saved path:
            /home/user/release/IndyDeployments/RTlog/RTLog.csv
    */
    std::vector<Nrmk::IndyFramework::IntVariable> int_vars_to_set;
    Nrmk::IndyFramework::IntVariable int_var;
    int_var.set_addr(300);
    int_var.set_value(2);
    int_vars_to_set.push_back(int_var);

    return set_int_variable(int_vars_to_set);
}

//------------- fw3.3 -------------

bool IndyDCP3::get_violation_message_queue(Nrmk::IndyFramework::ViolationMessageQueue& violation_queue) {
    /*
    Violation Data:
        violation_queue   -> ViolationData[]
    */
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetViolationMessageQueue(&context, Nrmk::IndyFramework::Empty(), &violation_queue);
    if (!status.ok()) {
        std::cerr << "Get Violation Message RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_stop_state(Nrmk::IndyFramework::StopState& stop_state) {
    /*
    Program Data:
        category   -> StopCategory
    */
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetStopState(&context, Nrmk::IndyFramework::Empty(), &stop_state);
    if (!status.ok()) {
        std::cerr << "Get Stop State RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

// bool IndyDCP3::set_endtool_rs485_rx(const uint32_t word1, const uint32_t word2) {
//     Nrmk::IndyFramework::EndtoolRS485Rx request;
//     request.set_word1(word1);
//     request.set_word2(word2);

//     Nrmk::IndyFramework::Response response;
//     grpc::ClientContext context;

//     grpc::Status status = device_stub->SetEndRS485Rx(&context, request, &response);
//     if (!status.ok()) {
//         std::cerr << "Set Endtool RS485 RX RPC failed: " << status.error_message() << std::endl;
//         return false;
//     }
//     return true;
// }

bool IndyDCP3::set_endtool_rs485_rx(const Nrmk::IndyFramework::EndtoolRS485Rx& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetEndRS485Rx(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Endtool RS485 RX RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::get_endtool_rs485_rx(Nrmk::IndyFramework::EndtoolRS485Rx& rx_data) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndRS485Rx(&context, Nrmk::IndyFramework::Empty(), &rx_data);
    if (!status.ok()) {
        std::cerr << "Get Endtool RS485 RX RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_endtool_rs485_tx(Nrmk::IndyFramework::EndtoolRS485Tx& tx_data) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetEndRS485Tx(&context, Nrmk::IndyFramework::Empty(), &tx_data);
    if (!status.ok()) {
        std::cerr << "Get Endtool RS485 TX RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

// bool IndyDCP3::set_end_led_dim(const uint32_t led_dim) {
//     Nrmk::IndyFramework::EndLedDim request;
//     request.set_led_dim(led_dim);

//     Nrmk::IndyFramework::Empty response;
//     grpc::ClientContext context;

//     grpc::Status status = device_stub->SetEndLedDim(&context, request, &response);
//     if (!status.ok()) {
//         std::cerr << "Set End LED Dim RPC failed: " << status.error_message() << std::endl;
//         return false;
//     }
//     return true;
// }

bool IndyDCP3::set_end_led_dim(const Nrmk::IndyFramework::EndLedDim& request) {
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetEndLedDim(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set End LED Dim RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::get_conveyor(Nrmk::IndyFramework::Conveyor& conveyor_data) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetConveyor(&context, Nrmk::IndyFramework::Empty(), &conveyor_data);
    if (!status.ok()) {
        std::cerr << "Get Conveyor RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

// bool IndyDCP3::set_conveyor_by_name(const std::string& name) {
//     Nrmk::IndyFramework::Name request;
//     request.set_name(name);

//     Nrmk::IndyFramework::Response response;
//     grpc::ClientContext context;

//     grpc::Status status = device_stub->SetConveyorByName(&context, request, &response);
//     if (!status.ok()) {
//         std::cerr << "Set Conveyor By Name RPC failed: " << status.error_message() << std::endl;
//         return false;
//     }
//     return true;
// }

bool IndyDCP3::set_conveyor_by_name(const Nrmk::IndyFramework::Name& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorByName(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Conveyor By Name RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::get_conveyor_state(Nrmk::IndyFramework::ConveyorState& conveyor_state) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetConveyorState(&context, Nrmk::IndyFramework::Empty(), &conveyor_state);
    if (!status.ok()) {
        std::cerr << "Get Conveyor State RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_sander_command(const Nrmk::IndyFramework::SanderCommand::SanderType& sander_type, 
                                  const std::string& ip, 
                                  const float speed, 
                                  const bool state) {
    Nrmk::IndyFramework::SanderCommand request;
    request.set_type(sander_type);
    request.set_ip(ip);
    request.set_speed(speed);
    request.set_state(state);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetSanderCommand(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Sander Command RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_sander_command(Nrmk::IndyFramework::SanderCommand& sander_command) {
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetSanderCommand(&context, Nrmk::IndyFramework::Empty(), &sander_command);
    if (!status.ok()) {
        std::cerr << "Get Sander Command RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_load_factors(Nrmk::IndyFramework::GetLoadFactorsRes& load_factors_res) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetLoadFactors(&context, request, &load_factors_res);
    if (!status.ok()) {
        std::cerr << "Get Load Factors RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_auto_mode(const bool on) {
    Nrmk::IndyFramework::SetAutoModeReq request;
    request.set_on(on);

    Nrmk::IndyFramework::SetAutoModeRes response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetAutoMode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Auto Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::check_auto_mode(Nrmk::IndyFramework::CheckAutoModeRes& check_auto_mode_res) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->CheckAutoMode(&context, request, &check_auto_mode_res);
    if (!status.ok()) {
        std::cerr << "Check Auto Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::check_reduced_mode(Nrmk::IndyFramework::CheckReducedModeRes& reduced_mode_res) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->CheckReducedMode(&context, request, &reduced_mode_res);
    if (!status.ok()) {
        std::cerr << "Check Reduced Mode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_safety_function_state(Nrmk::IndyFramework::SafetyFunctionState& safety_function_state) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetSafetyFunctionState(&context, request, &safety_function_state);
    if (!status.ok()) {
        std::cerr << "Get Safety Function State RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::request_safety_function(const Nrmk::IndyFramework::SafetyFunctionState& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->RequestSafetyFunction(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Request Safety Function RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_safety_control_data(Nrmk::IndyFramework::SafetyControlData& safety_control_data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetSafetyControlData(&context, request, &safety_control_data);
    if (!status.ok()) {
        std::cerr << "Get Safety Control Data RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_gripper_data(Nrmk::IndyFramework::GripperData& gripper_data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetGripperData(&context, request, &gripper_data);
    if (!status.ok()) {
        std::cerr << "Get Gripper Data RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_gripper_command(const Nrmk::IndyFramework::GripperCommand& gripper_command) {
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetGripperCommand(&context, gripper_command, &response);
    if (!status.ok()) {
        std::cerr << "Set Gripper Command RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::activate_cri(const bool on) {
    Nrmk::IndyFramework::State request;
    request.set_enable(on);
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->ActiveCRIVel(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ActiveCRIVel RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::is_cri_active(bool& is_active) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::State response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->IsSFDLogin(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "IsSFDLogin RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    is_active = response.enable();
    return true;
}

bool IndyDCP3::login_cri_server(const Nrmk::IndyFramework::SFDAccount& account) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->LoginSFD(&context, account, &response);
    if (!status.ok()) {
        std::cerr << "LoginSFD RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::is_cri_login(bool& is_logged_in) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::State response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->IsSFDLogin(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "IsSFDLogin RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    is_logged_in = response.enable();
    return true;
}

bool IndyDCP3::set_cri_target(const Nrmk::IndyFramework::SFDTarget& target) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->SelectSFDTarget(&context, target, &response);
    if (!status.ok()) {
        std::cerr << "SelectSFDTarget RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::set_cri_option(const Nrmk::IndyFramework::State& option) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->ActiveCRIVel(&context, option, &response);
    if (!status.ok()) {
        std::cerr << "ActiveCRIVel RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_cri_proj_list(Nrmk::IndyFramework::SFDProjectList& project_list) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = cri_stub->GetSFDProjList(&context, request, &project_list);
    if (!status.ok()) {
        std::cerr << "GetSFDProjList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_cri(Nrmk::IndyFramework::CriData& cri_data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = cri_stub->GetCRI(&context, request, &cri_data);
    if (!status.ok()) {
        std::cerr << "Get CRI RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::movelf(const std::array<float, 6>& ttarget,
                      const std::vector<bool>& enabledaxis,
                      const std::vector<float>& desforce,
                      const int base_type,
                      const int blending_type,
                      const float blending_radius,
                      const float vel_ratio,
                      const float acc_ratio,
                      const bool const_cond,
                      const int cond_type,
                      const int react_type,
                      DCPDICond di_condition,
                      DCPVarCond var_condition,
                      const bool teaching_mode)
{
    Nrmk::IndyFramework::MoveLFReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try
    {
        // Set target
        for (const auto& pos : ttarget){
            request.mutable_target()->add_t_target(pos);
        }

        // Set base type
        // request.mutable_target()->set_base_type(static_cast<TaskBaseType>(base_type));
        request.mutable_target()->set_base_type(
            base_type == 1 ? TaskBaseType::RELATIVE_TASK : TaskBaseType::ABSOLUTE_TASK);

        // Set blending type and radius
        request.mutable_blending()->set_type(static_cast<Nrmk::IndyFramework::BlendingType::Type>(blending_type));
        request.mutable_blending()->set_blending_radius(blending_radius);

        // Set desforce and enabledaxis
        for (const auto& force : desforce){
            request.add_des_force(force);
        }
        for (const auto& axis : enabledaxis){
            request.add_enabled_force(axis);
        }

        // Set post condition
        request.mutable_post_condition()->set_const_cond(const_cond);
        request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
        request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

        // Set post_condition - IO condition
        for (const auto& [address, state] : di_condition.di)
        {
            Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
            di_buff->set_address(address);
            di_buff->set_state(static_cast<DigitalState>(state));
        }

        for (const auto& [address, state] : di_condition.end_di)
        {
            Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
            enddi_buff->set_address(address);
            enddi_buff->set_state(static_cast<DigitalState>(state));
        }

        // Set post_condition - Variable condition
        for (const auto& [addr, value] : var_condition.bool_vars)
        {
            Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
            bool_buff->set_addr(addr);
            bool_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.int_vars)
        {
            Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
            int_buff->set_addr(addr);
            int_buff->set_value(value);
        }

        for (const auto& [addr, value] : var_condition.float_vars)
        {
            Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
            float_buff->set_addr(addr);
            float_buff->set_value(value);
        }

        for (const auto& [addr, values] : var_condition.joint_vars)
        {
            Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
            jpos_buff->set_addr(addr);

            if (_cobotDOF != values.size())
                return false;

            for (unsigned int i = 0; i < _cobotDOF; i++)
                jpos_buff->set_jpos(i, values[i]);
        }

        for (const auto& [addr, values] : var_condition.task_vars)
        {
            Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
            tpos_buff->set_addr(addr);

            for (int i = 0; i < 6; i++)
                tpos_buff->set_tpos(i, values[i]);
        }

        for (const auto& [addr, value] : var_condition.modbus_vars)
        {
            Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
            modbus_buff->set_addr(addr);
            modbus_buff->set_value(value);
        }

        request.set_vel_ratio(vel_ratio);
        request.set_acc_ratio(acc_ratio);
        request.set_teaching_mode(teaching_mode);

        // Call the gRPC method
        grpc::Status status = control_stub->MoveLF(&context, request, &response);
        if (!status.ok()){
            std::cerr << "MoveLF RPC failed: " << status.error_message() << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::string& err)
    {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}

bool IndyDCP3::get_transformed_ft_sensor_data(Nrmk::IndyFramework::TransformedFTSensorData& ft_sensor_data) {
    /*
    Transformed Force/Torque Sensor Data:
        ft_Fx -> float N
        ft_Fy -> float N
        ft_Fz -> float N
        ft_Tx -> float N*m
        ft_Ty -> float N*m
        ft_Tz -> float N*m
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTransformedFTSensorData(&context, request, &ft_sensor_data);
    if (!status.ok()) {
        std::cerr << "Get Transformed FT Sensor Data RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::move_joint_traj(const std::vector<std::vector<float>>& q_list, 
                               const std::vector<std::vector<float>>& qdot_list, 
                               const std::vector<std::vector<float>>& qddot_list) {
    Nrmk::IndyFramework::MoveJointTrajReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try {
        // q_list
        for (const auto& q : q_list) {
            Nrmk::IndyFramework::Vector* q_vector = request.add_q_list();
            for (const auto& value : q) {
                q_vector->add_values(value);
            }
        }

        // qdot_list
        for (const auto& qdot : qdot_list) {
            Nrmk::IndyFramework::Vector* qdot_vector = request.add_qdot_list();
            for (const auto& value : qdot) {
                qdot_vector->add_values(value);
            }
        }

        // qddot_list
        for (const auto& qddot : qddot_list) {
            Nrmk::IndyFramework::Vector* qddot_vector = request.add_qddot_list();
            for (const auto& value : qddot) {
                qddot_vector->add_values(value);
            }
        }

        grpc::Status status = control_stub->MoveJointTraj(&context, request, &response);
        if (!status.ok()) {
            std::cerr << "MoveJointTraj RPC failed: " << status.error_message() << std::endl;
            return false;
        }
        return true;
    } catch (const std::string& err) {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}

bool IndyDCP3::move_task_traj(const std::vector<std::vector<float>>& p_list, 
                              const std::vector<std::vector<float>>& pdot_list, 
                              const std::vector<std::vector<float>>& pddot_list) {
    Nrmk::IndyFramework::MoveTaskTrajReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    try {
        // p_list
        for (const auto& p : p_list) {
            Nrmk::IndyFramework::Vector* p_vector = request.add_p_list();
            for (const auto& value : p) {
                p_vector->add_values(value);
            }
        }

        // pdot_list
        for (const auto& pdot : pdot_list) {
            Nrmk::IndyFramework::Vector* pdot_vector = request.add_pdot_list();
            for (const auto& value : pdot) {
                pdot_vector->add_values(value);
            }
        }

        // pddot_list
        for (const auto& pddot : pddot_list) {
            Nrmk::IndyFramework::Vector* pddot_vector = request.add_pddot_list();
            for (const auto& value : pddot) {
                pddot_vector->add_values(value);
            }
        }

        grpc::Status status = control_stub->MoveTaskTraj(&context, request, &response);
        if (!status.ok()) {
            std::cerr << "MoveTaskTraj RPC failed: " << status.error_message() << std::endl;
            return false;
        }
        return true;
    } catch (const std::string& err) {
        std::cerr << "Error occurred: " << err << std::endl;
        return false;
    }
}

bool IndyDCP3::move_conveyor(const bool teaching_mode, 
                             const bool bypass_singular, 
                             const float acc_ratio,
                             const bool const_cond,
                             const int cond_type,
                             const int react_type,
                             DCPDICond di_condition,
                             DCPVarCond var_condition) 
{
    Nrmk::IndyFramework::MoveConveyorReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_teaching_mode(teaching_mode);
    request.set_bypass_singular(bypass_singular);
    request.set_acc_ratio(acc_ratio);

    // Set post condition
    request.mutable_post_condition()->set_const_cond(const_cond);
    request.mutable_post_condition()->set_type_cond(static_cast<MotionCondition_ConditionType>(cond_type));
    request.mutable_post_condition()->set_type_react(static_cast<MotionCondition_ReactionType>(react_type));

    // Set post_condition - IO condition
    for (const auto& [address, state] : di_condition.di)
    {
        Nrmk::IndyFramework::DigitalSignal* di_buff = request.mutable_post_condition()->mutable_io_cond()->add_di();
        di_buff->set_address(address);
        di_buff->set_state(static_cast<DigitalState>(state));
    }

    for (const auto& [address, state] : di_condition.end_di)
    {
        Nrmk::IndyFramework::DigitalSignal* enddi_buff = request.mutable_post_condition()->mutable_io_cond()->add_end_di();
        enddi_buff->set_address(address);
        enddi_buff->set_state(static_cast<DigitalState>(state));
    }

    // Set post_condition - Variable condition
    for (const auto& [addr, value] : var_condition.bool_vars)
    {
        Nrmk::IndyFramework::BoolVariable* bool_buff = request.mutable_post_condition()->mutable_var_cond()->add_b_vars();
        bool_buff->set_addr(addr);
        bool_buff->set_value(value);
    }

    for (const auto& [addr, value] : var_condition.int_vars)
    {
        Nrmk::IndyFramework::IntVariable* int_buff = request.mutable_post_condition()->mutable_var_cond()->add_i_vars();
        int_buff->set_addr(addr);
        int_buff->set_value(value);
    }

    for (const auto& [addr, value] : var_condition.float_vars)
    {
        Nrmk::IndyFramework::FloatVariable* float_buff = request.mutable_post_condition()->mutable_var_cond()->add_f_vars();
        float_buff->set_addr(addr);
        float_buff->set_value(value);
    }

    for (const auto& [addr, values] : var_condition.joint_vars)
    {
        Nrmk::IndyFramework::JPosVariable* jpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_j_vars();
        jpos_buff->set_addr(addr);

        if (_cobotDOF != values.size())
            return false;

        for (unsigned int i = 0; i < _cobotDOF; i++)
            jpos_buff->set_jpos(i, values[i]);
    }

    for (const auto& [addr, values] : var_condition.task_vars)
    {
        Nrmk::IndyFramework::TPosVariable* tpos_buff = request.mutable_post_condition()->mutable_var_cond()->add_t_vars();
        tpos_buff->set_addr(addr);

        for (int i = 0; i < 6; i++)
            tpos_buff->set_tpos(i, values[i]);
    }

    for (const auto& [addr, value] : var_condition.modbus_vars)
    {
        Nrmk::IndyFramework::ModbusVariable* modbus_buff = request.mutable_post_condition()->mutable_var_cond()->add_m_vars();
        modbus_buff->set_addr(addr);
        modbus_buff->set_value(value);
    }

    grpc::Status status = control_stub->MoveConveyor(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "MoveConveyor RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::move_axis(const std::array<float, 3>& start_mm,
                         const std::array<float, 3>& target_mm,
                         const bool is_absolute,
                         const float vel_ratio,
                         const float acc_ratio,
                         const bool teaching_mode) 
{
    Nrmk::IndyFramework::MoveAxisReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    // start and target positions
    for (const auto& pos : start_mm) {
        request.add_start_mm(pos);
    }
    for (const auto& pos : target_mm) {
        request.add_target_mm(pos);
    }

    // velocity and acceleration ratios
    request.set_vel_percentage(vel_ratio);
    request.set_acc_percentage(acc_ratio);

    // movement type and teaching mode
    request.set_is_absolute(is_absolute);
    request.set_teaching_mode(teaching_mode);

    grpc::Status status = control_stub->MoveLinearAxis(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "MoveLinearAxis RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::forward_kin(const Nrmk::IndyFramework::ForwardKinematicsReq& request, 
                           Nrmk::IndyFramework::ForwardKinematicsRes& response) 
{
    grpc::ClientContext context;

    grpc::Status status = control_stub->ForwardKinematics(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ForwardKinematics RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    return true;
}

bool IndyDCP3::set_tact_time(const Nrmk::IndyFramework::TactTime& tact_time) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetTactTime(&context, tact_time, &response);
    if (!status.ok()) {
        std::cerr << "SetTactTime RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_tact_time(Nrmk::IndyFramework::TactTime& tact_time) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTactTime(&context, request, &tact_time);
    if (!status.ok()) {
        std::cerr << "GetTactTime RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_ft_sensor_config(const Nrmk::IndyFramework::FTSensorDevice& sensor_config) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetFTSensorConfig(&context, sensor_config, &response);
    if (!status.ok()) {
        std::cerr << "SetFTSensorConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_ft_sensor_config(Nrmk::IndyFramework::FTSensorDevice& sensor_config) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetFTSensorConfig(&context, request, &sensor_config);
    if (!status.ok()) {
        std::cerr << "GetFTSensorConfig RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_do_config_list(const Nrmk::IndyFramework::DOConfigList& do_config_list) {
    /*
        DO Configuration List
        {
            'do_configs': [
                {
                    'state_code': 2,
                    'state_name': "name",
                    'onSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                    'offSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                }
            ]
        }
    */
    
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetDOConfigList(&context, do_config_list, &response);
    if (!status.ok()) {
        std::cerr << "SetDOConfigList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_do_config_list(Nrmk::IndyFramework::DOConfigList& do_config_list) {
    /*
        DO Configuration List
        {
            'do_configs': [
                {
                    'state_code': 2,
                    'state_name': "name",
                    'onSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                    'offSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                }
            ]
        }
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetDOConfigList(&context, request, &do_config_list);
    if (!status.ok()) {
        std::cerr << "GetDOConfigList RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::move_recover_joint(const std::vector<float>& jtarget, 
                                const int base_type) {
    /*
        Move recover joint
        jtarget = [deg, deg, deg, deg, deg, deg]    
    */

    Nrmk::IndyFramework::TargetJ request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& joint_pos : jtarget) {
        request.add_j_target(joint_pos);
    }

    request.set_base_type(base_type == 1 ? JointBaseType::RELATIVE_JOINT : JointBaseType::ABSOLUTE_JOINT);

    grpc::Status status = control_stub->MoveRecoverJoint(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "MoveRecoverJoint RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_control_info(Nrmk::IndyFramework::ControlInfo& control_info) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetControlInfo(&context, request, &control_info);
    if (!status.ok()) {
        std::cerr << "GetControlInfo RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_compliance_mode(const Nrmk::IndyFramework::ComplianceMode& mode) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->SetComplianceMode(&context, mode, &response);
    if (!status.ok()) {
        std::cerr << "SetComplianceMode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_compliance_mode(Nrmk::IndyFramework::ComplianceMode& mode) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = control_stub->GetComplianceMode(&context, request, &mode);
    if (!status.ok()) {
        std::cerr << "GetComplianceMode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::push_bus_event(const Nrmk::IndyFramework::BusEvent& event) {
    Nrmk::IndyFramework::State response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->PushBusEvent(&context, event, &response);
    if (!status.ok()) {
        std::cerr << "PushBusEvent RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.enable();
}

bool IndyDCP3::catch_bus_event(const Nrmk::IndyFramework::CatchBusEventReq& request,
                               Nrmk::IndyFramework::BusEvent& event) {
    grpc::ClientContext context;
    grpc::Status status = control_stub->CatchBusEvent(&context, request, &event);
    if (!status.ok()) {
        std::cerr << "CatchBusEvent RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_force_mode(const Nrmk::IndyFramework::ForceModeReq& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->SetForceMode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetForceMode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_force_mode(Nrmk::IndyFramework::ForceModeReq& response) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = control_stub->GetForceMode(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetForceMode RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::ping_from_conty() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->PingFromConty(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "PingFromConty RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_ft_zero() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->FTZero(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "FTZero RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_inference_data(const Nrmk::IndyFramework::ControlInferenceDataSet& data) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;
    grpc::Status status = control_stub->SetControlInferenceData(&context, data, &response);
    if (!status.ok()) {
        std::cerr << "SetControlInferenceData RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.code() == 0;
}

bool IndyDCP3::get_inference_data(Nrmk::IndyFramework::ControlInferenceDataSet& data) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;
    grpc::Status status = control_stub->GetControlInferenceData(&context, request, &data);
    if (!status.ok()) {
        std::cerr << "GetControlInferenceData RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::check_aproach_retract_valid(const std::array<float, 6>& tpos, 
                                           const std::vector<float>& init_jpos, 
                                           const std::array<float, 6>& pre_tpos, 
                                           const std::array<float, 6>& post_tpos, 
                                           Nrmk::IndyFramework::CheckAproachRetractValidRes& response)
{
    Nrmk::IndyFramework::CheckAproachRetractValidReq request;
    grpc::ClientContext context;

    for (const auto& pos : tpos) {
        request.add_tpos(pos);
    }
    for (const auto& jpos : init_jpos) {
        request.add_init_jpos(jpos);
    }
    for (const auto& pos : pre_tpos) {
        request.add_pre_tpos(pos);
    }
    for (const auto& pos : post_tpos) {
        request.add_post_tpos(pos);
    }

    grpc::Status status = control_stub->CheckAproachRetractValid(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Check Aproach Retract Valid RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return response.is_valid();
}

bool IndyDCP3::get_pallet_point_list(const std::array<float, 6>& tpos, 
                                     const std::vector<float>& jpos, 
                                     const std::array<float, 6>& pre_tpos, 
                                     const std::array<float, 6>& post_tpos, 
                                     const int pallet_pattern, 
                                     const int width, 
                                     const int height, 
                                     Nrmk::IndyFramework::GetPalletPointListRes& response)
{
    Nrmk::IndyFramework::GetPalletPointListReq request;
    grpc::ClientContext context;

    for (const auto& pos : tpos) {
        request.add_tpos(pos);
    }
    for (const auto& joint_pos : jpos) {
        request.add_jpos(joint_pos);
    }
    for (const auto& pos : pre_tpos) {
        request.add_pre_tpos(pos);
    }
    for (const auto& pos : post_tpos) {
        request.add_post_tpos(pos);
    }
    request.set_pallet_pattern(pallet_pattern);
    request.set_width(width);
    request.set_height(height);

    grpc::Status status = control_stub->GetPalletPointList(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get Pallet Point List RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::play_tuning_program(const std::string& prog_name, 
                                   const int prog_idx, 
                                   const Nrmk::IndyFramework::TuningSpace tuning_space, 
                                   const Nrmk::IndyFramework::TuningPrecision precision, 
                                   const uint32_t vel_level_max, 
                                   Nrmk::IndyFramework::CollisionThresholds& response)
{
    Nrmk::IndyFramework::TuningProgram request;
    grpc::ClientContext context;

    request.mutable_program()->set_prog_name(prog_name);
    request.mutable_program()->set_prog_idx(prog_idx);
    request.set_tuning_space(tuning_space);
    request.set_precision(precision);
    request.set_vel_level_max(vel_level_max);

    grpc::Status status = control_stub->PlayTuningProgram(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Play Tuning Program RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_di_config_list(const Nrmk::IndyFramework::DIConfigList& di_config_list) {
    /*
        DI Configuration List
        {
            'di_configs': [
                {
                    'function_code': 2,
                    'function_name': "name",
                    'triggerSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                    'successSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                    'failureSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                }
            ]
        }
    */
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetDIConfigList(&context, di_config_list, &response);
    if (!status.ok()) {
        std::cerr << "Set DI Config List RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_di_config_list(Nrmk::IndyFramework::DIConfigList& di_config_list) {
    /*
        DI Configuration List
        {
            'di_configs': [
                {
                    'function_code': 2,
                    'function_name': "name",
                    'triggerSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                    'successSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                    'failureSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                }
            ]
        }
    */
    
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetDIConfigList(&context, request, &di_config_list);
    if (!status.ok()) {
        std::cerr << "Get DI Config List RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_auto_servo_off(const Nrmk::IndyFramework::AutoServoOffConfig& config) {
    /*
        Auto Servo-Off Config
        enable -> bool
        time -> float
    */
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetAutoServoOff(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "Set Auto Servo Off Config RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_auto_servo_off(Nrmk::IndyFramework::AutoServoOffConfig& config) {
    /*
        Auto Servo-Off Config
        enable -> bool
        time -> float
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetAutoServoOff(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "Get Auto Servo Off Config RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_safety_stop_config(const Nrmk::IndyFramework::SafetyStopConfig& config) {
    /*
        Safety Stop Category:
        jpos_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        jvel_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        jtau_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        tvel_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        tforce_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        power_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
    */
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetSafetyStopConfig(&context, config, &response);
    if (!status.ok()) {
        std::cerr << "Set Safety Stop Config RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_safety_stop_config(Nrmk::IndyFramework::SafetyStopConfig& config) {
    /*
        Safety Stop Category:
        joint_position_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        joint_speed_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        joint_torque_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        tcp_speed_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        tcp_force_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        power_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
    */
    
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetSafetyStopConfig(&context, request, &config);
    if (!status.ok()) {
        std::cerr << "Get Safety Stop Config RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_reduced_ratio(float& ratio) {
    Nrmk::IndyFramework::GetReducedRatioRes response;
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetReducedRatio(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get Reduced Ratio RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    ratio = response.ratio();
    return true;
}

bool IndyDCP3::get_reduced_speed(float& speed) {
    Nrmk::IndyFramework::GetReducedSpeedRes response;
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetReducedSpeed(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get Reduced Speed RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    speed = response.speed();
    return true;
}

bool IndyDCP3::set_reduced_speed(const float speed) {
    Nrmk::IndyFramework::SetReducedSpeedReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_speed(speed);

    grpc::Status status = config_stub->SetReducedSpeed(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set Reduced Speed RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_teleop_params(const Nrmk::IndyFramework::TeleOpParams& request) {
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->SetTeleOpParams(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Set TeleOp Params RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_teleop_params(Nrmk::IndyFramework::TeleOpParams& response) {
    /*
        IO Data:
        smooth_factor   -> float
        cutoff_freq   -> float
        error_gain  -> float
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetTeleOpParams(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get TeleOp Params RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_kinematics_params(Nrmk::IndyFramework::KinematicsParams& response) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetKinematicsParams(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get Kinematics Params RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_io_data(Nrmk::IndyFramework::IOData& response) {
    /*
        IO Data:
        di   -> DigitalSignal[]
        do   -> DigitalSignal[]
        ai  -> AnalogSignal[]
        ao  -> AnalogSignal[]
        end_di  -> EndtoolSignal[]
        end_do  -> EndtoolSignal[]
        end_ai  -> AnalogSignal[]
        end_ao  -> AnalogSignal[]
        response  -> Response
    */
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = rtde_stub->GetIOData(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "Get IO Data RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::wait_io(
    const std::vector<Nrmk::IndyFramework::DigitalSignal>& di_signal_list,
    const std::vector<Nrmk::IndyFramework::DigitalSignal>& do_signal_list,
    const std::vector<Nrmk::IndyFramework::DigitalSignal>& end_di_signal_list,
    const std::vector<Nrmk::IndyFramework::DigitalSignal>& end_do_signal_list,
    const int conjunction)
{
    Nrmk::IndyFramework::WaitIOReq request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& signal : di_signal_list) {
        auto* di = request.add_di_list();
        di->set_address(signal.address());
        di->set_state(static_cast<Nrmk::IndyFramework::DigitalState>(signal.state()));
    }

    for (const auto& signal : do_signal_list) {
        auto* do_signal = request.add_do_list();
        do_signal->set_address(signal.address());
        do_signal->set_state(static_cast<Nrmk::IndyFramework::DigitalState>(signal.state()));
    }

    for (const auto& signal : end_di_signal_list) {
        auto* end_di = request.add_end_di_list();
        end_di->set_address(signal.address());
        end_di->set_state(static_cast<Nrmk::IndyFramework::DigitalState>(signal.state()));
    }

    for (const auto& signal : end_do_signal_list) {
        auto* end_do = request.add_end_do_list();
        end_do->set_address(signal.address());
        end_do->set_state(static_cast<Nrmk::IndyFramework::DigitalState>(signal.state()));
    }

    request.set_conjunction(conjunction);

    // Note: set_* lists have been removed from WaitIOReq in latest API.

    grpc::Status status = control_stub->WaitIO(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "WaitIO RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}


bool IndyDCP3::set_friction_comp_state(const bool enable) {
    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetFrictionCompensation(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetFrictionCompensation RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_friction_comp_state() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::State response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetFrictionCompensationState(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetFrictionCompensationState RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    bool is_enabled = response.enable();
    return is_enabled;
}

bool IndyDCP3::get_teleop_device(Nrmk::IndyFramework::TeleOpDevice& device) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTeleOpDevice(&context, request, &device);
    if (!status.ok()) {
        std::cerr << "GetTeleOpDevice RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_teleop_state(Nrmk::IndyFramework::TeleOpState& state) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTeleOpState(&context, request, &state);
    if (!status.ok()) {
        std::cerr << "GetTeleOpState RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::connect_teleop_device(const std::string& name, 
                                     const Nrmk::IndyFramework::TeleOpDevice_TeleOpDeviceType type, 
                                     const std::string& ip, 
                                     const uint32_t port) {
    Nrmk::IndyFramework::TeleOpDevice request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    request.set_name(name);
    request.set_type(type);
    request.set_ip(ip);
    request.set_port(port);

    grpc::Status status = control_stub->ConnectTeleOpDevice(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ConnectTeleOpDevice RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::disconnect_teleop_device() {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->DisConnectTeleOpDevice(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "DisConnectTeleOpDevice RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::read_teleop_input(Nrmk::IndyFramework::TeleP& teleop_input) {
    Nrmk::IndyFramework::Empty request;
    grpc::ClientContext context;

    grpc::Status status = control_stub->ReadTeleOpInput(&context, request, &teleop_input);
    if (!status.ok()) {
        std::cerr << "ReadTeleOpInput RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_play_rate(const float rate) {
    Nrmk::IndyFramework::TelePlayRate request;
    request.set_rate(rate);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SetPlayRate(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetPlayRate RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_play_rate(float& rate) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::TelePlayRate response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetPlayRate(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPlayRate RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    rate = response.rate();
    return true;
}

bool IndyDCP3::get_tele_file_list(std::vector<std::string>& files) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::TeleOpFileList response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->GetTeleFileList(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetTeleFileList RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    files.assign(response.files().begin(), response.files().end());
    return true;
}

bool IndyDCP3::save_tele_motion(const std::string& name) {
    Nrmk::IndyFramework::TeleFileReq request;
    request.set_name(name);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->SaveTeleMotion(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SaveTeleMotion RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::load_tele_motion(const std::string& name) {
    Nrmk::IndyFramework::TeleFileReq request;
    request.set_name(name);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->LoadTeleMotion(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "LoadTeleMotion RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::delete_tele_motion(const std::string& name) {
    Nrmk::IndyFramework::TeleFileReq request;
    request.set_name(name);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->DeleteTeleMotion(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "DeleteTeleMotion RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::enable_tele_key(const bool enable) {
    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = control_stub->EnableTeleKey(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "EnableTeleKey RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_pack_pos(std::vector<float>& jpos) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::JointPos response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetPackPosition(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetPackPosition RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    jpos.clear();
    for (int i = 0; i < response.jpos_size(); ++i) {
        jpos.push_back(response.jpos(i));
    }
    return true;
}

bool IndyDCP3::set_joint_control_gain(const std::vector<float>& kp, const std::vector<float>& kv, const std::vector<float>& kl2) {
    Nrmk::IndyFramework::JointGainSet request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : kp) request.add_kp(value);
    for (const auto& value : kv) request.add_kv(value);
    for (const auto& value : kl2) request.add_kl2(value);

    grpc::Status status = config_stub->SetJointControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetJointControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_joint_control_gain(std::vector<float>& kp, std::vector<float>& kv, std::vector<float>& kl2) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::JointGainSet response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetJointControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetJointControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    kp.assign(response.kp().begin(), response.kp().end());
    kv.assign(response.kv().begin(), response.kv().end());
    kl2.assign(response.kl2().begin(), response.kl2().end());
    return true;
}

bool IndyDCP3::set_task_control_gain(const std::vector<float>& kp, const std::vector<float>& kv, const std::vector<float>& kl2) {
    Nrmk::IndyFramework::TaskGainSet request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : kp) request.add_kp(value);
    for (const auto& value : kv) request.add_kv(value);
    for (const auto& value : kl2) request.add_kl2(value);

    grpc::Status status = config_stub->SetTaskControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetTaskControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_task_control_gain(std::vector<float>& kp, std::vector<float>& kv, std::vector<float>& kl2) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::TaskGainSet response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetTaskControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetTaskControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    kp.assign(response.kp().begin(), response.kp().end());
    kv.assign(response.kv().begin(), response.kv().end());
    kl2.assign(response.kl2().begin(), response.kl2().end());
    return true;
}

bool IndyDCP3::set_impedance_control_gain(const std::vector<float>& mass, 
                                          const std::vector<float>& damping, 
                                          const std::vector<float>& stiffness, 
                                          const std::vector<float>& kl2) {
    Nrmk::IndyFramework::ImpedanceGainSet request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : mass) request.add_mass(value);
    for (const auto& value : damping) request.add_damping(value);
    for (const auto& value : stiffness) request.add_stiffness(value);
    for (const auto& value : kl2) request.add_kl2(value);

    grpc::Status status = config_stub->SetImpedanceControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetImpedanceControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_impedance_control_gain(std::vector<float>& mass, 
                                          std::vector<float>& damping, 
                                          std::vector<float>& stiffness, 
                                          std::vector<float>& kl2) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ImpedanceGainSet response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetImpedanceControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetImpedanceControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    mass.assign(response.mass().begin(), response.mass().end());
    damping.assign(response.damping().begin(), response.damping().end());
    stiffness.assign(response.stiffness().begin(), response.stiffness().end());
    kl2.assign(response.kl2().begin(), response.kl2().end());
    return true;
}

bool IndyDCP3::set_force_control_gain(const std::vector<float>& kp, 
                                      const std::vector<float>& kv, 
                                      const std::vector<float>& kl2, 
                                      const std::vector<float>& mass, 
                                      const std::vector<float>& damping, 
                                      const std::vector<float>& stiffness, 
                                      const std::vector<float>& kpf, 
                                      const std::vector<float>& kif) {
    Nrmk::IndyFramework::ForceGainSet request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (const auto& value : kp) request.add_kp(value);
    for (const auto& value : kv) request.add_kv(value);
    for (const auto& value : kl2) request.add_kl2(value);
    for (const auto& value : mass) request.add_mass(value);
    for (const auto& value : damping) request.add_damping(value);
    for (const auto& value : stiffness) request.add_stiffness(value);
    for (const auto& value : kpf) request.add_kpf(value);
    for (const auto& value : kif) request.add_kif(value);

    grpc::Status status = config_stub->SetForceControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetForceControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_force_control_gain(std::vector<float>& kp, 
                                      std::vector<float>& kv, 
                                      std::vector<float>& kl2, 
                                      std::vector<float>& mass, 
                                      std::vector<float>& damping, 
                                      std::vector<float>& stiffness, 
                                      std::vector<float>& kpf, 
                                      std::vector<float>& kif) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::ForceGainSet response;
    grpc::ClientContext context;

    grpc::Status status = config_stub->GetForceControlGain(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetForceControlGain RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    kp.assign(response.kp().begin(), response.kp().end());
    kv.assign(response.kv().begin(), response.kv().end());
    kl2.assign(response.kl2().begin(), response.kl2().end());
    mass.assign(response.mass().begin(), response.mass().end());
    damping.assign(response.damping().begin(), response.damping().end());
    stiffness.assign(response.stiffness().begin(), response.stiffness().end());
    kpf.assign(response.kpf().begin(), response.kpf().end());
    kif.assign(response.kif().begin(), response.kif().end());
    return true;
}

// bool IndyDCP3::set_mount_pos(const float rot_y, const float rot_z) {
//     Nrmk::IndyFramework::MountingAngles request;
//     request.set_ry(rot_y);
//     request.set_rz(rot_z);

//     Nrmk::IndyFramework::Response response;
//     grpc::ClientContext context;

//     grpc::Status status = config_stub->SetMountPos(&context, request, &response);
//     if (!status.ok()) {
//         std::cerr << "SetMountPos RPC failed: " << status.error_message() << std::endl;
//         return false;
//     }
//     return true;
// }

// bool IndyDCP3::get_mount_pos(float& rot_y, float& rot_z) {
//     Nrmk::IndyFramework::Empty request;
//     Nrmk::IndyFramework::MountingAngles response;
//     grpc::ClientContext context;

//     grpc::Status status = config_stub->GetMountPos(&context, request, &response);
//     if (!status.ok()) {
//         std::cerr << "GetMountPos RPC failed: " << status.error_message() << std::endl;
//         return false;
//     }

//     rot_y = response.ry();
//     rot_z = response.rz();
//     return true;
// }

bool IndyDCP3::set_brakes(const std::vector<bool>& brake_state_list) {
    Nrmk::IndyFramework::MotorList request;
    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    for (size_t i = 0; i < brake_state_list.size(); ++i) {
        Nrmk::IndyFramework::Motor* motor = request.add_motors();
        motor->set_index(static_cast<uint32_t>(i));
        motor->set_enable(brake_state_list[i]);
    }

    grpc::Status status = device_stub->SetBrakes(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetBrakes RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_servo_all(const bool enable) {
    Nrmk::IndyFramework::State request;
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetServoAll(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetServoAll RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_servo(const uint32_t index, const bool enable) {
    Nrmk::IndyFramework::Servo request;
    request.set_index(index);
    request.set_enable(enable);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetServo(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetServo RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_endtool_led_dim(const uint32_t led_dim) {
    Nrmk::IndyFramework::EndLedDim request;
    request.set_led_dim(led_dim);

    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetEndLedDim(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetEndLedDim RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::execute_tool(const std::string& name) {
    Nrmk::IndyFramework::Name request;
    request.set_name(name);

    Nrmk::IndyFramework::Empty response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->ExecuteTool(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "ExecuteTool RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

// bool IndyDCP3::get_el5001(int& status, int& value, int& delta, float& average) {
//     Nrmk::IndyFramework::Empty request;
//     Nrmk::IndyFramework::GetEL5001Data response;
//     grpc::ClientContext context;

//     grpc::Status status_result = device_stub->GetEL5001(&context, request, &response);
//     if (!status_result.ok()) {
//         std::cerr << "GetEL5001 RPC failed: " << status_result.error_message() << std::endl;
//         return false;
//     }

//     status = response.status();
//     value = response.value();
//     delta = response.delta();
//     average = response.average();
//     return true;
// }

// bool IndyDCP3::get_el5101(int& status, int& value, int& latch, int& delta, float& average) {
//     Nrmk::IndyFramework::Empty request;
//     Nrmk::IndyFramework::GetEL5101Data response;
//     grpc::ClientContext context;

//     grpc::Status status_result = device_stub->GetEL5101(&context, request, &response);
//     if (!status_result.ok()) {
//         std::cerr << "GetEL5101 RPC failed: " << status_result.error_message() << std::endl;
//         return false;
//     }

//     status = response.status();
//     value = response.value();
//     latch = response.latch();
//     delta = response.delta();
//     average = response.average();
//     return true;
// }

bool IndyDCP3::get_brake_control_style(int& style) {
    Nrmk::IndyFramework::Empty request;
    Nrmk::IndyFramework::BrakeControlStyle response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->GetBrakeControlStyle(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "GetBrakeControlStyle RPC failed: " << status.error_message() << std::endl;
        return false;
    }

    style = static_cast<int>(response.style());
    return true;
}

bool IndyDCP3::set_conveyor_name(const std::string& name) {
    Nrmk::IndyFramework::Name request;
    request.set_name(name);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorName(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorName RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_encoder(int encoder_type, int64_t channel1, int64_t channel2, int64_t sample_num,
                                    float mm_per_tick, float vel_const_mmps, bool reversed) {
    Nrmk::IndyFramework::Encoder request;
    request.set_type(static_cast<Nrmk::IndyFramework::Encoder::EncoderType>(encoder_type));
    request.set_channel1(channel1);
    request.set_channel2(channel2);
    request.set_sample_num(sample_num);
    request.set_mm_per_tick(mm_per_tick);
    request.set_vel_const_mmps(vel_const_mmps);
    request.set_reversed(reversed);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorEncoder(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorEncoder RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_trigger(int trigger_type, int64_t channel, bool detect_rise) {
    Nrmk::IndyFramework::Trigger request;
    request.set_type(static_cast<Nrmk::IndyFramework::Trigger::TriggerType>(trigger_type));
    request.set_channel(channel);
    request.set_detect_rise(detect_rise);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorTrigger(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorTrigger RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_offset(float offset_mm) {
    Nrmk::IndyFramework::Float request;
    request.set_value(offset_mm);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorOffset(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorOffset RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_starting_pose(const std::vector<float>& jpos, const std::vector<float>& tpos) {
    Nrmk::IndyFramework::PosePair request;

    for (const auto& pos : jpos) {
        request.add_q(pos);
    }

    for (const auto& pos : tpos) {
        request.add_p(pos);
    }

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorStartingPose(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorStartingPose RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::set_conveyor_terminal_pose(const std::vector<float>& jpos, const std::vector<float>& tpos) {
    Nrmk::IndyFramework::PosePair request;

    for (const auto& pos : jpos) {
        request.add_q(pos);
    }

    for (const auto& pos : tpos) {
        request.add_p(pos);
    }

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->SetConveyorTerminalPose(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "SetConveyorTerminalPose RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::add_photoneo_calib_point(const std::string& vision_name, double px, double py, double pz) {
    Nrmk::IndyFramework::AddPhotoneoCalibPointReq request;
    request.set_vision_name(vision_name);
    request.set_px(px);
    request.set_py(py);
    request.set_pz(pz);

    Nrmk::IndyFramework::Response response;
    grpc::ClientContext context;

    grpc::Status status = device_stub->AddPhotoneoCalibPoint(&context, request, &response);
    if (!status.ok()) {
        std::cerr << "AddPhotoneoCalibPoint RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_photoneo_detection(const Nrmk::IndyFramework::VisionServer& vision_server, 
                                      const std::string& object, 
                                      const Nrmk::IndyFramework::VisionFrameType frame_type, 
                                      Nrmk::IndyFramework::VisionResult& result) {
    Nrmk::IndyFramework::VisionRequest request;
    *request.mutable_vision_server() = vision_server;
    request.set_object(object);
    request.set_frame_type(frame_type);

    grpc::ClientContext context;
    grpc::Status status = device_stub->GetPhotoneoDetection(&context, request, &result);

    if (!status.ok()) {
        std::cerr << "GetPhotoneoDetection RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

bool IndyDCP3::get_photoneo_retrieval(const Nrmk::IndyFramework::VisionServer& vision_server, 
                                      const std::string& object, 
                                      const Nrmk::IndyFramework::VisionFrameType frame_type, 
                                      Nrmk::IndyFramework::VisionResult& result) {
    Nrmk::IndyFramework::VisionRequest request;
    *request.mutable_vision_server() = vision_server;
    request.set_object(object);
    request.set_frame_type(frame_type);

    grpc::ClientContext context;
    grpc::Status status = device_stub->GetPhotoneoRetrieval(&context, request, &result);

    if (!status.ok()) {
        std::cerr << "GetPhotoneoRetrieval RPC failed: " << status.error_message() << std::endl;
        return false;
    }
    return true;
}

