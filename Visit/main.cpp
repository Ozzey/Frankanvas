#include <iomanip>
#include <ctime>
#include <cstdlib>
#include <thread>
#include "udp.h"

namespace
{
bool telemetry_smoke_test = false;
bool tablet_smoke_test = false;
bool help_requested = false;
int telemetry_smoke_seconds = 5;
int tablet_smoke_seconds = 5;
std::string startup_replay_file;

void print_usage(const char *program_name)
{
    std::cout
        << "Usage: " << program_name << " [options]\n"
        << "Options:\n"
        << "  --robot-ip <ip>           Franka/MIOS controller IP (default: 192.168.3.200)\n"
        << "  --mios-port <port>        MIOS websocket port (default: 12000)\n"
        << "  --local-ip <ip>           This computer's IP reachable by the robot\n"
        << "  --start-pose <index>      Joint calibration start pose index (default: 0)\n"
        << "  --replay-file <file>      Start replay from Data/<file> after robot startup\n"
        << "  --draw-file <file>        Alias for --replay-file\n"
        << "  --telemetry-smoke [sec]   Subscribe to force telemetry only; no robot motion\n"
        << "  --tablet-smoke [sec]      Listen for tablet UDP data only; no robot motion\n"
        << "  --help                    Show this help\n";
}

bool parse_arguments(int argc, char **argv)
{
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            print_usage(argv[0]);
            help_requested = true;
            return false;
        }
        if (arg == "--robot-ip" && i + 1 < argc)
        {
            mios_ip = argv[++i];
            continue;
        }
        if (arg == "--mios-port" && i + 1 < argc)
        {
            mios_port = std::atoi(argv[++i]);
            runtime_mios_port = static_cast<unsigned>(mios_port);
            continue;
        }
        if (arg == "--local-ip" && i + 1 < argc)
        {
            local_ip = argv[++i];
            developerIP = local_ip;
            continue;
        }
        if (arg == "--start-pose" && i + 1 < argc)
        {
            program_start_pose = std::atoi(argv[++i]);
            continue;
        }
        if ((arg == "--replay-file" || arg == "--draw-file") && i + 1 < argc)
        {
            startup_replay_file = argv[++i];
            continue;
        }
        if (arg == "--telemetry-smoke")
        {
            telemetry_smoke_test = true;
            if (i + 1 < argc && argv[i + 1][0] != '-')
            {
                telemetry_smoke_seconds = std::atoi(argv[++i]);
            }
            if (telemetry_smoke_seconds < 1)
                telemetry_smoke_seconds = 1;
            continue;
        }
        if (arg == "--tablet-smoke")
        {
            tablet_smoke_test = true;
            if (i + 1 < argc && argv[i + 1][0] != '-')
            {
                tablet_smoke_seconds = std::atoi(argv[++i]);
            }
            if (tablet_smoke_seconds < 1)
                tablet_smoke_seconds = 1;
            continue;
        }
        if (!arg.empty() && arg[0] != '-')
        {
            program_start_pose = std::atoi(arg.c_str());
            continue;
        }

        std::cout << "Unknown or incomplete argument: " << arg << std::endl;
        print_usage(argv[0]);
        return false;
    }

    configure_network(mios_ip, local_ip);
    return true;
}

bool subscribe_force_telemetry()
{
    auto request = nlohmann::json(), response = nlohmann::json();
    double timeout = 7;
    request["ip"] = local_ip;
    request["port"] = telemetry_port;
    request["subscribe"] = {"O_F_ext_K"};
    request["withTerminatingNullCharacter"] = true;
    msrm_utils::JsonWebsocketClient::call_method(mios_ip, mios_port, endpoint, "subscribe_telemetry", request, response,
                                                 timeout);
    std::cout << "Response of subscribe_telemetry: " << response.dump() << std::endl;
    if (response["result"]["result"] == true)
    {
        std::cout << "Subscribe telemetry success" << std::endl;
        telemetry_status = true;
        return true;
    }

    std::cout << "Subscribe telemetry fail -> unable to use autocalibration" << std::endl;
    telemetry_status = false;
    return false;
}

void unsubscribe_force_telemetry()
{
    auto request = nlohmann::json(), response = nlohmann::json();
    double timeout = 2;
    request["ip"] = local_ip;
    msrm_utils::JsonWebsocketClient::call_method(mios_ip, mios_port, endpoint, "unsubscribe_telemetry",
                                                 request, response, timeout);
    std::cout << "Response of unsubscribe_telemetry: " << response.dump() << std::endl;
}

int run_telemetry_smoke_test()
{
    std::cout << "---Telemetry smoke test; no motion commands will be sent---" << std::endl;
    std::cout << "Robot IP: " << mios_ip << std::endl;
    std::cout << "MIOS port: " << mios_port << std::endl;
    std::cout << "Local IP: " << local_ip << std::endl;
    std::cout << "Telemetry client port: " << telemetry_port << std::endl;

    auto service = std::make_shared<CppServer::Asio::Service>(1);
    service->Start();
    auto telemetry_server = std::make_shared<Telemetry>(service, telemetry_port);
    telemetry_server->Start();

    bool subscribed = subscribe_force_telemetry();
    if (subscribed)
    {
        for (int i = 0; i < telemetry_smoke_seconds; i++)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (telemetry_data.mutex.try_lock())
            {
                std::cout << "External force:";
                for (double value : telemetry_data.external_force)
                    std::cout << " " << value;
                std::cout << std::endl;
                telemetry_data.mutex.unlock();
            }
        }
        unsubscribe_force_telemetry();
    }

    telemetry_server->Stop();
    service->Stop();
    return subscribed ? 0 : 1;
}

int run_tablet_smoke_test()
{
    std::cout << "---Tablet smoke test; no robot or MIOS commands will be sent---" << std::endl;
    std::cout << "Tablet UDP input port: " << external_data_receive_port << std::endl;

    auto service = std::make_shared<CppServer::Asio::Service>(1);
    service->Start();
    auto tablet_server = std::make_shared<TabletUDPServer>(service, external_data_receive_port);
    tablet_server->Start();

    for (int i = 0; i < tablet_smoke_seconds; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (output_data.mutex.try_lock())
        {
            std::cout << "Tablet data: present=" << output_data.has_tablet_data
                      << " on_paper=" << output_data.on_paper;
            if (output_data.tablet_data.size() >= 5)
            {
                std::cout << " x=" << output_data.tablet_data[0]
                          << " y=" << output_data.tablet_data[1]
                          << " pressure=" << output_data.tablet_data[2]
                          << " altitude=" << output_data.tablet_data[3]
                          << " azimuth=" << output_data.tablet_data[4];
            }
            std::cout << std::endl;
            output_data.has_tablet_data = false;
            output_data.mutex.unlock();
        }
    }

    tablet_server->Stop();
    service->Stop();
    return 0;
}
}

void load_config(){
    std::ifstream file;

    file.open(relative_path + "Config/Robot_Calibration_Point_1.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : First Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Robot_Calibration_Point_1[i];
        Robot_Calibration_Points[0][i] = Robot_Calibration_Point_1[i];
    }
    file.close();

    file.open(relative_path + "Config/Robot_Calibration_Point_2.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Second Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Robot_Calibration_Point_2[i];
        Robot_Calibration_Points[1][i] = Robot_Calibration_Point_2[i];
    }
    file.close();

    file.open(relative_path + "Config/Robot_Calibration_Point_3.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Third Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Robot_Calibration_Point_3[i];
        Robot_Calibration_Points[2][i] = Robot_Calibration_Point_3[i];
    }
    file.close();

    file.open(relative_path + "Config/Robot_Calibration_Point_4.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Fourth Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Robot_Calibration_Point_4[i];
        Robot_Calibration_Points[3][i] = Robot_Calibration_Point_4[i];
    }
    file.close();
    std::cout << "Robot Calibration Points Read Successfully" << std::endl;
//    std::cout << Robot_Calibration_Point_1[15] << std::endl;
//    std::cout << Robot_Calibration_Point_2[15] << std::endl;
//    std::cout << Robot_Calibration_Point_3[15] << std::endl;
//    std::cout << Robot_Calibration_Point_4[15] << std::endl;

    file.open(relative_path + "Config/MoCap_Calibration_Point_1.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : First MoCap Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> MoCap_Calibration_Point_1[i];
    }
    file.close();

    file.open(relative_path + "Config/MoCap_Calibration_Point_2.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Second MoCap Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> MoCap_Calibration_Point_2[i];
    }
    file.close();

    file.open(relative_path + "Config/MoCap_Calibration_Point_3.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Third MoCap Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> MoCap_Calibration_Point_3[i];
    }
    file.close();

    file.open(relative_path + "Config/MoCap_Calibration_Point_4.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Fourth MoCap Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> MoCap_Calibration_Point_4[i];
    }
    file.close();
    std::cout << "MoCap Calibration Points Read Successfully" << std::endl;

    file.open(relative_path + "Config/Robot_Calibration_Point_1_Joint.txt", std::ios::in);
    if (file.is_open() == 0)
    {
        std::cout << "Error : First Robot Calibration Point Joint Pose Not Found" << std::endl;
        return;
    }
    std::getline(file, Robot_Calibration_Points_Joint[0]);
    std::cout << "Robot Calibration Point 1 Joint " << Robot_Calibration_Points_Joint[0] << std::endl;
    file.close();

    file.open(relative_path + "Config/Camera_Robot_Calibration_Point_1_Joint.txt", std::ios::in);
    if (file.is_open() == 0)
    {
        std::cout << "Error : First Robot Calibration Point Joint Pose Not Found" << std::endl;
        return;
    }
    std::getline(file, Camera_Robot_Calibration_Points_Joint[0]);
    std::cout << "Camera Robot Calibration Point 1 Joint " << Camera_Robot_Calibration_Points_Joint[0] << std::endl;
    file.close();

    file.open(relative_path + "Config/Robot_Telepresence_Point_Start.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Robot Telepresence Point Start Point Not Found" << std::endl; return; }
    Telepresence_Start_Pose.resize(16);
    for (int i=0; i<16; i++){
        file >> Telepresence_Start_Pose[i];
    }
    file.close();
    std::cout << "Robot Telepresence Point Start Point Read Successfully" << std::endl;

    file.open(relative_path + "Config/Camera_Robot_Point_Start.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Camera Robot Start Point Not Found" << std::endl; return; }
    Camera_Start_Pose.resize(16);
    for (int i=0; i<16; i++){
        file >> Camera_Start_Pose[i];
    }
    file.close();
    std::cout << "Camera Robot Start Point Read Successfully" << std::endl;


}

void load_camera_config(){
    std::ifstream file;

    file.open(relative_path + "Config/Camera_1.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : First Camera Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Camera_Robot_Calibration_Point_1[i];
        Camera_Robot_Calibration_Points[0][i] = Camera_Robot_Calibration_Point_1[i];
    }
    file.close();

    file.open(relative_path + "Config/Camera_2.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Second Camera Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Camera_Robot_Calibration_Point_2[i];
        Camera_Robot_Calibration_Points[1][i] = Camera_Robot_Calibration_Point_2[i];
    }
    file.close();

    file.open(relative_path + "Config/Camera_3.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Third Camera Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Camera_Robot_Calibration_Point_3[i];
        Camera_Robot_Calibration_Points[2][i] = Camera_Robot_Calibration_Point_3[i];
    }
    file.close();


    file.open(relative_path + "Config/Camera_4.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Third Camera Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Camera_Robot_Calibration_Point_4[i];
        Camera_Robot_Calibration_Points[3][i] = Camera_Robot_Calibration_Point_4[i];
    }
    file.close();


    file.open(relative_path + "Config/Camera_5.txt", std::ios::in);
    if (file.is_open() == 0) {std::cout << "Error : Third Camera Robot Calibration Point Not Found" << std::endl; return; }
    for (int i=0; i<16; i++){
        file >> Camera_Robot_Calibration_Point_5[i];
        Camera_Robot_Calibration_Points[4][i] = Camera_Robot_Calibration_Point_5[i];
    }
    file.close();

    std::cout << "Camera Robot Calibration Points Read Successfully" << std::endl;
}

void calculate_transformation(){
    std::vector<Eigen::Vector3d> Robot_Plane_Points;
    std::vector<Eigen::Vector3d> MoCap_Plane_Points;

    Robot_Plane_Points.push_back(Eigen::Vector3d(Robot_Calibration_Point_1[12],Robot_Calibration_Point_1[13],Robot_Calibration_Point_1[14]));
    Robot_Plane_Points.push_back(Eigen::Vector3d(Robot_Calibration_Point_2[12],Robot_Calibration_Point_2[13],Robot_Calibration_Point_2[14]));
    Robot_Plane_Points.push_back(Eigen::Vector3d(Robot_Calibration_Point_3[12],Robot_Calibration_Point_3[13],Robot_Calibration_Point_3[14]));
    //Robot_Plane_Points.push_back(Eigen::Vector3d(Robot_Calibration_Point_4[12],Robot_Calibration_Point_4[13],Robot_Calibration_Point_4[14]));

    MoCap_Plane_Points.push_back(Eigen::Vector3d(MoCap_Calibration_Point_1[12],MoCap_Calibration_Point_1[13],MoCap_Calibration_Point_1[14]));
    MoCap_Plane_Points.push_back(Eigen::Vector3d(MoCap_Calibration_Point_2[12],MoCap_Calibration_Point_2[13],MoCap_Calibration_Point_2[14]));
    MoCap_Plane_Points.push_back(Eigen::Vector3d(MoCap_Calibration_Point_3[12],MoCap_Calibration_Point_3[13],MoCap_Calibration_Point_3[14]));
    //MoCap_Plane_Points.push_back(Eigen::Vector3d(MoCap_Calibration_Point_4[12],MoCap_Calibration_Point_4[13],MoCap_Calibration_Point_4[14]));

    Robot_Drawing_Plane = PlaneFitting(Robot_Plane_Points);
    Button_Modified_Drawing_Plane = Robot_Drawing_Plane;
    MoCap_Drawing_Plane = PlaneFitting(MoCap_Plane_Points);

    // Plane Fitting Result
    std::cout << "Robot Drawing Plane: " << std::endl << Robot_Drawing_Plane << std::endl;
    //std::cout << "MoCap Drawing Plane: " << MoCap_Drawing_Plane << std::endl;
}

int main(int argc, char **argv)
{

    if (!parse_arguments(argc, argv))
        return help_requested ? 0 : 1;

    std::cout << "Robot IP: " << mios_ip << std::endl;
    std::cout << "Local IP: " << local_ip << std::endl;
    std::cout << "Start Pose: " << program_start_pose << std::endl;
    if (!startup_replay_file.empty())
        std::cout << "Startup replay file: " << startup_replay_file << std::endl;

    if (telemetry_smoke_test)
        return run_telemetry_smoke_test();
    if (tablet_smoke_test)
        return run_tablet_smoke_test();

    std::cout << "---Enter Direct Cartesian Control Mode---" << std::endl;
    load_config(); // Read all calibration data
    load_camera_config(); 

    calculate_transformation();
    
    std::cout << "External data port: " << external_data_receive_port << std::endl;
    std::cout << "Telemetry client port: " << telemetry_port << std::endl;

    // Create a new Asio service
    auto service = std::make_shared<CppServer::Asio::Service>(1);

    // Start the Asio service
    std::cout << "Asio service starting...";
    service->Start();
    std::cout << "Done!" << std::endl;

    // Create a new UDP server

    if (!move_to_joint_motion(mios_ip, endpoint, Robot_Calibration_Points_Joint[program_start_pose]))
    {
        std::cerr << "Robot startup motion failed. Check that MIOS is running and reachable at "
                  << mios_ip << ":" << mios_port << "." << std::endl;
        service->Stop();
        return 1;
    }
    
    // move_to_joint_motion_camera(mios_ip, endpoint, Camera_Robot_Calibration_Points_Joint[0]);

    if (!start_telepresence(program_start_pose))
    {
        std::cerr << "Telepresence startup failed. Check that MIOS is running and reachable at "
                  << mios_ip << ":" << mios_port << "." << std::endl;
        service->Stop();
        return 1;
    }

    // start_telepresence_camera(0);
    // auto command_server = std::make_shared<CommandServer>(service, udp_command_server_port);
    auto tablet_server = std::make_shared<TabletUDPServer>(service, external_data_receive_port);
    auto telemetry_server = std::make_shared<Telemetry>(service, telemetry_port);
    auto timer = std::make_shared<AsioTimer>(service);

    // Setup Timer
    timer->Last_Output.resize(16);
    timer->Output.resize(16);
    
    timer->Camera_Last_Output.resize(16);
    timer->Camera_Output.resize(16);

    for (int i = 0; i < 16; i++)
    {
        timer->Last_Output[i] = Telepresence_Start_Pose[i];
        timer->Output[i] = Telepresence_Start_Pose[i];
        timer->Camera_Last_Output[i] = Camera_Start_Pose[i];
        timer->Camera_Output[i] = Camera_Start_Pose[i];
        std::cout << timer->Output[i] << std::endl;
    }

    timer->First_Valid_Tablet_Data=true;
    timer->First_Valid_MoCap_Data=false;
    timer->Last_Output_Exist=true;
    timer->Sampling_Step=0;
    timer->no_data_count=0;
    timer->no_digital_tablet_data_count=0;
    timer->button_fixed_countdown=0;
    timer->Target_Drawing_Plane=Robot_Drawing_Plane;
    timer->calibrationinit=false;
    timer->target_percentage_x=0.5;
    timer->target_percentage_y=0.5;
    timer->target_button=0;
    timer->target_pressure=0.0;
    timer->target_altitude=1.57079632679;
    timer->target_azimuth=0.0;
    timer->target_velocity_limit=Move_Vlimit;
    timer->last_output_storage.output_x=0.5;
    timer->last_output_storage.output_y=0.5;
    timer->last_output_storage.on_paper=false;
    timer->temp_storage.has_tablet_data=false;
    timer->temp_storage.has_mocap_data=false;
    timer->telemetryForce.assign(6, 0.0);
    telemetry_data.external_force.assign(6, 0.0);
    timer->Setup(CppCommon::Timespan::milliseconds(5));
    timer->WaitAsync();
    // Start the server
    std::cout << "Tablet data receive server starting on UDP port " << external_data_receive_port << "...";
    tablet_server->Start();
    std::cout << "Done!" << std::endl;

    std::cout << "Telemetry receive server starting...";
    telemetry_server->Start();
    std::cout << "Done!" << std::endl;

    // Start telemetry for external force measurement, not needed if the sensor is sent from python server
    subscribe_force_telemetry();

    if (!startup_replay_file.empty())
    {
        if (request_data.mutex.try_lock())
        {
            request_data.draw_file_name = startup_replay_file;
            receive_file_draw_request = true;
            request_data.mutex.unlock();
            std::cout << "Queued startup replay file: " << startup_replay_file << std::endl;
        }
        else
        {
            std::cout << "Unable to queue startup replay file because request mutex is locked" << std::endl;
        }
    }

    std::cout << "Type 'q' then Enter to stop the program or '!' then Enter to restart the server..." << std::endl;
    // Perform text input
    std::string line;

    while (true) // &&
    {
        if (!getline(std::cin, line))
        {
            std::cin.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (line.empty())
            continue;
        if (line == "q")
            break;
        // Restart the server
        if (line == "!")
        {
            std::cout << "Server restarting...";
            tablet_server->Restart();
            telemetry_server->Restart();
            std::cout << "Done!" << std::endl;
            continue;
        }

        // Autocalibration
        if (line.compare(0,2,"c ")==0){
            std::cout << "Receive calibration request" << std::endl;
            if (!telemetry_status){
                std::cout << "Telemetry start failed, cannot start autocalibration!" << std::endl;
                continue;
            }
            double temp=0;
            try {
                temp = std::stod(line.substr(2,line.size()-2));
            }  catch (const std::invalid_argument&) {
                std::cerr << "Argument is invalid\n";
                continue;
            } catch (const std::out_of_range&) {
                std::cerr << "Argument is out of range for a double\n";
                continue;
            }
            std::cout << "Requested external force target = " << temp << " N" << std::endl;
            if (temp<=0){
                std::cout << "Negative request force target -> Ignore" << std::endl;
                continue;
            }
            if (temp>=10){
                std::cout << "High extenal force target, please be careful" << std::endl;
            }
            if (request_data.mutex.try_lock()){
                request_data.calibration_target_force = temp;
                receive_calibration_request = true;
                request_data.mutex.unlock();
            }else{
                std::cout << "Mutex in lock, please try again" << std::endl;
            }
            continue;
        }

        // Record
        if (line.compare(0,7,"record ")==0){
            std::cout << "Receive record file request" << std::endl;
            std::string temp;
            temp = line.substr(7,line.size()-7);
            std::cout << "Record file name: " << temp << std::endl;
            if (request_data.mutex.try_lock()){
                request_data.record_file_name = temp;
                receive_file_record_request = true;
                request_data.mutex.unlock();
            }else{
                std::cout << "Mutex in lock, please try again" << std::endl;
            }
            continue;
        }

        // Stop Record
        if (line.compare(0,10,"recordstop")==0){
            std::cout << "Receive record stop request" << std::endl;
                receive_stop_file_record_request = true;
            continue;
        }

        // Replay
        if (line.compare(0,7,"replay ")==0){
            std::cout << "Receive replay file request" << std::endl;
            std::string temp;
            temp = line.substr(7,line.size()-7);
            std::cout << "Replay file name: " << temp << std::endl;
            if (request_data.mutex.try_lock()){
                request_data.draw_file_name = temp;
                receive_file_draw_request = true;
                request_data.mutex.unlock();
            }else{
                std::cout << "Mutex in lock, please try again" << std::endl;
            }
            continue;
        }

        std::cout << "Unrecognized input -> Ignore" << std::endl;
    }

    // Stop the server
    std::cout << "Both Server stopping..." << std::endl;
    tablet_server->Stop();
    telemetry_server->Stop();

    std::cout << "Done!" << std::endl;

    // Stop the Asio service
    std::cout << "Asio service stopping...";
    service->Stop();
    std::cout << "Done!" << std::endl;

    unsubscribe_force_telemetry();

    return 0;
}
