#include "server/asio/udp_server.h"
#include "server/asio/timer.h"
#include "threads/thread.h"
#include <iostream>

#include <boost/filesystem.hpp>
// #include <glm/glm.hpp>
// #include <glm/gtc/matrix_transform.hpp>
// #include <glm/gtc/type_ptr.hpp>

#include <cmath>
#include "Eigen/Dense"
#include "Eigen/SVD"
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <locale>
#include <memory>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <sstream>

#include "MiosEventServer.hpp"

// #include "../new_message_parser/bvh-parser.h"

#include "msrm_utils/network.hpp"
#include "msrm_utils/json.hpp"
#include "msrm_utils/math.hpp"

//#include <CGAL/Simple_cartesian.h>
//#include <CGAL/Surface_mesh.h>
//#include <CGAL/AABB_tree.h>
//#include <CGAL/AABB_traits.h>
//#include <CGAL/AABB_face_graph_triangle_primitive.h>
//#include <CGAL/Polygon_mesh_processing/compute_normal.h>
//#include <CGAL/Polygon_mesh_processing/orientation.h>

#include "franka_ik_He.hpp"

#include "rate_limiting.h"

namespace bf = boost::filesystem;

std::string local_ip = "127.0.0.1";
std::string mios_ip = "192.168.3.200";
std::string endpoint = "mios/core";
std::string developerIP = "127.0.0.1";

int mios_port = 12000;
int developerUDPReceiveDataPort = 13333;
int CameraUDPReceiveDataPort = 23333;
int external_data_receive_port = 3333;
int telemetry_port = 3334;
int program_start_pose = 0;

auto udpSender = std::make_shared<msrm_utils::UDPStreamSender>(mios_ip, developerUDPReceiveDataPort);
// auto udpSender_Camera = std::make_shared<msrm_utils::UDPStreamSender>(developerIP, CameraUDPReceiveDataPort);
// Telepresence related components

TelepresenceMode telepresence_mode;

// auto python_client_receiver_endpoint = asio::ip::udp::endpoint(asio::ip::address_v4::from_string(python_server_ip), (unsigned short)python_commmand_recive_port);

// bvh::Bvh_parser parser_test;
// bvh::Bvh data_test;

std::string target_joint = "RightHandIndex3";
int target_joing_index = -1;

// auto second_receiver_endpoint = asio::ip::udp::endpoint(asio::ip::address_v4::from_string("192.168.8.160"), (unsigned short)3333);

// std::vector<std::string> udp_package_list;
Eigen::Matrix4d Last_Pose, Pose_Difference; // Calculate Relative Pose between the current pose and last pose
bool Last_Pose_Exist = false, velocity_control = false, direct_cart_control = true;
double Last_Output[3];
double Robot_Calibration_Point_1[16], Robot_Calibration_Point_2[16], Robot_Calibration_Point_3[16], Robot_Calibration_Point_4[16];
double MoCap_Calibration_Point_1[16], MoCap_Calibration_Point_2[16], MoCap_Calibration_Point_3[16], MoCap_Calibration_Point_4[16];
std::array<double, 16> Camera_Robot_Calibration_Point_1, Camera_Robot_Calibration_Point_2, Camera_Robot_Calibration_Point_3, Camera_Robot_Calibration_Point_4, Camera_Robot_Calibration_Point_5; 
double Robot_Calibration_Points[4][16];
double Camera_Robot_Calibration_Points[5][16];
double User_Button_Modification = 0, Force_Regulate_Modification = 0;

std::vector<double> Telepresence_Start_Pose;

std::vector<double> Camera_Start_Pose;

std::string Robot_Calibration_Points_Joint[2];

std::string Camera_Robot_Calibration_Points_Joint[2];

// int Last_button, ;
// double Last_percentage_X;
// double Last_percentage_Y;

double Current_User_Calibration_Point[3] = {0, 0, 0};
double Current_Origin_Movement[3] = {0, 0, 0};

const double Draw_Vlimit = 0.0005, Move_Vlimit = 0.00075;
const double targetVelocityLimit = 0.0005;

int button_count = 0, current_calibration_point_no = 1, fixed_button_duration = 0;
bool current_calibration_point_output_finished = false, button_forward_pressed = false, button_backward_pressed = false;
bool calibration_mode = false, tablet_mode = false, fusion_mode = false;
int packet_type = 1; // 1 for digital tablet, 2 for motion capture suit
Eigen::Vector4d Robot_Drawing_Plane, MoCap_Drawing_Plane, Button_Modified_Drawing_Plane;

double customplane_mod = 0.013;
std::string configDirectoryLocation = "../../Config/";
std::string fileDirectoryLocation = "../../Data/";
std::string relative_path = "../../";

struct
{
    std::mutex mutex;
    bool has_tablet_data;
    bool has_mocap_data;
    bool on_paper;
    std::vector<double> tablet_data;
    std::vector<double> mocap_data;
} output_data{};

struct storage_type
{
    int time_step;
    bool has_tablet_data;
    bool has_mocap_data;
    bool on_paper;
    double tablet_data[5];
    double mocap_data[3];
    double mocap_x, mocap_y;
    double output_x, output_y;
};

struct
{
    std::mutex mutex;
    double calibration_target_force;
    std::string record_file_name;
    std::string draw_file_name;
} request_data{};

struct
{
    std::mutex mutex;
    std::vector<double> external_force;
} telemetry_data{};

std::atomic<bool> receive_calibration_request = false, receive_file_record_request = false, receive_file_draw_request = false, receive_stop_file_record_request = false, telemetry_status = false;
std::atomic<int> current_state_machine_status = 0; // 0: Normal drawing status, 1: Calibration mode, 2: Record mode 3: Replay mode

std::chrono::time_point time_start = std::chrono::high_resolution_clock::now();
std::chrono::time_point time_end = std::chrono::high_resolution_clock::now();

struct
{
    std::mutex mutex;
    bool has_sensor_data;
    std::vector<double> sensor_data;
} received_sensor_data{};

std::atomic<bool> receive_command_request = false;
std::atomic<int> current_task_no = 0;
std::atomic<int> current_experiment_state = 0; // 0: Idle, 1: Force test, 2: Force test with orientation 3: Slip test 4: Side force
std::atomic<int> current_experiment_sub_state = 0;
std::atomic<int> target_experiment_sub_state = 0;

std::string convertChronoToString(const std::chrono::time_point<std::chrono::system_clock> &time, const std::string &format = "%Y-%m-%d-%H-%M-%S")
{
    time_t timeStruct;
    std::stringstream ss;
    timeStruct = std::chrono::system_clock::to_time_t(time);
    ss.clear();
    ss.str(std::string());
    ss << std::put_time(localtime(&timeStruct), format.c_str());
    return ss.str();
}

bool move_to_joint_motion(std::string mios_ip, std::string mios_endpoint, std::string initial_joint_pose)
{
    return startMoveToJointPoseTask(mios_ip, mios_endpoint, initial_joint_pose);
}

bool move_to_joint_motion_camera(std::string mios_ip, std::string mios_endpoint, std::string initial_joint_pose)
{
    return startMoveToJointPoseTask_Camera(mios_ip, mios_endpoint, initial_joint_pose);
}

void configure_network(const std::string &robot_ip, const std::string &host_ip)
{
    mios_ip = robot_ip;
    local_ip = host_ip;
    developerIP = host_ip;
    udpSender = std::make_shared<msrm_utils::UDPStreamSender>(mios_ip, developerUDPReceiveDataPort);
    if (!udpSender->connect())
    {
        std::cout << "Failed to connect udp stream sender to " << mios_ip << ":" << developerUDPReceiveDataPort << std::endl;
    }
}

bool start_telepresence(int point_no)
{
    if (!startTelepresenceSkill("direct_cartesian", developerIP, mios_ip, endpoint, developerUDPReceiveDataPort, 13334, telepresence_mode))
        return false;
    MiosEventServer *miosEventServer = new MiosEventServer(true, 100);
    // std::vector<double> temp(std::begin(Robot_Calibration_Points[point_no]), std::end(Robot_Calibration_Points[point_no]));
    miosEventServer->handshakeData = Telepresence_Start_Pose;
    if (!miosEventServer->InitTelepresence(mios_ip, mios_port, 16666, endpoint, telepresence_mode,
                                           10002))
    {
        std::cout << "Failed to connect to mios with address "
                  << mios_ip << std::endl;
        return false;
    }
    return true;
}

bool start_telepresence_camera(int point_no)
{
    if (!startTelepresenceSkill_Camera("direct_cartesian", developerIP, mios_ip, endpoint, 23333, 23334, telepresence_mode))
        return false;
    MiosEventServer *miosEventServer = new MiosEventServer(true, 100);
    // std::vector<double> temp(std::begin(Robot_Calibration_Points[point_no]), std::end(Robot_Calibration_Points[point_no]));
    miosEventServer->handshakeData = Camera_Start_Pose;
    if (!miosEventServer->InitTelepresence(mios_ip, 13000, 17666, endpoint, telepresence_mode,
                                           11002))
    {
        std::cout << "Failed to connect to mios with address "
                  << mios_ip << std::endl;
        return false;
    }
    return true;
}

void stop_telepresence()
{
    auto request = nlohmann::json(), response = nlohmann::json();
    double timeout = 7;
    request.clear();
    msrm_utils::JsonWebsocketClient::call_method(mios_ip, mios_port, endpoint, "stop_task",
                                                 request, response, timeout);
}

Eigen::Vector4d PlaneFitting(const std::vector<Eigen::Vector3d> &plane_pts)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto &pt : plane_pts)
        center += pt;
    center /= plane_pts.size();

    Eigen::MatrixXd A(plane_pts.size(), 3);
    for (int i = 0; i < plane_pts.size(); i++)
    {
        A(i, 0) = plane_pts[i][0] - center[0];
        A(i, 1) = plane_pts[i][1] - center[1];
        A(i, 2) = plane_pts[i][2] - center[2];
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    double a = svd.matrixV()(0, 2);
    double b = svd.matrixV()(1, 2);
    double c = svd.matrixV()(2, 2);
    double d = -(a * center[0] + b * center[1] + c * center[2]);
    if (d>0){
        a = -a;
        b = -b;
        c = -c;
        d = -d;
    }
    return Eigen::Vector4d(a, b, c, d);
}

void update_robot_plane(std::vector<double> calibration_results[3])
{
    std::vector<Eigen::Vector3d> Robot_Plane_Points;

    for (int i = 0; i < 3; i++)
    {
        std::cout << "Calibration Point used for update robot plane" << i << " " << calibration_results[i][12] << calibration_results[i][13] << calibration_results[i][14] << std::endl;
        Robot_Plane_Points.push_back(Eigen::Vector3d(calibration_results[i][12], calibration_results[i][13], calibration_results[i][14]));
    }

    Robot_Drawing_Plane = PlaneFitting(Robot_Plane_Points);
    Button_Modified_Drawing_Plane = Robot_Drawing_Plane;

    // Plane Fitting Result
    std::cout << "Robot Drawing Plane: " << std::endl
              << Robot_Drawing_Plane << std::endl;
}

Eigen::Vector3d Find_Projected_Point(Eigen::Vector3d point, Eigen::Vector4d plane)
{
    Eigen::Vector3d projected_point;
    Eigen::Vector3d plane_normal;
    plane_normal = plane.head<3>();
    projected_point = point - (plane_normal.dot(point) + plane[3]) * plane_normal;
    return projected_point;
}

Eigen::Vector3d Find_Target_Point(Eigen::Vector3d mocap_point, Eigen::Vector4d mocap_plane, Eigen::Vector4d robot_plane)
{
    Eigen::Vector3d target_point;

    double A, B, C, D, E, F, k1, k2;

    A = (MoCap_Calibration_Point_3[12] - MoCap_Calibration_Point_2[12]);
    B = (MoCap_Calibration_Point_1[12] - MoCap_Calibration_Point_2[12]);
    C = mocap_point[0] - MoCap_Calibration_Point_2[12];
    D = (MoCap_Calibration_Point_3[13] - MoCap_Calibration_Point_2[13]);
    E = (MoCap_Calibration_Point_1[13] - MoCap_Calibration_Point_2[13]);
    F = mocap_point[1] - MoCap_Calibration_Point_2[13];

    // std::cout << (A*E-B*D) << std::endl;
    if (abs(A * E - B * D) < 1e-6)
    {
        std::cout << "Possible Calibration Data Error, Please Check the Calibration Data" << std::endl;
    };
    k1 = (C * E - B * F) / (A * E - B * D);
    k2 = (C * D - A * F) / (B * D - A * E);
    // std::cout << "k1: " << k1 << std::endl;
    // std::cout << "k2: " << k2 << std::endl;
    if (k1 < 0)
        k1 = 0;
    if (k1 >= 1)
        k1 = 1;
    if (k2 < 0)
        k2 = 0;
    if (k2 >= 1)
        k2 = 1;

    target_point[0] = k1 * (Robot_Calibration_Point_3[12] - Robot_Calibration_Point_2[12]) + k2 * (Robot_Calibration_Point_1[12] - Robot_Calibration_Point_2[12]) + Robot_Calibration_Point_2[12];
    target_point[1] = k1 * (Robot_Calibration_Point_3[13] - Robot_Calibration_Point_2[13]) + k2 * (Robot_Calibration_Point_1[13] - Robot_Calibration_Point_2[13]) + Robot_Calibration_Point_2[13];
    target_point[2] = k1 * (Robot_Calibration_Point_3[14] - Robot_Calibration_Point_2[14]) + k2 * (Robot_Calibration_Point_1[14] - Robot_Calibration_Point_2[14]) + Robot_Calibration_Point_2[14];

    target_point = Find_Projected_Point(Eigen::Vector3d(target_point[0], target_point[1], target_point[2]), robot_plane);
    // std::cout << "Target Point: " << target_point << std::endl;
    // std::cout << "Verify Target Point at Robot Draw Plane" << Find_Projected_Point(Eigen::Vector3d(target_point[0],target_point[1],target_point[2]), Robot_Drawing_Plane) << std::endl;

    return target_point;
}

Eigen::Vector2d Find_Target_Point_MoCap_X_Y(Eigen::Vector3d mocap_point, Eigen::Vector4d mocap_plane, Eigen::Vector4d robot_plane)
{
    Eigen::Vector2d target_xy;

    double A, B, C, D, E, F, k1, k2;

    A = (MoCap_Calibration_Point_3[12] - MoCap_Calibration_Point_2[12]);
    B = (MoCap_Calibration_Point_1[12] - MoCap_Calibration_Point_2[12]);
    C = mocap_point[0] - MoCap_Calibration_Point_2[12];
    D = (MoCap_Calibration_Point_3[13] - MoCap_Calibration_Point_2[13]);
    E = (MoCap_Calibration_Point_1[13] - MoCap_Calibration_Point_2[13]);
    F = mocap_point[1] - MoCap_Calibration_Point_2[13];

    // std::cout << (A*E-B*D) << std::endl;
    if (abs(A * E - B * D) < 1e-6)
    {
        std::cout << "Possible Calibration Data Error, Please Check the Calibration Data" << std::endl;
    };
    k1 = (C * E - B * F) / (A * E - B * D);
    k2 = (C * D - A * F) / (B * D - A * E);
    // std::cout << "k1: " << k1 << std::endl;
    // std::cout << "k2: " << k2 << std::endl;
    //     if (k1<0)  k1 = 0;
    //     if (k1>=1) k1 = 1;
    //     if (k2<0)  k2 = 0;
    //     if (k2>=1) k2 = 1;

    target_xy[0] = k1;
    target_xy[1] = k2;
    // std::cout << "Target Point: " << target_point << std::endl;
    // std::cout << "Verify Target Point at Robot Draw Plane" << Find_Projected_Point(Eigen::Vector3d(target_point[0],target_point[1],target_point[2]), Robot_Drawing_Plane) << std::endl;

    return target_xy;
}

Eigen::Vector3d Find_Target_Point_X_Y(double mousex, double mousey, Eigen::Vector4d robot_plane)
{
    Eigen::Vector3d target_point;

    double k1, k2;
    k1 = mousex;
    k2 = mousey;
    if (k1 < 0)
        k1 = 0;
    if (k1 >= 1)
        k1 = 1;
    if (k2 < 0)
        k2 = 0;
    if (k2 >= 1)
        k2 = 1;
    target_point[0] = k1 * (Robot_Calibration_Point_3[12] - Robot_Calibration_Point_2[12]) + k2 * (Robot_Calibration_Point_1[12] - Robot_Calibration_Point_2[12]) + Robot_Calibration_Point_2[12];
    target_point[1] = k1 * (Robot_Calibration_Point_3[13] - Robot_Calibration_Point_2[13]) + k2 * (Robot_Calibration_Point_1[13] - Robot_Calibration_Point_2[13]) + Robot_Calibration_Point_2[13];
    target_point[2] = k1 * (Robot_Calibration_Point_3[14] - Robot_Calibration_Point_2[14]) + k2 * (Robot_Calibration_Point_1[14] - Robot_Calibration_Point_2[14]) + Robot_Calibration_Point_2[14];

    target_point = Find_Projected_Point(Eigen::Vector3d(target_point[0], target_point[1], target_point[2]), robot_plane);
    // std::cout << "Target Point: " << target_point << std::endl;
    // std::cout << "Verify Target Point at Robot Draw Plane" << Find_Projected_Point(Eigen::Vector3d(target_point[0],target_point[1],target_point[2]), Robot_Drawing_Plane) << std::endl;

    return target_point;
}

Eigen::Vector3d Find_Camera_Target_Translation_Transformation(double mousex, double mousey){
    Eigen::Vector3d target_point;
    Eigen::Vector3d temp_line_segement_left;
    Eigen::Vector3d temp_line_segement_right;

    double k1, k2;
    k1 = mousex;
    k2 = mousey;
    if (k1 < 0)
        k1 = 0;
    if (k1 >= 1)
        k1 = 1;
    if (k2 < 0)
        k2 = 0;
    if (k2 >= 1)
        k2 = 1;
    
    if ((k1>=k2)){
        if ((k1+k2 <= 1)){              // top
            temp_line_segement_left[0] = Camera_Robot_Calibration_Point_2[12] + k2 / 0.5 * (Camera_Robot_Calibration_Point_5[12] - Camera_Robot_Calibration_Point_2[12]);
            temp_line_segement_left[1] = Camera_Robot_Calibration_Point_2[13] + k2 / 0.5 * (Camera_Robot_Calibration_Point_5[13] - Camera_Robot_Calibration_Point_2[13]);
            temp_line_segement_left[2] = Camera_Robot_Calibration_Point_2[14] + k2 / 0.5 * (Camera_Robot_Calibration_Point_5[14] - Camera_Robot_Calibration_Point_2[14]);

            temp_line_segement_right[0] = Camera_Robot_Calibration_Point_3[12] + k2 / 0.5 * (Camera_Robot_Calibration_Point_5[12] - Camera_Robot_Calibration_Point_3[12]);
            temp_line_segement_right[1] = Camera_Robot_Calibration_Point_3[13] + k2 / 0.5 * (Camera_Robot_Calibration_Point_5[13] - Camera_Robot_Calibration_Point_3[13]);
            temp_line_segement_right[2] = Camera_Robot_Calibration_Point_3[14] + k2 / 0.5 * (Camera_Robot_Calibration_Point_5[14] - Camera_Robot_Calibration_Point_3[14]);

            target_point[0] = temp_line_segement_left[0] + (temp_line_segement_right[0] - temp_line_segement_left[0]) * k1;
            target_point[1] = temp_line_segement_left[1] + (temp_line_segement_right[1] - temp_line_segement_left[1]) * k1;
            target_point[2] = temp_line_segement_left[2] + (temp_line_segement_right[2] - temp_line_segement_left[2]) * k1;
        }
        else{                           // right

            temp_line_segement_left[0] = Camera_Robot_Calibration_Point_3[12] + (1 - k1)/0.5 * (Camera_Robot_Calibration_Point_5[12] - Camera_Robot_Calibration_Point_3[12]);
            temp_line_segement_left[1] = Camera_Robot_Calibration_Point_3[13] + (1 - k1)/0.5 * (Camera_Robot_Calibration_Point_5[13] - Camera_Robot_Calibration_Point_3[13]);
            temp_line_segement_left[2] = Camera_Robot_Calibration_Point_3[14] + (1 - k1)/0.5 * (Camera_Robot_Calibration_Point_5[14] - Camera_Robot_Calibration_Point_3[14]);

            temp_line_segement_right[0] = Camera_Robot_Calibration_Point_4[12] + (1 - k1)/0.5 * (Camera_Robot_Calibration_Point_5[12] - Camera_Robot_Calibration_Point_4[12]);
            temp_line_segement_right[1] = Camera_Robot_Calibration_Point_4[13] + (1 - k1)/0.5 * (Camera_Robot_Calibration_Point_5[13] - Camera_Robot_Calibration_Point_4[13]);
            temp_line_segement_right[2] = Camera_Robot_Calibration_Point_4[14] + (1 - k1)/0.5 * (Camera_Robot_Calibration_Point_5[14] - Camera_Robot_Calibration_Point_4[14]);

            target_point[0] = temp_line_segement_left[0] + (temp_line_segement_right[0] - temp_line_segement_left[0]) * k2;
            target_point[1] = temp_line_segement_left[1] + (temp_line_segement_right[1] - temp_line_segement_left[1]) * k2;
            target_point[2] = temp_line_segement_left[2] + (temp_line_segement_right[2] - temp_line_segement_left[2]) * k2;
        }
    }else{                              
        if ((k1+k2 <= 1)){              // left

            temp_line_segement_left[0] = Camera_Robot_Calibration_Point_1[12] + k1/0.5 * (Camera_Robot_Calibration_Point_5[12] - Camera_Robot_Calibration_Point_1[12]);
            temp_line_segement_left[1] = Camera_Robot_Calibration_Point_1[13] + k1/0.5 * (Camera_Robot_Calibration_Point_5[13] - Camera_Robot_Calibration_Point_1[13]);
            temp_line_segement_left[2] = Camera_Robot_Calibration_Point_1[14] + k1/0.5 * (Camera_Robot_Calibration_Point_5[14] - Camera_Robot_Calibration_Point_1[14]);

            temp_line_segement_right[0] = Camera_Robot_Calibration_Point_2[12] + k1/0.5 * (Camera_Robot_Calibration_Point_5[12] - Camera_Robot_Calibration_Point_2[12]);
            temp_line_segement_right[1] = Camera_Robot_Calibration_Point_2[13] + k1/0.5 * (Camera_Robot_Calibration_Point_5[13] - Camera_Robot_Calibration_Point_2[13]);
            temp_line_segement_right[2] = Camera_Robot_Calibration_Point_2[14] + k1/0.5 * (Camera_Robot_Calibration_Point_5[14] - Camera_Robot_Calibration_Point_2[14]);

            target_point[0] = temp_line_segement_left[0] + (temp_line_segement_right[0] - temp_line_segement_left[0]) * (1-k2);
            target_point[1] = temp_line_segement_left[1] + (temp_line_segement_right[1] - temp_line_segement_left[1]) * (1-k2);
            target_point[2] = temp_line_segement_left[2] + (temp_line_segement_right[2] - temp_line_segement_left[2]) * (1-k2);
        }
        else{                           // bottom

            temp_line_segement_left[0] = Camera_Robot_Calibration_Point_1[12] + (1-k2) / 0.5 * (Camera_Robot_Calibration_Point_5[12] - Camera_Robot_Calibration_Point_1[12]);
            temp_line_segement_left[1] = Camera_Robot_Calibration_Point_1[13] + (1-k2) / 0.5 * (Camera_Robot_Calibration_Point_5[13] - Camera_Robot_Calibration_Point_1[13]);
            temp_line_segement_left[2] = Camera_Robot_Calibration_Point_1[14] + (1-k2) / 0.5 * (Camera_Robot_Calibration_Point_5[14] - Camera_Robot_Calibration_Point_1[14]);

            temp_line_segement_right[0] = Camera_Robot_Calibration_Point_4[12] + (1-k2) / 0.5 * (Camera_Robot_Calibration_Point_5[12] - Camera_Robot_Calibration_Point_4[12]);
            temp_line_segement_right[1] = Camera_Robot_Calibration_Point_4[13] + (1-k2) / 0.5 * (Camera_Robot_Calibration_Point_5[13] - Camera_Robot_Calibration_Point_4[13]);
            temp_line_segement_right[2] = Camera_Robot_Calibration_Point_4[14] + (1-k2) / 0.5 * (Camera_Robot_Calibration_Point_5[14] - Camera_Robot_Calibration_Point_4[14]);

            target_point[0] = temp_line_segement_left[0] + (temp_line_segement_right[0] - temp_line_segement_left[0]) * k1;
            target_point[1] = temp_line_segement_left[1] + (temp_line_segement_right[1] - temp_line_segement_left[1]) * k1;
            target_point[2] = temp_line_segement_left[2] + (temp_line_segement_right[2] - temp_line_segement_left[2]) * k1;

        }
    }
    return target_point;
}

std::array<double, 16> Find_Camera_Target_Rotation_Transformation(double mousex, double mousey){

    double k1, k2;
    k1 = mousex;
    k2 = mousey;
    if (k1 < 0)
        k1 = 0;
    if (k1 >= 1)
        k1 = 1;
    if (k2 < 0)
        k2 = 0;
    if (k2 >= 1)
        k2 = 1;

    return {1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1};
}

class AsioTimer : public CppServer::Asio::Timer
{
public:
    using CppServer::Asio::Timer::Timer;
    std::vector<double> Last_Output;
    std::vector<double> Output;

    std::vector<double> Camera_Output;
    std::vector<double> Camera_Last_Output;
    bool First_Valid_MoCap_Data, First_Valid_Tablet_Data, Last_Output_Exist;
    int Sampling_Step, no_data_count, no_digital_tablet_data_count, button_fixed_countdown;
    // std::deque<storage_type> storage;
    storage_type temp_storage, last_output_storage, last_mocap_storage;
    Eigen::Vector2d temp_mocap_xy;
    Eigen::Vector3d robot_target_point, temp_mocap_projected_point, camera_target_point;
    Eigen::Vector4d Target_Drawing_Plane;
    double target_percentage_x, target_percentage_y, target_button, target_velocity_limit, temp_vx, temp_vy, target_pressure, target_azimuth, target_altitude;

    // Varaibles for calibration
    const double RETREAT_DISTANCE = 0.025, CALIBRATION_STEP = 0.0005, EXTERNAL_FORCE_THRESHOLD_HIGH = 10, EXTRA_RETREAT_DISTANCE = 0.035; // Retreat 50mm, step 0.5mm, threshold 3, 5N
    const double normal_drawing_retreat_distance = 0.025;
    double EXTERNAL_FORCE_THRESHOLD_LOW = 3;
    const int MOVING_WAITING_PERIOD = 10;
    static constexpr int SAMPLING_PERIOD = 20;

    bool currentCalibrationProcessFinished, lastCalibrationCommandPoseExist, calibrationinit, final_target, phase2init, phase3init, phase4init, calibration_finish, current_point_force_tare_exist;
    int currentCalibrationPointNo, currentCalibrationPointPhase, movingWaitingCounter, sampleStepCounter, action_branch_status = 0;
    double calibrationDistancetoSurface;
    Eigen::Vector3d currentCalibrationTargetCartPose;
    Eigen::Vector4d originalDrawingPlane, currentCalibrationPlane;
    std::vector<double> lastCalibrationCommandPose;
    std::vector<double> calibrationCommandPose;
    std::vector<double> calibratedPointPoses[3];
    std::array<double, SAMPLING_PERIOD> force_data{};
    std::vector<double> telemetryForce_tare;
    std::vector<double> telemetryForce;
    double calculated_telemtryForce;

    // Varibles for replay/draw file
    std::ifstream draw_file;
    bool file_valid_reading;
    double file_x, file_y, file_pressure, file_altitude, file_azimuth;
    int file_button;
    int verbose_output = 0;

    // Variables for record file
    std::ofstream record_file;
    int output_line_no = 0;


    // Variable for orienation control
    double last_altitude = 1.57079632679;

protected:
    void onTimer(bool canceled) override
    {
        // std::cout << "Asio timer " << (canceled ? "canceled" : "expired") << std::endl;
        AsioTimer::Setup(CppCommon::Timespan::milliseconds(5));
        AsioTimer::WaitAsync();
        // auto time_now = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> diff = time_now - time_start;
        // std::cout << diff.count() << std::endl;
        Sampling_Step++;
        // std::cout << Sampling_Step << std::endl;

        // State machine change start

        if ((receive_calibration_request))
        {
            if (current_state_machine_status == 0)
            {
                if (request_data.mutex.try_lock())
                {
                    std::cout << "Enter calibration mode!" << std::endl;
                    receive_calibration_request = false;
                    calibrationinit = true;
                    EXTERNAL_FORCE_THRESHOLD_LOW = request_data.calibration_target_force;
                    current_state_machine_status = 1; // Change current state machine status to calibration mode;
                    request_data.mutex.unlock();
                }
                else
                {
                    std::cout << "Unable to unlock mutex, will try again" << std::endl;
                }
            }
            else
            {
                std::cout << "Unable to conduct calibration when it's in other modes" << std::endl;
                receive_calibration_request = false;
            }
        }

        if ((receive_file_record_request))
        {
            if (current_state_machine_status == 0)
            {
                if (request_data.mutex.try_lock())
                {
                    std::filesystem::path temp_p = fileDirectoryLocation + request_data.record_file_name;
                    record_file.open(temp_p);
                    if (record_file.fail())
                    {
                        std::cout << "Unable to save to record file, please check file!" << std::endl;
                    }
                    else
                    {
                        std::cout << "Successfully open file" << std::endl;
                        record_file << convertChronoToString(std::chrono::system_clock::now()) << std::endl;
                        current_state_machine_status = 2;
                        output_line_no = 0;
                    }
                    receive_file_record_request = false;
                    request_data.mutex.unlock();
                }
                else
                {
                    std::cout << "Unable to unlock mutex, will try again" << std::endl;
                }
            }
            else
            {
                std::cout << "Unable to conduct file record when it's in other modes" << std::endl;
                receive_file_record_request = false;
            }
        }

        if (receive_stop_file_record_request)
        {
            if (current_state_machine_status == 2)
            {
                record_file.close();
                std::cout << "Close current record file" << std::endl;
                current_state_machine_status = 0;
                receive_stop_file_record_request = false;
            }
            else
            {
                std::cout << "Error: Currently not in record file mode!" << std::endl;
                receive_stop_file_record_request = false;
            }
        }

        if ((receive_file_draw_request))
        {
            if ((current_state_machine_status == 0))
            {
                if (draw_file.is_open())
                {
                    std::cout << "Draw file already opened, something wrong!" << std::endl;
                }
                else
                {
                    if (request_data.mutex.try_lock())
                    {
                        std::filesystem::path temp_p = fileDirectoryLocation + request_data.draw_file_name;
                        if (std::filesystem::exists(temp_p))
                        {
                            std::cout << "File: " << request_data.draw_file_name << " exist!" << std::endl;
                            draw_file.open(temp_p);
                            if (draw_file.fail())
                            {
                                std::cout << "Unable to open file, please check file!" << std::endl;
                            }
                            else
                            {
                                std::cout << "Successfully open file" << std::endl;
                                std::cout << "First line: ";
                                std::string temp_line;
                                std::getline(draw_file, temp_line);
                                std::cout << temp_line << std::endl;
                                verbose_output = 0;
                                current_state_machine_status = 3;
                                receive_file_draw_request = false;
                            }
                        }
                        else
                        {
                            std::cout << "File: " << request_data.draw_file_name << " doesn't exist!" << std::endl;
                        }
                        request_data.mutex.unlock();
                    }
                }
            }
            else
            {
                std::cout << "Unable to conduct draw file when it's in other modes" << std::endl;
                receive_file_draw_request = false;
            }
        }

        // State machine change finish

        // Action branch -> 0 for normal pos x, y based control, ->1 for calibraion
        switch (current_state_machine_status)
        {
        case 0:
        {
            action_branch_status = 0;
            break;
        }
        case 1:
        {
            action_branch_status = 1;
            break;
        }
        case 2:
        {
            action_branch_status = 0;
            break;
        }
        case 3:
        {
            action_branch_status = 0;
            break;
        }
        default:
            std::cout << "Error state machine status!" << std::endl;
        }
        // std::cout << "Action Switch:" << action_branch_status << std::endl;

        // Action branch switch

        // Store telemetry force data first

        if (telemetry_data.mutex.try_lock())
        {
            telemetryForce = telemetry_data.external_force;
            telemetry_data.mutex.unlock();
        }
        if (telemetryForce.size() < 6)
        {
            telemetryForce.assign(6, 0.0);
        }

        switch (action_branch_status)
        {
        case 0:
        {
            if (output_data.mutex.try_lock())
            {
                if (output_data.has_tablet_data)
                {
                    temp_storage.has_tablet_data = true;
                    temp_storage.tablet_data[0] = output_data.tablet_data[0];
                    temp_storage.tablet_data[1] = output_data.tablet_data[1];
                    temp_storage.tablet_data[2] = output_data.tablet_data[2];
                    temp_storage.tablet_data[3] = output_data.tablet_data[3];
                    temp_storage.tablet_data[4] = output_data.tablet_data[4];
                    temp_storage.on_paper = output_data.on_paper;
                }
                if (output_data.has_mocap_data)
                {
                    temp_storage.has_mocap_data = true;
                    temp_storage.mocap_data[0] = output_data.mocap_data[0];
                    temp_storage.mocap_data[1] = output_data.mocap_data[1];
                    temp_storage.mocap_data[2] = output_data.mocap_data[2];
                }
                temp_storage.has_tablet_data = output_data.has_tablet_data;
                temp_storage.has_mocap_data = output_data.has_mocap_data;
                output_data.has_mocap_data = false;
                output_data.has_tablet_data = false;
                output_data.on_paper = false;
                output_data.mutex.unlock();
            }

            if (temp_storage.has_tablet_data)
            {
                if (First_Valid_Tablet_Data)
                {
                    if ((temp_storage.on_paper == 1) && (last_output_storage.on_paper == 0))
                    {
                        if ((abs(temp_storage.tablet_data[0] - last_output_storage.output_x) > 0.1) || (abs(temp_storage.tablet_data[1] - last_output_storage.output_y) > 0.1))
                        {
                            button_fixed_countdown = 100;
                        }
                    }
                }
                target_percentage_x = temp_storage.tablet_data[0];
                target_percentage_y = temp_storage.tablet_data[1];
                target_pressure = temp_storage.tablet_data[2];
                target_altitude = temp_storage.tablet_data[3];
                target_azimuth  = temp_storage.tablet_data[4];

                target_button = temp_storage.on_paper;
                First_Valid_Tablet_Data = true;
                if (temp_storage.has_mocap_data)
                {
                    temp_mocap_projected_point = Find_Projected_Point(Eigen::Vector3d(temp_storage.mocap_data[0] - Current_Origin_Movement[0], temp_storage.mocap_data[1] - Current_Origin_Movement[1], temp_storage.mocap_data[2] - Current_Origin_Movement[2]), MoCap_Drawing_Plane);
                    temp_mocap_xy = Find_Target_Point_MoCap_X_Y(temp_mocap_projected_point, MoCap_Drawing_Plane, Button_Modified_Drawing_Plane);
                    last_mocap_storage.mocap_x = temp_mocap_xy[0];
                    last_mocap_storage.mocap_y = temp_mocap_xy[1];
                    last_mocap_storage.time_step = Sampling_Step;
                    First_Valid_MoCap_Data = true;
                }
                no_data_count = 0;
                no_digital_tablet_data_count = 0;
            }
            else
            {
                no_digital_tablet_data_count++;
            }
            if ((First_Valid_Tablet_Data) && (no_digital_tablet_data_count > 10))
            {
                target_button = 0;
            }
            if ((!temp_storage.has_tablet_data) && temp_storage.has_mocap_data)
            {
                temp_mocap_projected_point = Find_Projected_Point(Eigen::Vector3d(temp_storage.mocap_data[0] - Current_Origin_Movement[0], temp_storage.mocap_data[1] - Current_Origin_Movement[1], temp_storage.mocap_data[2] - Current_Origin_Movement[2]), MoCap_Drawing_Plane);
                temp_mocap_xy = Find_Target_Point_MoCap_X_Y(temp_mocap_projected_point, MoCap_Drawing_Plane, Button_Modified_Drawing_Plane);
                if ((no_digital_tablet_data_count > 20) && (First_Valid_MoCap_Data) && (last_output_storage.on_paper == 0))
                { // Indicate no datapoint from digital tablet
                    temp_vx = temp_mocap_xy[0] - last_mocap_storage.mocap_x;
                    temp_vy = temp_mocap_xy[1] - last_mocap_storage.mocap_y;
                    target_percentage_x = last_output_storage.output_x + temp_vx * 0.25;
                    target_percentage_y = last_output_storage.output_y + temp_vy * 0.25;
                    target_button = 0;
                }
                last_mocap_storage.mocap_x = temp_mocap_xy[0];
                last_mocap_storage.mocap_y = temp_mocap_xy[1];
                last_mocap_storage.time_step = Sampling_Step;
                First_Valid_MoCap_Data = true;
                no_data_count = 0;
            }

            if ((!temp_storage.has_tablet_data) && (!temp_storage.has_mocap_data))
            {
                no_data_count++;
                if (First_Valid_Tablet_Data)
                {
                    if (no_data_count > 10)
                    {                      // No valid datapoint for 5 consecutive points
                        target_button = 0; // Move robot back from paper
                    }
                }
            }

            if (current_state_machine_status == 2)
            {
                output_line_no++;
                record_file << output_line_no << " " << target_percentage_x << " " << target_percentage_y << " " << target_button << " " << target_altitude << std::endl;
            }

            if (current_state_machine_status == 3)
            {
                std::string temp_line, temp_line_sub_string;
                std::getline(draw_file, temp_line);
                file_valid_reading = true;
                file_altitude = 1.57079632679;
                if (draw_file.good())
                {
                    // std::cout << temp_line << std::endl;
                    std::string::size_type temp_pos;
                    double no_of_line;
                    try
                    {
                        no_of_line = std::stod(temp_line, &temp_pos);
                        temp_line_sub_string = temp_line.substr(temp_pos);
                        try
                        {
                            file_x = std::stod(temp_line_sub_string, &temp_pos);
                            temp_line_sub_string = temp_line_sub_string.substr(temp_pos);
                            try
                            {
                                file_y = 1.0 - std::stod(temp_line_sub_string, &temp_pos);
                                temp_line_sub_string = temp_line_sub_string.substr(temp_pos);
                                try
                                {
                                    file_button = std::stoi(temp_line_sub_string, &temp_pos);
                                    temp_line_sub_string = temp_line_sub_string.substr(temp_pos);
                                    if (temp_line_sub_string.find_first_not_of(" \t\r\n") != std::string::npos)
                                    {
                                        try
                                        {
                                            file_altitude = std::stod(temp_line_sub_string, &temp_pos);
                                        }
                                        catch (const std::invalid_argument &)
                                        {
                                            file_valid_reading = false;
                                            std::cerr << "Argument is invalid\n";
                                        }
                                        catch (const std::out_of_range &)
                                        {
                                            file_valid_reading = false;
                                            std::cerr << "Argument is out of range for a double\n";
                                        }
                                    }
                                }
                                catch (const std::invalid_argument &)
                                {
                                    file_valid_reading = false;
                                    std::cerr << "Argument is invalid\n";
                                }
                                catch (const std::out_of_range &)
                                {
                                    file_valid_reading = false;
                                    std::cerr << "Argument is out of range for a double\n";
                                }
                            }
                            catch (const std::invalid_argument &)
                            {
                                file_valid_reading = false;
                                std::cerr << "Argument is invalid\n";
                            }
                            catch (const std::out_of_range &)
                            {
                                file_valid_reading = false;
                                std::cerr << "Argument is out of range for a double\n";
                            }
                        }
                        catch (const std::invalid_argument &)
                        {
                            file_valid_reading = false;
                            std::cerr << "Argument is invalid\n";
                        }
                        catch (const std::out_of_range &)
                        {
                            file_valid_reading = false;
                            std::cerr << "Argument is out of range for a double\n";
                        }
                    }
                    catch (const std::invalid_argument &)
                    {
                        file_valid_reading = false;
                        std::cerr << "Argument is invalid\n";
                    }
                    catch (const std::out_of_range &)
                    {
                        file_valid_reading = false;
                        std::cerr << "Argument is out of range for a double\n";
                    }
                    if (file_valid_reading)
                    {
                        target_percentage_x = file_x;
                        target_percentage_y = file_y;
                        target_button = file_button;
                        target_altitude = file_altitude;
                        if (verbose_output % 2000 == 0)
                        {
                            std::cout << "Line No. " << verbose_output << " | " << file_x << " " << file_y << " " << file_button << std::endl;
                        }
                        verbose_output++;
                    }
                    else
                    {
                        std::cout << "Error in converting numbers in file" << std::endl;
                        draw_file.close();
                        current_state_machine_status = 0;
                    }
                }
                else
                {
                    std::cout << "Draw file finish, return to normal drawing mode" << std::endl;
                    draw_file.close();
                    current_state_machine_status = 0;
                }
            }

            if (First_Valid_Tablet_Data)
            { // Output Data to telepresence interface program, always, same 0.5 0.5
                last_output_storage.output_x = target_percentage_x;
                last_output_storage.output_y = target_percentage_y;
                last_output_storage.on_paper = target_button;
                // std::cout << "X: " << target_percentage_x << " Y: " << target_percentage_y << " Button: " << target_button << std::endl;
                if ((target_button == 0))
                {
                    Target_Drawing_Plane[3] += normal_drawing_retreat_distance;
                    if (Target_Drawing_Plane[3] > (Robot_Drawing_Plane[3] + normal_drawing_retreat_distance))
                    { // Direction based on sign of Robot_Drawing_Plane
                        Target_Drawing_Plane[3] = Robot_Drawing_Plane[3] + User_Button_Modification + normal_drawing_retreat_distance;
                    }
                    target_velocity_limit = Move_Vlimit;
                    if (button_fixed_countdown > 0)
                        button_fixed_countdown--;
                    Force_Regulate_Modification = 0;
                    // std::cout << "Not On Plane" << std::endl;
                }
                else
                {
                    if (button_fixed_countdown > 0)
                    {
                        target_button = 0;
                        button_fixed_countdown--;
                        Target_Drawing_Plane[3] += normal_drawing_retreat_distance;
                        if (Target_Drawing_Plane[3] > (Robot_Drawing_Plane[3] + normal_drawing_retreat_distance))
                        { // Direction
                            Target_Drawing_Plane[3] = Robot_Drawing_Plane[3] + User_Button_Modification + normal_drawing_retreat_distance;
                        }
                        target_velocity_limit = Move_Vlimit;
                        Force_Regulate_Modification = 0;
                        // std::cout << "On Plane, fixed time" << std::endl;
                    }
                    else
                    {
                        Target_Drawing_Plane[3] -= normal_drawing_retreat_distance;
                        if (Target_Drawing_Plane[3] < Robot_Drawing_Plane[3])
                        { // Direction
                            Target_Drawing_Plane[3] = Robot_Drawing_Plane[3] + User_Button_Modification;
                            calculated_telemtryForce = sqrt((telemetryForce[0] * telemetryForce[0]) + (telemetryForce[1] * telemetryForce[1]));
                            // calculated_telemtryForce = telemetryForce[2];
                            if (calculated_telemtryForce < 3)
                            {
                                Force_Regulate_Modification -= 0.0001;
                            }
                            else
                            {
                                if (calculated_telemtryForce > 8)
                                {
                                    Force_Regulate_Modification += 0.0001;
                                }
                            }
                            if (Force_Regulate_Modification < -0.001)
                            {
                                Force_Regulate_Modification = -0.001;
                            }
                            if (Force_Regulate_Modification > 0.005)
                            {
                                Force_Regulate_Modification = 0.005;
                            }
                            Target_Drawing_Plane[3] = Robot_Drawing_Plane[3] + Force_Regulate_Modification;
                        }
                        target_velocity_limit = Draw_Vlimit;
                        // std::cout << "On plane" << "Force: " << abs(telemetryForce[1]) << " Mod: " << Force_Regulate_Modification << std::endl;
                    }
                }
                if (!Last_Output_Exist)
                {
                    Target_Drawing_Plane[3] -= EXTRA_RETREAT_DISTANCE; // First handshake drawing plane
                }
                robot_target_point = Find_Target_Point_X_Y(target_percentage_x, target_percentage_y, Target_Drawing_Plane);

                camera_target_point = Find_Camera_Target_Translation_Transformation(target_percentage_x, target_percentage_y); 
                // Test Change Orientation

                Eigen::Matrix3f Additional_Rotation_Matrix;
                Eigen::Matrix3f Original_Rotation_Matrix;
                Eigen::Matrix3f Output_Matrix; 

                Original_Rotation_Matrix(0,0) = Robot_Calibration_Point_1[0];
                Original_Rotation_Matrix(1,0) = Robot_Calibration_Point_1[1];
                Original_Rotation_Matrix(2,0) = Robot_Calibration_Point_1[2];

                Original_Rotation_Matrix(0,1) = Robot_Calibration_Point_1[4];
                Original_Rotation_Matrix(1,1) = Robot_Calibration_Point_1[5];
                Original_Rotation_Matrix(2,1) = Robot_Calibration_Point_1[6];

                Original_Rotation_Matrix(0,2) = Robot_Calibration_Point_1[8];
                Original_Rotation_Matrix(1,2) = Robot_Calibration_Point_1[9];
                Original_Rotation_Matrix(2,2) = Robot_Calibration_Point_1[10];

                //target_altitude  / target azimuth test

                // std::cout << "Target_Altitude " <<  target_altitude << std::endl;

                if (target_altitude < 1.35){
                    target_altitude = 1.35;
                }

                if ((target_altitude - last_altitude) > 0.001){
                    target_altitude = last_altitude + 0.001;
                }

                if ((target_altitude - last_altitude) < -0.001){
                    target_altitude = last_altitude - 0.001;
                }



                Additional_Rotation_Matrix = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(1.57079632679 - target_altitude, Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
                
                last_altitude  = target_altitude;

                Output_Matrix = Original_Rotation_Matrix; // * Additional_Rotation_Matrix;

                Output[0]  = Output_Matrix(0,0);
                Output[1]  = Output_Matrix(1,0);
                Output[2]  = Output_Matrix(2,0);
                
                Output[4]  = Output_Matrix(0,1);
                Output[5]  = Output_Matrix(1,1);
                Output[6]  = Output_Matrix(2,1);

                Output[8]  = Output_Matrix(0,2);
                Output[9]  = Output_Matrix(1,2);
                Output[10]  = Output_Matrix(2,2);


                Output[12] = robot_target_point[0];
                Output[13] = robot_target_point[1];
                Output[14] = robot_target_point[2];

                Camera_Output[12] = camera_target_point[0];
                Camera_Output[13] = camera_target_point[1];
                Camera_Output[14] = camera_target_point[2];

                if (Last_Output_Exist)
                {
                    double Difference[3], Distance, Modified_Distance, Ratio;
                    Difference[0] = Output[12] - Last_Output[12];
                    Difference[1] = Output[13] - Last_Output[13];
                    Difference[2] = Output[14] - Last_Output[14];
                    Distance = sqrt((Difference[0] * Difference[0]) + (Difference[1] * Difference[1]) + (Difference[2] * Difference[2]));
                    if (Distance >= target_velocity_limit)
                    {
                        Ratio = target_velocity_limit / Distance;
                        Output[12] = Ratio * Difference[0] + Last_Output[12];
                        Output[13] = Ratio * Difference[1] + Last_Output[13];
                        Output[14] = Ratio * Difference[2] + Last_Output[14];
                    }
                }

                if (Last_Output_Exist)
                {
                    double Difference[3], Distance, Modified_Distance, Ratio;
                    Difference[0] = Camera_Output[12] - Camera_Last_Output[12];
                    Difference[1] = Camera_Output[13] - Camera_Last_Output[13];
                    Difference[2] = Camera_Output[14] - Camera_Last_Output[14];
                    Distance = sqrt((Difference[0] * Difference[0]) + (Difference[1] * Difference[1]) + (Difference[2] * Difference[2]));
                    if (Distance >= target_velocity_limit/2)
                    {
                        Ratio = target_velocity_limit/2 / Distance;
                        Camera_Output[12] = Ratio * Difference[0] + Camera_Last_Output[12];
                        Camera_Output[13] = Ratio * Difference[1] + Camera_Last_Output[13];
                        Camera_Output[14] = Ratio * Difference[2] + Camera_Last_Output[14];
                    }
                }

                static int output_pose_log_counter = 0;
                if (++output_pose_log_counter >= 200)
                {
                    output_pose_log_counter = 0;
                    std::cout << "Output Cart Pose: " << Output[12] << " " << Output[13] << " " << Output[14] << std::endl;
                }
            }
            break;
        }
        case 1:
        {

            // Initilization:

            if (calibrationinit)
            {
                std::cout << "Enter Calibration Process\nEnter Phase 1" << std::endl;
                originalDrawingPlane = Robot_Drawing_Plane; // Reinitial drawing plane
                currentCalibrationPointNo = 0;
                currentCalibrationPointPhase = 1;
                calibrationDistancetoSurface = RETREAT_DISTANCE;
                current_point_force_tare_exist = false;
                calibrationinit = false;
                lastCalibrationCommandPoseExist = false;
                final_target = false;
                calibration_finish = false;
            }

            // if calibration finshed, directly break
            if (calibration_finish)
            {
                std::cout << "Calibration proces finished, enter normal drawing mode" << std::endl;
                current_state_machine_status = 0;
                break;
            }
            // Add original calibration pose to the current calibration process
            currentCalibrationTargetCartPose[0] = Robot_Calibration_Points[currentCalibrationPointNo][12];
            currentCalibrationTargetCartPose[1] = Robot_Calibration_Points[currentCalibrationPointNo][13];
            currentCalibrationTargetCartPose[2] = Robot_Calibration_Points[currentCalibrationPointNo][14];

            // Depend on the Phase
            // Phase 1: Goes to certain pose
            // Phase 2: Feedforward into the paper plane
            // Phase 3: Sampling force
            // Phase 4: Retreat back from the paper plane
            switch (this->currentCalibrationPointPhase)
            {
            case 1:
            { // Move to the targeted calibraion position
                // Cartesian pose motion limiter (Only on end effector position part)
                currentCalibrationPlane = originalDrawingPlane;
                currentCalibrationPlane[3] += calibrationDistancetoSurface;
                currentCalibrationTargetCartPose = Find_Projected_Point(currentCalibrationTargetCartPose, currentCalibrationPlane);

                Output[12] = currentCalibrationTargetCartPose[0];
                Output[13] = currentCalibrationTargetCartPose[1];
                Output[14] = currentCalibrationTargetCartPose[2];
                final_target = true;

                //                if (!lastCalibrationCommandPoseExist){
                //                    double difference[3], distance, ratio;
                //                    difference[0] = currentCalibrationTargetCartPose[0] - telemetryPose[12];
                //                    difference[1] = currentCalibrationTargetCartPose[1] - telemetryPose[13];
                //                    difference[2] = currentCalibrationTargetCartPose[2] - telemetryPose[14];

                //                    distance = sqrt((difference[0] * difference[0]) + (difference[1] * difference[1]) +
                //                            (difference[2] * difference[2]));
                //                    if (distance >= this->targetVelocityLimit) {
                //                        ratio = this->targetVelocityLimit / distance;
                //                        this->miosOutput[12] = ratio * difference[0] + telemetryPose[12];
                //                        this->miosOutput[13] = ratio * difference[1] + telemetryPose[13];
                //                        this->miosOutput[14] = ratio * difference[2] + telemetryPose[14];
                //                        final_target = false;
                //                    }
                //                }
                //                else{
                double difference[3], distance, ratio;
                difference[0] = currentCalibrationTargetCartPose[0] - Last_Output[12];
                difference[1] = currentCalibrationTargetCartPose[1] - Last_Output[13];
                difference[2] = currentCalibrationTargetCartPose[2] - Last_Output[14];

                distance = sqrt((difference[0] * difference[0]) + (difference[1] * difference[1]) +
                                (difference[2] * difference[2]));

                if (distance >= targetVelocityLimit)
                {
                    ratio = targetVelocityLimit / distance;
                    Output[12] = ratio * difference[0] + Last_Output[12];
                    Output[13] = ratio * difference[1] + Last_Output[13];
                    Output[14] = ratio * difference[2] + Last_Output[14];
                    final_target = false;
                }

                if (final_target)
                {                                   // Reach the final target -> finish phase 1
                    currentCalibrationPointPhase++; // Goto phase 2
                    phase2init = true;
                }
                break;
            }
            case 2:
            { // Feedforward into the paper plane
                if (phase2init)
                {
                    // Phase 2 variable init
                    std::cout << "Enter Phase 2" << std::endl;
                    calibrationDistancetoSurface -= CALIBRATION_STEP;
                    std::cout << "Calibration distance " << calibrationDistancetoSurface << std::endl;
                    movingWaitingCounter = 0;
                    phase2init = false;
                }
                else
                {
                    movingWaitingCounter++;
                }
                currentCalibrationPlane = originalDrawingPlane;
                currentCalibrationPlane[3] += calibrationDistancetoSurface;
                currentCalibrationTargetCartPose = Find_Projected_Point(currentCalibrationTargetCartPose, currentCalibrationPlane);
                Output[12] = currentCalibrationTargetCartPose[0];
                Output[13] = currentCalibrationTargetCartPose[1];
                Output[14] = currentCalibrationTargetCartPose[2];
                // std::cout << Output[12] << " " << Output[13] << " " << Output[14] << std::endl;
                if (movingWaitingCounter >= MOVING_WAITING_PERIOD)
                {
                    currentCalibrationPointPhase++;
                    phase3init = true;
                    // Waiting time acheived, go to phase 3
                }
                break;
            }
            case 3:
            { // Sampling force
                if (phase3init)
                {
                    // Phase 3 variable init
                    std::cout << "Enter Phase 3" << std::endl;
                    sampleStepCounter = 0;
                    phase3init = false;
                    if (!current_point_force_tare_exist)
                    {
                        telemetryForce_tare.resize(3);
                        telemetryForce_tare[0] = telemetryForce[0];
                        telemetryForce_tare[1] = telemetryForce[1];
                        telemetryForce_tare[2] = telemetryForce[2];
                        std::cout << "Use current force as tare base for current point" << telemetryForce_tare[0] << " " << telemetryForce_tare[1] << " " << telemetryForce_tare[2] << std::endl;
                        current_point_force_tare_exist = true;
                    }
                }

                //                Output external force readings
                std::cout << "Telemetry external force: ";
                std::cout << telemetryForce[0] << " " << telemetryForce[1] << " " << telemetryForce[2] << std::endl;

                force_data[sampleStepCounter] = sqrt((telemetryForce[0]-telemetryForce_tare[0]) * (telemetryForce[0]-telemetryForce_tare[0]) + (telemetryForce[2]-telemetryForce_tare[2]) * (telemetryForce[2]-telemetryForce_tare[2]));
                //                        (telemetryForce[0]-telemetryForce_tare[0]) * (telemetryForce[0]-telemetryForce_tare[0]) +
                //                        (telemetryForce[1]-telemetryForce_tare[1]) * (telemetryForce[1]-telemetryForce_tare[1]) +
                //                        (telemetryForce[2]-telemetryForce_tare[2]) * (telemetryForce[2]-telemetryForce_tare[2]));

                sampleStepCounter++;
                if (sampleStepCounter >= SAMPLING_PERIOD)
                {
                    double total_force = 0;
                    for (auto temp : force_data)
                        total_force += temp;
                    total_force /= SAMPLING_PERIOD;
                    std::cout << "Average External Force: " << total_force << std::endl;
                    if (total_force >= EXTERNAL_FORCE_THRESHOLD_LOW)
                    {
                        // Enough force
                        calibratedPointPoses[currentCalibrationPointNo] = Last_Output;
                        currentCalibrationPointPhase++;
                        phase4init = true;
                    }
                    else
                    {
                        // Not enough force
                        std::cout << "Not Enough Force, go back to phase 2" << std::endl;
                        currentCalibrationPointPhase--;
                        phase2init = true;
                    }
                    if (total_force > EXTERNAL_FORCE_THRESHOLD_HIGH)
                        std::cout << "High external force, try reduce the step" << std::endl;
                }
                break;
            }
            case 4:
            { // Move away from the plane
                if (phase4init)
                {
                    std::cout << "Enter Phase 4" << std::endl;
                    phase4init = false;
                }
                currentCalibrationPlane = originalDrawingPlane;
                currentCalibrationPlane[3] += RETREAT_DISTANCE;
                currentCalibrationTargetCartPose = Find_Projected_Point(currentCalibrationTargetCartPose, currentCalibrationPlane);

                Output[12] = currentCalibrationTargetCartPose[0];
                Output[13] = currentCalibrationTargetCartPose[1];
                Output[14] = currentCalibrationTargetCartPose[2];
                final_target = true;

                double difference[3], distance, ratio;
                difference[0] = currentCalibrationTargetCartPose[0] - Last_Output[12];
                difference[1] = currentCalibrationTargetCartPose[1] - Last_Output[13];
                difference[2] = currentCalibrationTargetCartPose[2] - Last_Output[14];

                distance = sqrt((difference[0] * difference[0]) + (difference[1] * difference[1]) +
                                (difference[2] * difference[2]));
                if (distance >= targetVelocityLimit)
                {
                    ratio = targetVelocityLimit / distance;
                    Output[12] = ratio * difference[0] + Last_Output[12];
                    Output[13] = ratio * difference[1] + Last_Output[13];
                    Output[14] = ratio * difference[2] + Last_Output[14];
                    final_target = false;
                }

                if (final_target)
                { // Reach the final target -> finish phase 4
                    std::cout << "Finish retreat back from plane" << std::endl;
                    std::cout << "Finish calibration on point " << currentCalibrationPointNo << std::endl;
                    currentCalibrationPointNo++;
                    currentCalibrationPointPhase = 1;
                    current_point_force_tare_exist = false;
                    calibrationDistancetoSurface = RETREAT_DISTANCE;
                }
                break;
            }
            }
            if (currentCalibrationPointNo >= 3)
            {
                std::cout << "Finish all points calibration" << std::endl;

                std::cout << "Updating calibrated plane" << std::endl;
                update_robot_plane(calibratedPointPoses);
                Target_Drawing_Plane = Robot_Drawing_Plane;
                std::cout << "New calibrated points: " << std::endl;
                calibration_finish = true;

                std::string filename, filenameStart = configDirectoryLocation + "Robot_Calibration_Point_";
                std::string filenameEnd = "_" + convertChronoToString(std::chrono::system_clock::now()) + ".txt";

                for (int i = 0; i <= 2; i++)
                {
                    filename = filenameStart;
                    filename += std::to_string(i + 1);
                    filename += filenameEnd;
                    std::ofstream fout(filename);
                    std::cout << "Point " << i << ": " << std::endl;
                    for (auto temp : calibratedPointPoses[i])
                    {
                        std::cout << temp << " ";
                        fout << temp << " ";
                    }
                    std::cout << std::endl;
                    fout << std::endl;
                    fout.close();
                }
            }
            // Finish all points calibration process -> Update current calibration plane, points by new calibration files
            // lastCalibrationCommandPose = this->miosOutput;
            // lastCalibrationCommandPoseExist = true;
            break;
        }
        default:
            std::cout << "Error action branch status!" << std::endl;
        }

        udpSender->send(Output);
        // udpSender_Camera->send(Camera_Output);
        Last_Output_Exist = true;
        Last_Output = Output;
        Camera_Last_Output = Camera_Output;
    }

    void onError(int error, const std::string &category, const std::string &message) override
    {
        std::cout << "Asio timer caught an error with code " << error << " and category '" << category << "': " << message << std::endl;
    }
};

class Telemetry : public CppServer::Asio::UDPServer
{
public:
    using CppServer::Asio::UDPServer::UDPServer;

protected:
    void onStarted() override
    {
        // Start receive datagrams
        ReceiveAsync();
    }

    void onReceived(const asio::ip::udp::endpoint &endpoint, const void *buffer, size_t size) override
    {
        std::string message((const char *)buffer, size);
        // std::cout << "Received telemetry data " << message << std::endl;
        nlohmann::json temp_json_storage = nlohmann::json::parse(message);
        std::vector<double> temp_external_force(6);
        msrm_utils::read_json_param<double>(temp_json_storage, "O_F_ext_K", temp_external_force);
        // for (auto i:temp_external_force) std::cout << i << " "; std::cout << std:: endl;
        if (telemetry_data.mutex.try_lock())
        {
            telemetry_data.external_force = temp_external_force;
            telemetry_data.mutex.unlock();
        }
        ReceiveAsync();
    }

    void onSent(const asio::ip::udp::endpoint &endpoint, size_t sent) override
    {
        // Continue receive datagrams
        ReceiveAsync();
    }

    void onError(int error, const std::string &category, const std::string &message) override
    {
        std::cout << "Echo UDP server caught an error with code " << error << " and category '" << category << "': " << message << std::endl;
    }
};

#if 0
// class BVHUDPServer : public CppServer::Asio::UDPServer
// {
// public:
//     using CppServer::Asio::UDPServer::UDPServer;

// protected:
//     void onStarted() override
//     {
//         if (!tablet_mode)
//         {
//             bf::path sample_path = "../test_sample/Header_01.cmf"; //   "/home/lingyun/Documents/MoCap_Parser_New/test_sample/walk_01.bvh"
//             parser_test.parse(sample_path, &data_test);
//             // data_test.recalculate_joints_ltm();

//             for (int i = 0; i < data_test.joints().size(); i++)
//             {
//                 // std::cout << "NAME: " << data_test.joints()[i]->name() << std::endl; //<< ", " << utils_test::vec3tos(data_test.joints()[i]->pos(2));
//                 if (data_test.joints()[i]->name().compare(target_joint) == 0)
//                 {
//                     std::cout << "Target Joint Name" << data_test.joints()[i]->name() << std::endl;
//                     std::cout << "Get Joint Index Number " << i << std::endl;
//                     target_joing_index = i;
//                 }
//             }
//         }
//         if (output_data.mutex.try_lock())
//         {
//             output_data.has_mocap_data = false;
//             output_data.has_tablet_data = false;
//             output_data.on_paper = false;
//             output_data.tablet_data.resize(2);
//             output_data.mocap_data.resize(3);
//             output_data.mutex.unlock();
//         }
//         if (udpSender == nullptr)
//         {
//             std::cout << "Failed to open udp stream sender" << std::endl;
//         }
//         if (!udpSender->connect())
//         {
//             std::cout << "Failed to connect udp stream sender" << std::endl;
//         }
        
//         // if (udpSender_Camera == nullptr){
//         //     std::cout << "Failed to open camera udp stream sender" << std::endl;
//         // }

//         // if (!udpSender_Camera->connect()){
//         //     std::cout << "Failed to open camera udp stream sender" << std::endl;
//         // }

//         time_start = std::chrono::high_resolution_clock::now();

//         ReceiveAsync();
//     }

    void onReceived(const asio::ip::udp::endpoint &endpoint, const void *buffer, size_t size) override
    {
        std::string message((const char *)buffer, size);
        // udp_package_list.push_back(message);
        // std::cout << "Incoming Package Size: " << int(size) << std::endl;

        // auto time_now = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> diff = time_now - time_start;
        // record_file << diff.count() << " " << message << std::endl;

        if (size == 4)
        {
            ReceiveAsync();
            return;
        }

        // Simply identify message type and record to files
        if (size > 1000)
        {
            if (tablet_mode)
            {
                ReceiveAsync();
                return;
            }
            // parser_test.parse_motion_package((const char *)buffer, int(size));
            // data_test.recalculate_joints_ltm();
            packet_type = 2;
            // record_file << diff.count() << " " << data_test.joints()[target_joing_index]->pos(0)[0] << " " << data_test.joints()[target_joing_index]->pos(0)[1] << " " << data_test.joints()[target_joing_index]->pos(0)[2] << std::endl;
        }
        else
        {
            packet_type = 1;
            // record_file << diff.count() << " " << message << std::endl;
        }

        if (packet_type == 1)
        {

            // std::vector<double> input(16);
            char separater[] = " :,\t\r";
            char *token;
            char MotionData_Temp[100];
            double mouse_x, mouse_y, percentage_x, percentage_y, pressure, altitude, azimuth; // vlimit;
            int button, button_forward, button_backward, record_time;

            // Eigen::Vector3d robot_target_point;
            memset(MotionData_Temp, 0, sizeof(MotionData_Temp));
            memcpy(MotionData_Temp, (const char *)buffer, size);

            // std::cout << MotionData_Temp << std::endl;
            token = strtok(MotionData_Temp, separater);
            // std::cout << token << std::endl;
            if (strcmp(token, "Stylus")==0){
                button = 1;
                token = strtok(NULL, separater);
                mouse_x = atof(token);
                percentage_x = mouse_x / 955.0;
                // std::cout << "Stylus" << std::endl;
                // std::cout << "First Value " << percentage_x << std::endl;

                token = strtok(NULL, separater);
                mouse_y = atof(token);
                percentage_y = mouse_y / 954.0;

                token = strtok(NULL, separater);
                pressure = atof(token);

                token = strtok(NULL, separater);
                altitude = atof(token);

                token = strtok(NULL, separater);
                azimuth = atof(token);
                // std::cout << "Second Value " << percentage_y << std::endl;
            }
            else{
                button = 0;
                token = strtok(NULL, separater);
                mouse_x = atof(token);
                // std::cout << "First Value " << percentage_x << std::endl;

                token = strtok(NULL, separater);
                mouse_y = atof(token);
                
                percentage_x = 1.0 - mouse_y / 43200.0;
                percentage_y = mouse_x / 62200.0; 

		token = strtok(NULL, separater);
		button = atoi(token);
            // std::cout << "Second Value " << percentage_y << std::endl;

            }
            // record_time = atoi(token);

            // token = strtok(NULL, separater);
            // mouse_x = atof(token);
            // percentage_y = 1 - mouse_x / 2450.0;
            // // std::cout << "First Value " << percentage_x << std::endl;

            // token = strtok(NULL, separater);
            // mouse_y = atof(token);
            // percentage_x = mouse_y / 1440.0;
            // // std::cout << "Second Value " << percentage_y << std::endl;

            // token = strtok(NULL, separater);
            // button = atoi(token);
            // std::cout << "Third Vlaue " << button << std::endl;

            // std::cout << "Raw Packet: " << percentage_x << " " << percentage_y << " " << button << std::endl;

            if (output_data.mutex.try_lock())
            {
                output_data.has_tablet_data = true;
                output_data.tablet_data[0] = percentage_x;
                output_data.tablet_data[1] = percentage_y;
                output_data.tablet_data[2] = pressure;
                output_data.tablet_data[3] = altitude;
                output_data.tablet_data[4] = azimuth; 
                output_data.on_paper = button;
                output_data.mutex.unlock();
            }

            //            token = strtok(NULL, separater);
            //            button_forward = atoi(token);
            //            if (button_forward == 1){
            //                button_forward_pressed = true;
            //            }
            //            else{
            //                if (button_forward_pressed){
            //                    User_Button_Modification -=0.0005;
            //                    button_forward_pressed = false;
            //                }
            //            }
            //            token = strtok(NULL, separater);

            //            button_backward = atoi(token);
            //            if (button_backward == 1){
            //                button_backward_pressed = true;
            //            }
            //            else{
            //                if (button_backward_pressed){
            //                    User_Button_Modification +=0.0005;
            //                    button_backward_pressed = false;
            //                }
            //            }

            // record_file << "Time, Mouse_x, Mouse_y, Button, Button_Forward, Button_Backward" << std::endl;

            //            input[0]  = Robot_Calibration_Point_1[0];
            //            input[1]  = Robot_Calibration_Point_1[1];
            //            input[2]  = Robot_Calibration_Point_1[2];
            //            input[3]  = Robot_Calibration_Point_1[3];

            //            input[4]  = Robot_Calibration_Point_1[4];
            //            input[5]  = Robot_Calibration_Point_1[5];
            //            input[6]  = Robot_Calibration_Point_1[6];
            //            input[7]  = Robot_Calibration_Point_1[7];

            //            input[8]  = Robot_Calibration_Point_1[8];
            //            input[9]  = Robot_Calibration_Point_1[9];
            //            input[10] = Robot_Calibration_Point_1[10];
            //            input[11] = Robot_Calibration_Point_1[11];
            //            input[15] = 1;

            //            if (Last_Output_Exist){
            //                if ((button == 1) && (Last_button == 0)){
            //                    if ((abs(percentage_x - Last_percentage_X)>0.1) || (abs(percentage_y - Last_percentage_Y)>0.1)){
            //                        button_fixed_countdown = 400;
            //                    }
            //                }
            //            }
            //            Last_button = button;
            //            Last_percentage_X = percentage_x;
            //            Last_percentage_Y = percentage_y;

            //            if ((button==0)){
            //                Button_Modified_Drawing_Plane[3] += 0.035;
            //                if (Button_Modified_Drawing_Plane[3]>(Robot_Drawing_Plane[3] + 0.035)){
            //                    Button_Modified_Drawing_Plane[3] = Robot_Drawing_Plane[3] + User_Button_Modification + 0.035;
            //                }
            //                vlimit = Move_Vlimit;
            //                if (button_fixed_countdown >0) button_fixed_countdown --;
            //                std::cout << "Not On Plane" << std::endl;
            //            } else {
            //                if (button_fixed_countdown >0) {
            //                    button_fixed_countdown --;
            //                    Button_Modified_Drawing_Plane[3] += 0.035;
            //                    if (Button_Modified_Drawing_Plane[3]>(Robot_Drawing_Plane[3] + 0.035)){
            //                        Button_Modified_Drawing_Plane[3] = Robot_Drawing_Plane[3] + User_Button_Modification + 0.035;
            //                    }
            //                    vlimit = Move_Vlimit;
            //                    std::cout << "On Plane, fixed time" << std::endl;
            //                }
            //                else{
            //                Button_Modified_Drawing_Plane[3] -= 0.035;
            //                if (Button_Modified_Drawing_Plane[3]<Robot_Drawing_Plane[3]){
            //                    Button_Modified_Drawing_Plane[3] = Robot_Drawing_Plane[3] + User_Button_Modification;
            //                }
            //                vlimit = Draw_Vlimit;
            //                std::cout << "On plane" << std::endl;
            //                }
            //            }

            //            if (Last_Output_Exist == false){
            //                Button_Modified_Drawing_Plane[3] +=0.1;
            //            }
            //            robot_target_point = Find_Target_Point_X_Y(percentage_x, percentage_y, Button_Modified_Drawing_Plane);

            //            input[12] = robot_target_point[0];
            //            input[13] = robot_target_point[1];
            //            input[14] = robot_target_point[2];
            //            if (button_fixed_countdown > 0){
            //                std::cout << button_fixed_countdown << " " << User_Button_Modification << " " << input[12] << " " << input[13] << " " << input[14] << std::endl;
            //            }

            //            if (Last_Output_Exist){
            //                double Difference[3], Distance, Modified_Distance, Ratio;
            //                Difference[0] = input[12] - Last_Output[0];
            //                Difference[1] = input[13] - Last_Output[1];
            //                Difference[2] = input[14] - Last_Output[2];
            //                Distance = sqrt((Difference[0] * Difference[0]) + (Difference[1] * Difference[1]) + (Difference[2] * Difference[2]));
            //                if (Distance >= vlimit){
            //                    Ratio = vlimit / Distance;
            //                    input[12] = Ratio * Difference[0] + Last_Output[0];
            //                    input[13] = Ratio * Difference[1] + Last_Output[1];
            //                    input[14] = Ratio * Difference[2] + Last_Output[2];
            //                }
            //            }

            //            Last_Output_Exist = true;
            //            Last_Output[0] = input[12];
            //            Last_Output[1] = input[13];
            //            Last_Output[2] = input[14];
            // udpSender->send(input);

            // std::string message((const char*)buffer, size);
            // SendAsync(second_receiver_endpoint, message);
        }
        // std::vector<double> input(7);
        // char separater[] = " :,\t\r";
        // char *token;
        // char MotionData_Temp[100];
        // Eigen::Vector3d robot_target_point;
        // memset(MotionData_Temp, 0, sizeof(MotionData_Temp));
        // memcpy(MotionData_Temp, (const char*)buffer, size);
        // token = strtok(MotionData_Temp, separater);
        // for (int i=0; i<6; i++){
        //         input[i] = atof(token);
        //         token = strtok(NULL, separater);
        // }
        // input[6] = atof(token);
        // udpSender->send(input);
        //
        // Echo the message back to the sender
        // SendAsync(endpoint, message);

        if (packet_type == 2)
        {
            // parser_test.parse_motion_package((const char*)buffer, int(size));
            // data_test.recalculate_joints_ltm();

            // if (!calibration_mode)
            // {
            //     // For direct pose control
            //     //                 std::vector<double> input(16);

            //     //                input[0]  = Robot_Calibration_Point_1[0];
            //     //                input[1]  = Robot_Calibration_Point_1[1];
            //     //                input[2]  = Robot_Calibration_Point_1[2];
            //     //                input[3]  = Robot_Calibration_Point_1[3];

            //     //                input[4]  = Robot_Calibration_Point_1[4];
            //     //                input[5]  = Robot_Calibration_Point_1[5];
            //     //                input[6]  = Robot_Calibration_Point_1[6];
            //     //                input[7]  = Robot_Calibration_Point_1[7];

            //     //                input[8]  = Robot_Calibration_Point_1[8];
            //     //                input[9]  = Robot_Calibration_Point_1[9];
            //     //                input[10] = Robot_Calibration_Point_1[10];
            //     //                input[11] = Robot_Calibration_Point_1[11];

            //     //                input[12] = data_test.joints()[target_joing_index]->pos(0)[0];
            //     //                input[13] = data_test.joints()[target_joing_index]->pos(0)[2];
            //     //                input[14] = data_test.joints()[target_joing_index]->pos(0)[1];

            //     //                std::cout << "Joint Global Pos: " << input[12] << " " << input[13] << " " << input[14] << std::endl;
            //     //                input[15] = 1;

            //     //                // Test the button function
            //     //                if ((parser_test.button==0) || (fixed_button_duration > 0)){
            //     //                    Button_Modified_Drawing_Plane[3] += 0.002;
            //     //                    if (Button_Modified_Drawing_Plane[3]>(Robot_Drawing_Plane[3] + 0.05)){
            //     //                        Button_Modified_Drawing_Plane[3] = Robot_Drawing_Plane[3] + 0.05;
            //     //                    }
            //     //                    button_count = 0;
            //     //                    fixed_button_duration --;
            //     //                } else {
            //     //                    button_count ++;
            //     //                    Button_Modified_Drawing_Plane[3] -= 0.002;
            //     //                    if (Button_Modified_Drawing_Plane[3]< Robot_Drawing_Plane[3]){
            //     //                        Button_Modified_Drawing_Plane[3] = Robot_Drawing_Plane[3];
            //     //                    }
            //     //                    if (button_count >= 240){
            //     //                        Current_Origin_Movement[0] = input[12] - MoCap_Calibration_Point_1[12];
            //     //                        Current_Origin_Movement[1] = input[13] - MoCap_Calibration_Point_1[13];
            //     //                        Current_Origin_Movement[2] = input[14] - MoCap_Calibration_Point_1[14];
            //     //                        button_count = 0;
            //     //                        fixed_button_duration = 60;
            //     //                    }
            //     //                }

            //     //                Eigen::Vector3d point_projected_mocap, robot_target_point;
            //     //                point_projected_mocap = Find_Projected_Point(Eigen::Vector3d(input[12]-Current_Origin_Movement[0],input[13]-Current_Origin_Movement[1],input[14]-Current_Origin_Movement[2]), MoCap_Drawing_Plane);
            //     //                robot_target_point = Find_Target_Point(point_projected_mocap, MoCap_Drawing_Plane, Button_Modified_Drawing_Plane);

            //     //                input[12] = robot_target_point[0];
            //     //                input[13] = robot_target_point[1];
            //     //                input[14] = robot_target_point[2];

            //     //                if (Last_Output_Exist){
            //     //                    double Difference[3], Distance, Modified_Distance, Ratio;
            //     //                    Difference[0] = input[12] - Last_Output[0];
            //     //                    Difference[1] = input[13] - Last_Output[1];
            //     //                    Difference[2] = input[14] - Last_Output[2];
            //     //                    Distance = sqrt((Difference[0] * Difference[0]) + (Difference[1] * Difference[1]) + (Difference[2] * Difference[2]));
            //     //                    if (Distance >= 0.005){
            //     //                        Ratio = 0.005/Distance;
            //     //                        input[12] = Ratio * Difference[0] + Last_Output[0];
            //     //                        input[13] = Ratio * Difference[1] + Last_Output[1];
            //     //                        input[14] = Ratio * Difference[2] + Last_Output[2];
            //     //                    }
            //     //                }

            //     //                Last_Output_Exist = true;
            //     //                Last_Output[0] = input[12];
            //     //                Last_Output[1] = input[13];
            //     //                Last_Output[2] = input[14];

            //     if (output_data.mutex.try_lock())
            //     {
            //         output_data.has_mocap_data = true;
            //         output_data.mocap_data[0] = data_test.joints()[target_joing_index]->pos(0)[0];
            //         output_data.mocap_data[1] = data_test.joints()[target_joing_index]->pos(0)[2];
            //         output_data.mocap_data[2] = data_test.joints()[target_joing_index]->pos(0)[1];
            //         output_data.mutex.unlock();
            //     }
            //     // std::cout << "Output Cart_Pos: " << input[12] << " " << input[13] << " " << input[14] << std::endl;

            //     // udpSender->send(input);
            // }
            // else
            // {
            //     if (parser_test.button == 1)
            //     {
            //         button_count++;
            //         if (current_calibration_point_output_finished == false)
            //         {
            //             if (button_count == 100)
            //             {
            //                 if (current_calibration_point_no <= 4)
            //                 {
            //                     std::ofstream out;
            //                     out.open(relative_path + "Config/MoCap_Calibration_Point_" + std::to_string(current_calibration_point_no) + ".txt", std::ios::out);
            //                     out << data_test.joints()[target_joing_index]->ltm(0)[1].x << " " << data_test.joints()[target_joing_index]->ltm(0)[0].y << " " << data_test.joints()[target_joing_index]->ltm(0)[0].z << " " << data_test.joints()[target_joing_index]->ltm(0)[0].w
            //                         << " " << data_test.joints()[target_joing_index]->ltm(0)[1].x << " " << data_test.joints()[target_joing_index]->ltm(0)[1].y << " " << data_test.joints()[target_joing_index]->ltm(0)[1].z << " " << data_test.joints()[target_joing_index]->ltm(0)[1].w
            //                         << " " << data_test.joints()[target_joing_index]->ltm(0)[2].x << " " << data_test.joints()[target_joing_index]->ltm(0)[2].y << " " << data_test.joints()[target_joing_index]->ltm(0)[2].z << " " << data_test.joints()[target_joing_index]->ltm(0)[2].w
            //                         << " " << data_test.joints()[target_joing_index]->pos(0)[0] << " " << data_test.joints()[target_joing_index]->pos(0)[2] << " " << data_test.joints()[target_joing_index]->pos(0)[1] << " " << data_test.joints()[target_joing_index]->ltm(0)[3].w
            //                         << " " << std::endl;
            //                     out.close();
            //                     std::cout << "The " << current_calibration_point_no << " MoCap Calibration Point Created" << std::endl;
            //                     // std::cout << "Current Position Read by LTM " << temp_right_hand_joint->ltm(0,3) << " " << temp_right_hand_joint->ltm(1,3) << " " << temp_right_hand_joint->ltm(2,3) << std::endl;
            //                     // std::cout << "Current Position Read by Position " << temp_right_hand_joint->global_pos[0] << " " << temp_right_hand_joint->global_pos[1] << " " << temp_right_hand_joint->global_pos[2] << std::endl;
            //                     current_calibration_point_output_finished = true;
            //                 }
            //             }
            //         }
            //     }
            //     else
            //     {
            //         if (current_calibration_point_output_finished)
            //         {
            //             current_calibration_point_no++;
            //             current_calibration_point_output_finished = false;
            //             if (current_calibration_point_no > 4)
            //             {
            //                 std::cout << "MoCap Calibration Phase Finished" << std::endl;
            //             }
            //         }
            //         button_count = 0;
            //     }
            // }
        }

        ReceiveAsync();
    }

    void onSent(const asio::ip::udp::endpoint &endpoint, size_t sent) override
    {
        // Continue receive datagrams
        ReceiveAsync();
    }

    void onError(int error, const std::string &category, const std::string &message) override
    {
        std::cout << "Echo UDP server caught an error with code " << error << " and category '" << category << "': " << message << std::endl;
    }
};
#endif

class TabletUDPServer : public CppServer::Asio::UDPServer
{
public:
    using CppServer::Asio::UDPServer::UDPServer;

protected:
    void onStarted() override
    {
        if (output_data.mutex.try_lock())
        {
            output_data.has_mocap_data = false;
            output_data.has_tablet_data = false;
            output_data.on_paper = false;
            output_data.tablet_data.resize(5, 0.0);
            output_data.mocap_data.resize(3, 0.0);
            output_data.mutex.unlock();
        }

        std::cout << "Tablet UDP input server listening" << std::endl;
        ReceiveAsync();
    }

    void onReceived(const asio::ip::udp::endpoint &endpoint, const void *buffer, size_t size) override
    {
        std::string message((const char *)buffer, size);
        for (char &c : message)
        {
            if (c == ',' || c == ':' || c == '\t' || c == '\r' || c == '\n')
                c = ' ';
        }

        std::stringstream ss(message);
        std::string first;
        ss >> first;

        double percentage_x = 0.0;
        double percentage_y = 0.0;
        double pressure = 0.0;
        double altitude = 1.57079632679;
        double azimuth = 0.0;
        int button = 0;
        bool valid = false;

        if (first == "Tablet" || first == "Wacom")
        {
            ss >> percentage_x >> percentage_y >> pressure;
            if (ss.fail())
            {
                ReceiveAsync();
                return;
            }
            if (!(ss >> altitude))
                altitude = 1.57079632679;
            if (!(ss >> azimuth))
                azimuth = 0.0;
            if (!(ss >> button))
                button = pressure > 0.02 ? 1 : 0;
            valid = true;
        }
        else if (first == "Stylus")
        {
            double raw_x = 0.0;
            double raw_y = 0.0;
            ss >> raw_x >> raw_y >> pressure;
            if (ss.fail())
            {
                ReceiveAsync();
                return;
            }
            if (!(ss >> altitude))
                altitude = 1.57079632679;
            if (!(ss >> azimuth))
                azimuth = 0.0;
            percentage_x = raw_x / 955.0;
            percentage_y = raw_y / 954.0;
            button = pressure > 0.02 ? 1 : 0;
            valid = true;
        }
        else
        {
            try
            {
                percentage_x = std::stod(first);
            }
            catch (const std::exception &)
            {
                ReceiveAsync();
                return;
            }

            ss >> percentage_y;
            if (!(ss >> button))
                button = 0;
            if (!(ss >> pressure))
                pressure = button ? 1.0 : 0.0;
            if (!(ss >> altitude))
                altitude = 1.57079632679;
            if (!(ss >> azimuth))
                azimuth = 0.0;
            valid = !ss.fail() || message.size() > 0;
        }

        if (valid && output_data.mutex.try_lock())
        {
            output_data.tablet_data.resize(5, 0.0);
            output_data.has_tablet_data = true;
            output_data.tablet_data[0] = std::clamp(percentage_x, 0.0, 1.0);
            output_data.tablet_data[1] = std::clamp(percentage_y, 0.0, 1.0);
            output_data.tablet_data[2] = std::clamp(pressure, 0.0, 1.0);
            output_data.tablet_data[3] = altitude;
            output_data.tablet_data[4] = azimuth;
            output_data.on_paper = button != 0;
            output_data.mutex.unlock();
        }

        ReceiveAsync();
    }

    void onError(int error, const std::string &category, const std::string &message) override
    {
        std::cout << "Tablet UDP server caught an error with code " << error << " and category '" << category << "': " << message << std::endl;
    }
};
