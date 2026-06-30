//
// Created by Andrei on 03-Oct-20.
//

#ifndef MIOS_INTERFACE_UTILS_H
#define MIOS_INTERFACE_UTILS_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include "msrm_utils/network.hpp"

#define MIOS_PORT 12000
#define MIOS_CAMERA_PORT 13000
#define LOCAL_WS_SERVER_PORT 16666

extern unsigned runtime_mios_port;
extern unsigned runtime_mios_camera_port;
extern const std::string DEFAULT_DEVELOPER_IP;
extern const std::string DEFAULT_CONTROLLER_IP;
extern const std::string DEFAULT_ENDPOINT;
extern const int DEFAULT_TELEPRESENCE_PORT_SEND;
extern const int DEFAULT_TELEPRESENCE_PORT_RECEIVE;
extern const int DEFAULT_DEVELOPER_UDP_RECEIVE_DATA_PORT;
extern const int PHONE_CONTROL_DEVELOPER_UDP_RECEIVE_DATA_PORT;

enum TelepresenceMode {
    Joystick,
    DirectCart,
    DirectJoint,
    DirectCartFF
};

std::string boolToString(bool x);

bool startTask(msrm_utils::JsonWebsocketClient &client, const std::string &task, const std::string &parameters,
               bool addToQueue, const std::string &mios_id, const std::string &endpoint, nlohmann::json &response,
               const double &timeout = 100);

bool startTask_Camera(msrm_utils::JsonWebsocketClient &client, const std::string &task, const std::string &parameters,
               bool addToQueue, const std::string &mios_id, const std::string &endpoint, nlohmann::json &response,
               const double &timeout = 100);


bool startSkill(msrm_utils::JsonWebsocketClient &client, const std::string &skill, const std::string &skillParameters,
                const std::string &controlParameters, bool addToQueue, const std::string &mios_id,
                const std::string &endpoint, nlohmann::json &response, const double &timeout = 100);

bool startSkill_Camera(msrm_utils::JsonWebsocketClient &client, const std::string &skill, const std::string &skillParameters,
                const std::string &controlParameters, bool addToQueue, const std::string &mios_id,
                const std::string &endpoint, nlohmann::json &response, const double &timeout = 100);

//void setPositionInput(std::vector<double> &input, const std::vector<std::pair<std::string, double>> &keys);

//void setVelocityInput(std::vector<double> &input, const std::vector<std::pair<std::string, double>> &keys);

//void setAccelerationInput(std::vector<double> &input, const std::vector<std::pair<std::string, double>> &keys);

//bool checkVectorNonZero(const std::vector<double> &v);

//void printVectorIfNonZero(const std::vector<double> &v, bool printEvenIfZero = false);

//void testSend();

//void testReceive();

bool initializeUDPListener(
        std::shared_ptr<msrm_utils::UDPStreamReceiver> &udpReceiver, unsigned receivePort,
        const std::function<void(std::vector<double> &)> &callbackFunction, unsigned bufferSize = 256,
        unsigned timeoutSeconds = 0, unsigned timeoutMicroSeconds = 10000, unsigned maxLostPackets = 200);

bool initializeUDPSender(std::shared_ptr<msrm_utils::UDPStreamSender> &udpSender, const std::string &sendIP,
                         unsigned sendPort);

//Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond &o);

#endif //MIOS_INTERFACE_UTILS_H
