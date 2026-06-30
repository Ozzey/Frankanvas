#pragma once

#include <chrono>
#include <iostream>
#include <thread>
#include <utils.h>
#include <nlohmann/json.hpp>

class MiosEventServer {
public:
    MiosEventServer(bool is_leader, unsigned ms_loop_delay);

    // Initialize connection for communication with the mios-telepresence-skill
    bool InitTelepresence(const std::string &address, unsigned ws_port, unsigned ws_local_port, const std::string &endpoint,
                          const TelepresenceMode &telepresence, const int handshakePort);

    nlohmann::json WsCbPostEvent(const nlohmann::json &request);
    std::vector<double> handshakeData;
    void Stop();

protected:
    void getData(std::vector<double> &data);

    nlohmann::json createHandshakeRequest(const TelepresenceMode &telepresence, const int handshakePort);
    bool handshakeDataReceived;
    bool is_leader_;
    unsigned ms_loop_delay_;
    std::mutex input_mutex_{};
    std::unique_ptr<msrm_utils::IJsonMethodServer> json_ws_server_{};
    nlohmann::json request_{};
    nlohmann::json response_{};
    unsigned handshake_stage_{};
    bool mios_synced_{};
    bool running_;
};

bool startMoveToJointPoseTask(const std::string &controllerIP, const std::string &endpoint, const std::string &initial_joint);

bool startMoveToJointPoseTask_Camera(const std::string &controllerIP, const std::string &endpoint, const std::string &initial_joint);

bool startTelepresenceSkill(const std::string &telepresenceModeString, const std::string &developerIP, const std::string &controllerIP,
                            const std::string &endpoint, const int &telepresencePortSend,
                            const int &telepresencePortReceive, TelepresenceMode &telepresence);

bool startTelepresenceSkill_Camera(const std::string &telepresenceModeString, const std::string &developerIP, const std::string &controllerIP,
                            const std::string &endpoint, const int &telepresencePortSend,
                            const int &telepresencePortReceive, TelepresenceMode &telepresence);

void setTelepresenceSkillParameters(std::string &skillParameters, std::string &controlParameters, TelepresenceMode &telepresence,
                                    const std::string &telepresenceModeString, const std::string &developerIP,
                                    const int &telepresencePortSend, const int &telepresencePortReceive);

void setTelepresenceSkillParameters_Camera(std::string &skillParameters, std::string &controlParameters, TelepresenceMode &telepresence,
                                    const std::string &telepresenceModeString, const std::string &developerIP,
                                    const int &telepresencePortSend, const int &telepresencePortReceive);                                    