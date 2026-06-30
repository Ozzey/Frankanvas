#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <MiosEventServer.hpp>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <utils.h>

using namespace std;

void readArguments(const int &argc, char **argv, string &inputSource, string &telepresenceMode, string &developerIP,
                   string &controllerIP, int &telepresencePortSend, int &telepresencePortReceive, string &endpoint) {
    inputSource = "keyboardVelocity";
    telepresenceMode = "velocity";
    developerIP = DEFAULT_DEVELOPER_IP;
    controllerIP = DEFAULT_CONTROLLER_IP;
    telepresencePortSend = DEFAULT_TELEPRESENCE_PORT_SEND;
    telepresencePortReceive = DEFAULT_TELEPRESENCE_PORT_RECEIVE;
    endpoint = DEFAULT_ENDPOINT;
    for (int i = 1; i < argc; i++) {
        switch (i) {
            case 1: {
                inputSource = argv[i];
                break;
            }
            case 2: {
                telepresenceMode = argv[i];
                break;
            }
            case 3: {
                developerIP = argv[i];
                break;
            }
            case 4: {
                controllerIP = argv[i];
                break;
            }
            case 5: {
                telepresencePortSend = stoi(argv[i]);
                break;
            }
            case 6: {
                telepresencePortReceive = stoi(argv[i]);
                break;
            }
            case 7: {
                endpoint = argv[i];
                break;
            }
            default: {
                break;
            }
        }
    }
}

bool startTestCommunication(const string &controllerIP, const string &endpoint) {
    auto client = msrm_utils::JsonWebsocketClient(controllerIP, MIOS_PORT, endpoint);
    auto request = nlohmann::json(), response = nlohmann::json();
    double timeout = 100;
    client.call_method(controllerIP, MIOS_PORT, endpoint, "lock_brakes", request, response, timeout);
    this_thread::sleep_for(chrono::seconds(10));
    client.call_method(controllerIP, MIOS_PORT, endpoint, "unlock_brakes", request, response, timeout);
    this_thread::sleep_for(chrono::seconds(2));
    return true;
}

bool startMoveToJointPoseTask(const string &controllerIP, const string &endpoint, const string &initial_joint) {
    auto client = msrm_utils::JsonWebsocketClient(controllerIP, runtime_mios_port, endpoint);
    auto request = nlohmann::json(), response = nlohmann::json();
    // string parameters = R"({"parameters": {"q_g": [0.0, 0.0, 0.0, 0.0, 0.0, 1.57079, 0.1]}})";
    string parameters = R"({"parameters": {"q_g": )"+ initial_joint + R"(, "pose": "NoneObject"}})";
    //cout << "parameter:" << parameters << endl;
    bool ok = startTask(client, "MoveToJointPose", parameters, false, controllerIP, endpoint, response, 10);
    cout << "MoveToJointPose response: " << response.dump() << endl;
    if (!ok || response["result"]["result"] != true) {
        cerr << "Failed to start MoveToJointPose on " << controllerIP << ":" << runtime_mios_port << endl;
        return false;
    }
    this_thread::sleep_for(chrono::seconds(5));
    return true;
}

bool startMoveToJointPoseTask_Camera(const string &controllerIP, const string &endpoint, const string &initial_joint) {
    auto client = msrm_utils::JsonWebsocketClient(controllerIP, runtime_mios_camera_port, endpoint);
    auto request = nlohmann::json(), response = nlohmann::json();
    // string parameters = R"({"parameters": {"q_g": [0.0, 0.0, 0.0, 0.0, 0.0, 1.57079, 0.1]}})";
    string parameters = R"({"parameters": {"q_g": )"+ initial_joint + R"(, "pose": "NoneObject"}})";
    //cout << "parameter:" << parameters << endl;
    bool ok = startTask_Camera(client, "MoveToJointPose", parameters, false, controllerIP, endpoint, response, 10);
    cout << "Camera MoveToJointPose response: " << response.dump() << endl;
    if (!ok || response["result"]["result"] != true) {
        cerr << "Failed to start camera MoveToJointPose on " << controllerIP << ":" << runtime_mios_camera_port << endl;
        return false;
    }
    this_thread::sleep_for(chrono::seconds(5));
    return true;
}


void setTelepresenceSkillParameters(string &skillParameters, string &controlParameters, TelepresenceMode &telepresence,
                                    const string &telepresenceModeString, const string &developerIP,
                                    const int &telepresencePortSend, const int &telepresencePortReceive) {
    /*
    string skillParametersDC = R"({"is_master": false, "developerIP": ")" + developerIP + R"(", "port_dst": )" +
                               to_string(telepresencePortSend) + ", \"port_src\": " + to_string(telepresencePortReceive) +
                               R"(, "telepresence_mode": "DirectCart", "direct_cart": {"alpha": [1, 1, 1, 1, 1, 1]}})";
    //*/
    if (telepresenceModeString == "velocity") {
        skillParameters = R"({"is_master": false, "ip_dst": ")" + developerIP + R"(", "port_dst": )" +
                          to_string(telepresencePortReceive) + ", \"port_src\": " + to_string(telepresencePortSend) +
                          R"(, "telepresence_mode": "Joystick", "joystick": {"static_frame": true}})";
        controlParameters = "{\"control_mode\": 0}";
        telepresence = TelepresenceMode::Joystick;
    } else if (telepresenceModeString == "direct_cartesian" || telepresenceModeString == "directCartesian") {
        skillParameters = R"({"is_master": false, "ip_dst": ")" + developerIP + R"(", "port_dst": )" +
                          to_string(telepresencePortReceive) + ", \"port_src\": " + to_string(telepresencePortSend) +
			  				     + ", \"terminate_when_loc\": true"	+ ", \"remote_event_port\":16666"
                          R"(, "telepresence_mode": "DirectCart", "direct_cart": {"alpha": [0, 0, 0, 0, 0, 0]}})";
        controlParameters = "{\"control_mode\": 0}";
        telepresence = TelepresenceMode::DirectCart;
    } else if (telepresenceModeString == "direct_joint" || telepresenceModeString == "directJoint") {
        skillParameters = R"({"is_master": false, "ip_dst": ")" + developerIP + R"(", "port_dst": )" +
                          to_string(telepresencePortReceive) + ", \"port_src\": " + to_string(telepresencePortSend) +
                          R"(, "telepresence_mode": "DirectJoint", "direct_joint": {"alpha": [0, 0, 0, 0, 0, 0]}})";
        controlParameters = "{\"control_mode\": 1}";
        telepresence = TelepresenceMode::DirectJoint;
    } else if (telepresenceModeString == "direct_cartesian_ff"){
        skillParameters = R"({"is_master": false, "ip_dst": ")" + developerIP + R"(", "port_dst": )" +
                          to_string(telepresencePortReceive) + ", \"port_src\": " + to_string(telepresencePortSend) +
                            + ", \"terminate_when_loc\": true"	+ ", \"remote_event_port\":16666"
                          R"(, "telepresence_mode": "DirectCart", "direct_cart": {"alpha": [0, 0, 0, 0, 0, 0], "plane": true, "F_ff": [0, 0, 10, 0, 0, 0]}})";
        controlParameters = "{\"control_mode\": 0}";
        telepresence = TelepresenceMode::DirectCart;
    }
    else {
        throw invalid_argument("Unknown telepresence mode " + telepresenceModeString);
    }
}

bool startTelepresenceSkill(const string &telepresenceModeString, const string &developerIP, const string &controllerIP,
                            const string &endpoint, const int &telepresencePortSend,
                            const int &telepresencePortReceive, TelepresenceMode &telepresence) {
    auto client = msrm_utils::JsonWebsocketClient(controllerIP, runtime_mios_port, endpoint);
    auto request = nlohmann::json(), response = nlohmann::json();
    string skillParameters, controlParameters;
    setTelepresenceSkillParameters(skillParameters, controlParameters, telepresence, telepresenceModeString,
                                   developerIP, telepresencePortSend, telepresencePortReceive);

    cout << "Pre calling startSkill" << endl;
    bool ok = startSkill(client, "Telepresence", skillParameters, controlParameters, false, controllerIP, endpoint, response, 10);
    cout << "Skill initialized!" << endl;

    if (!ok || response["result"]["result"] != true) {
        cout << response.dump() << endl;
        cerr << "Failed to start Telepresence skill on " << controllerIP << ":" << runtime_mios_port << endl;
        return false;
    }
    return true;
}


void setTelepresenceSkillParameters_Camera(string &skillParameters, string &controlParameters, TelepresenceMode &telepresence,
                                    const string &telepresenceModeString, const string &developerIP,
                                    const int &telepresencePortSend, const int &telepresencePortReceive) {
    /*
    string skillParametersDC = R"({"is_master": false, "developerIP": ")" + developerIP + R"(", "port_dst": )" +
                               to_string(telepresencePortSend) + ", \"port_src\": " + to_string(telepresencePortReceive) +
                               R"(, "telepresence_mode": "DirectCart", "direct_cart": {"alpha": [1, 1, 1, 1, 1, 1]}})";
    //*/
    if (telepresenceModeString == "velocity") {
        skillParameters = R"({"is_master": false, "ip_dst": ")" + developerIP + R"(", "port_dst": )" +
                          to_string(telepresencePortReceive) + ", \"port_src\": " + to_string(telepresencePortSend) +
                          R"(, "telepresence_mode": "Joystick", "joystick": {"static_frame": true}})";
        controlParameters = "{\"control_mode\": 0}";
        telepresence = TelepresenceMode::Joystick;
    } else if (telepresenceModeString == "direct_cartesian" || telepresenceModeString == "directCartesian") {
        skillParameters = R"({"is_master": false, "ip_dst": ")" + developerIP + R"(", "port_dst": )" +
                          to_string(telepresencePortReceive) + ", \"port_src\": " + to_string(telepresencePortSend) +
			  				     + ", \"terminate_when_loc\": true"	+ ", \"remote_event_port\":17666"
                          R"(, "telepresence_mode": "DirectCart", "direct_cart": {"alpha": [0, 0, 0, 0, 0, 0]}})";
        controlParameters = "{\"control_mode\": 0}";
        telepresence = TelepresenceMode::DirectCart;
    } else if (telepresenceModeString == "direct_joint" || telepresenceModeString == "directJoint") {
        skillParameters = R"({"is_master": false, "ip_dst": ")" + developerIP + R"(", "port_dst": )" +
                          to_string(telepresencePortReceive) + ", \"port_src\": " + to_string(telepresencePortSend) +
                          R"(, "telepresence_mode": "DirectJoint", "direct_joint": {"alpha": [0, 0, 0, 0, 0, 0]}})";
        controlParameters = "{\"control_mode\": 1}";
        telepresence = TelepresenceMode::DirectJoint;
    } else if (telepresenceModeString == "direct_cartesian_ff"){
        skillParameters = R"({"is_master": false, "ip_dst": ")" + developerIP + R"(", "port_dst": )" +
                          to_string(telepresencePortReceive) + ", \"port_src\": " + to_string(telepresencePortSend) +
                            + ", \"terminate_when_loc\": true"	+ ", \"remote_event_port\":17666"
                          R"(, "telepresence_mode": "DirectCart", "direct_cart": {"alpha": [0, 0, 0, 0, 0, 0], "plane": true, "F_ff": [0, 0, 10, 0, 0, 0]}})";
        controlParameters = "{\"control_mode\": 0}";
        telepresence = TelepresenceMode::DirectCart;
    }
    else {
        throw invalid_argument("Unknown telepresence mode " + telepresenceModeString);
    }
}

bool startTelepresenceSkill_Camera(const string &telepresenceModeString, const string &developerIP, const string &controllerIP,
                            const string &endpoint, const int &telepresencePortSend,
                            const int &telepresencePortReceive, TelepresenceMode &telepresence) {
    auto client = msrm_utils::JsonWebsocketClient(controllerIP, runtime_mios_port, endpoint);
    auto request = nlohmann::json(), response = nlohmann::json();
    string skillParameters, controlParameters;
    setTelepresenceSkillParameters_Camera(skillParameters, controlParameters, telepresence, telepresenceModeString,
                                   developerIP, telepresencePortSend, telepresencePortReceive);

    cout << "Pre calling startSkill" << endl;
    bool ok = startSkill_Camera(client, "Telepresence", skillParameters, controlParameters, false, controllerIP, endpoint, response, 10);
    cout << "Skill initialized!" << endl;

    if (!ok || response["result"]["result"] != true) {
        cout << response.dump() << endl;
        cerr << "Failed to start camera Telepresence skill on " << controllerIP << ":" << runtime_mios_port << endl;
        return false;
    }
    return true;
}


//bool initializeMIOSServerAndPanda(MiosEventServer *&miosEventServer, MiosPanda *&miosPanda, const string &controllerIP,
//                                  const string &endpoint, const TelepresenceMode &telepresence,
//                                  const int &telepresencePortSend, const int &telepresencePortReceive) {
//    miosEventServer = new MiosEventServer(true, 100);
//    cout << "Created miosEventServer!" << endl;
//    miosPanda = new MiosPanda(1, false);
//    cout << "Created miosPandaClient!" << endl;

//    // connect to mios via websocket and take handshake stages 0 and 1, finishing with stage 2
//    if (!miosEventServer->InitTelepresence(controllerIP, MIOS_PORT, LOCAL_WS_SERVER_PORT, endpoint, telepresence,
//                                           DEFAULT_DEVELOPER_UDP_RECEIVE_DATA_PORT)) {
//        cerr << "Failed to connect to mios with address " << controllerIP << endl;
//        return false;
//    }

//    // setup UDP stream to and from mios (handshake stage 2)
//    if (!miosPanda->StartTelepresence(controllerIP, telepresencePortSend, telepresencePortReceive)) {
//        cerr << "Failed to start telepresence" << endl;
//        return false;
//    }
//    return true;
//}

//bool initialize(DataInput *&dataInput, MiosEventServer *&miosEventServer, MiosPanda *&miosPanda,
//                TelepresenceMode &telepresence, const string &inputSource, const string &telepresenceModeString,
//                const string &developerIP, const string &controllerIP, int telepresencePortSend,
//                int telepresencePortReceive, const string &endpoint) {
//    /*
//    if (!startTestCommunication(controllerIP, endpoint)){
//        return false;
//    }

//    if (!startMoveToJointPoseTask(controllerIP, endpoint)) {
//        return false;
//    }
//    //*/
//    if (!startTelepresenceSkill(telepresenceModeString, developerIP, controllerIP, endpoint, telepresencePortSend,
//                                telepresencePortReceive, telepresence)) {
//        return false;
//    }

//    if (!initializeMIOSServerAndPanda(miosEventServer, miosPanda, controllerIP, endpoint, telepresence,
//                                        telepresencePortSend, telepresencePortReceive)){
//        return false;
//    }

//    dataInput = convertInputSourceToDataInput(inputSource);
//    cout << "Created input source" << endl;
//    return true;
//}

//int main(int argc, char **argv) {
//    string inputSource, developerIP, controllerIP, endpoint, telepresenceMode;
//    int32_t telepresencePortSend, telepresencePortReceive;
//    readArguments(argc, argv, inputSource, telepresenceMode, developerIP, controllerIP, telepresencePortSend,
//                  telepresencePortReceive, endpoint);

//    try {
//        DataInput *dataInput;
//        MiosEventServer *miosEventServer;
//        MiosPanda *miosPanda;
//        TelepresenceMode telepresence;
//        if (!initialize(dataInput, miosEventServer, miosPanda, telepresence, inputSource, telepresenceMode, developerIP,
//                        controllerIP, telepresencePortSend, telepresencePortReceive, endpoint)) {
//            return 1;
//        }

//        vector<double> input;
//        while (true) {
//            dataInput->getLastInput(input, false);

//            if (!miosPanda->SendInputToMios(input, telepresence, print)) {
//                cout << "error..." << endl;
//                break;
//            }

//            // Receive feedback from mios, transform and send to host
//            // converter->setData(miosPandaClient->LatestMiosFeedback());

//            this_thread::sleep_for(chrono::milliseconds(1));
//        }
//        delete dataInput;
//        delete miosEventServer;
//        delete miosPanda;
//    } catch (exception &e) {
//        cerr << "Exception: " << e.what() << "\n";
//    }
//}
