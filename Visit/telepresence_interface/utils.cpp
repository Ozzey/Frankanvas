#include <iostream>
#include <memory>
#include <thread>
#include <utils.h>

using namespace Eigen;
using namespace msrm_utils;
using namespace std;

const string DEFAULT_DEVELOPER_IP = "10.162.15.236";
const string DEFAULT_CONTROLLER_IP = "10.162.15.46";
const string DEFAULT_ENDPOINT = "mios/core";
unsigned runtime_mios_port = MIOS_PORT;
unsigned runtime_mios_camera_port = MIOS_CAMERA_PORT;
const int DEFAULT_TELEPRESENCE_PORT_SEND = 13333;
const int DEFAULT_TELEPRESENCE_PORT_RECEIVE = 13334;
const int DEFAULT_DEVELOPER_UDP_RECEIVE_DATA_PORT = 10002;
const int PHONE_CONTROL_DEVELOPER_UDP_RECEIVE_DATA_PORT = 10003;

string boolToString(bool x) {
    return (x ? "true" : "false");
}

bool startTask(JsonWebsocketClient &client, const string &task, const string &parameters, bool addToQueue,
               const string &mios_id, const string &endpoint, nlohmann::json &response, const double &timeout) {
    cout << "IN START TASK!" << endl;
    string payloadString =
            R"({"task": ")" + task + R"(", "parameters": )" + parameters + ", \"queue\": " + boolToString(addToQueue) +
            "}";
    cout << "After constructing payload string..." << endl;
    auto payload = nlohmann::json::parse(payloadString);
    cout << "After converting payload string to json..." << endl;
    cout << "payload = " << payload.dump() << endl;
    cout << "After dumping payload to stdout..." << endl;
    return client.call_method(mios_id, runtime_mios_port, endpoint, "start_task", payload, response, timeout);
    cout << "Calling the 'call_method' method..." << endl;
}

bool startTask_Camera(JsonWebsocketClient &client, const string &task, const string &parameters, bool addToQueue,
               const string &mios_id, const string &endpoint, nlohmann::json &response, const double &timeout) {
    cout << "IN START TASK!" << endl;
    string payloadString =
            R"({"task": ")" + task + R"(", "parameters": )" + parameters + ", \"queue\": " + boolToString(addToQueue) +
            "}";
    cout << "After constructing payload string..." << endl;
    auto payload = nlohmann::json::parse(payloadString);
    cout << "After converting payload string to json..." << endl;
    cout << "payload = " << payload.dump() << endl;
    cout << "After dumping payload to stdout..." << endl;
    return client.call_method(mios_id, runtime_mios_camera_port, endpoint, "start_task", payload, response, timeout);
    cout << "Calling the 'call_method' method..." << endl;
}

bool startSkill(JsonWebsocketClient &client, const string &skill, const string &skillParameters,
                const string &controlParameters, bool addToQueue, const string &mios_id, const string &endpoint,
                nlohmann::json &response, const double &timeout) {
    cout << "In startSkill function!" << endl;

    auto firstParam = R"("parameters": {"skill_names": ["skill"], "skill_types": [")" + skill +
                      R"("]})";
    cout << "firstParam = " << firstParam << endl;
    cout << nlohmann::json::parse("{" + firstParam + "}").dump() << endl << endl;

    auto secondParam = R"("skills": {"skill": {"skill": )" + skillParameters + R"(, "control": )" +
                       controlParameters + "}}";
    cout << "secondParam = " << secondParam << endl;
    cout << nlohmann::json::parse("{" + secondParam + "}").dump() << endl << endl;

    string parametersString = "{" + firstParam + ", " + secondParam + "}";
    cout << "parameterString = " << parametersString << endl;
    cout << nlohmann::json::parse(parametersString).dump() << endl << endl;

    return startTask(client, "GenericTask", parametersString, addToQueue, mios_id, endpoint, response, timeout);
}

bool startSkill_Camera(JsonWebsocketClient &client, const string &skill, const string &skillParameters,
                const string &controlParameters, bool addToQueue, const string &mios_id, const string &endpoint,
                nlohmann::json &response, const double &timeout) {
    cout << "In startSkill function!" << endl;

    auto firstParam = R"("parameters": {"skill_names": ["skill"], "skill_types": [")" + skill +
                      R"("]})";
    cout << "firstParam = " << firstParam << endl;
    cout << nlohmann::json::parse("{" + firstParam + "}").dump() << endl << endl;

    auto secondParam = R"("skills": {"skill": {"skill": )" + skillParameters + R"(, "control": )" +
                       controlParameters + "}}";
    cout << "secondParam = " << secondParam << endl;
    cout << nlohmann::json::parse("{" + secondParam + "}").dump() << endl << endl;

    string parametersString = "{" + firstParam + ", " + secondParam + "}";
    cout << "parameterString = " << parametersString << endl;
    cout << nlohmann::json::parse(parametersString).dump() << endl << endl;

    return startTask_Camera(client, "GenericTask", parametersString, addToQueue, mios_id, endpoint, response, timeout);
}


//void setPositionInput(vector<double> &input, const vector<pair<string, double>> &keys) {
//    for (const auto &keyPair : keys) {
//        if (keyPair.first == "linX") {
//            input[12] += keyPair.second;
//        } else if (keyPair.first == "linY") {
//            input[13] += keyPair.second;
//        } else if (keyPair.first == "linZ") {
//            input[14] += keyPair.second;
//        }
//    }
//}

//void setVelocityInput(vector<double> &input, const vector<pair<string, double>> &keys) {
//    input[0] = 0;
//    input[1] = 0;
//    input[2] = 0;
//    input[3] = 0;
//    input[4] = 0;
//    input[5] = 0;
//    for (const auto &keyPair : keys) {
//        if (keyPair.first == "linX") {
//            input[0] = keyPair.second;
//        } else if (keyPair.first == "linY") {
//            input[1] = keyPair.second;
//        } else if (keyPair.first == "linZ") {
//            input[2] = keyPair.second;
//        } else if (keyPair.first == "rotX") {
//            input[3] = keyPair.second;
//        } else if (keyPair.first == "rotY") {
//            input[4] = keyPair.second;
//        } else if (keyPair.first == "rotZ") {
//            input[5] = keyPair.second;
//        }
//    }
//}

//void setAccelerationInput(vector<double> &input, const vector<pair<string, double>> &keys) {
//    input[0] = 0;
//    input[1] = 0;
//    input[2] = 0;
//    Quaterniond o(input[3], input[4], input[5], input[6]);
//    o.normalize();
//    for (const auto &keyPair : keys) {
//        if (keyPair.first == "linX") {
//            input[0] = keyPair.second;
//        } else if (keyPair.first == "linY") {
//            input[1] = keyPair.second;
//        } else if (keyPair.first == "linZ") {
//            input[2] = keyPair.second;
//        } else if (keyPair.first == "rotX") {
//            o *= Quaterniond(cos(keyPair.second / 2), sin(keyPair.second / 2), 0, 0);
//        } else if (keyPair.first == "rotY") {
//            o *= Quaterniond(cos(keyPair.second / 2), 0, sin(keyPair.second / 2), 0);
//        } else if (keyPair.first == "rotZ") {
//            o *= Quaterniond(cos(keyPair.second / 2), 0, 0, sin(keyPair.second / 2));
//        }
//    }
//    input[3] = o.w();
//    input[4] = o.x();
//    input[5] = o.y();
//    input[6] = o.z();
//}

//bool checkVectorNonZero(const vector<double> &v) {
//    return any_of(v.begin(), v.end(), [](int val) { return val != 0.0; });
//}

//void printVectorIfNonZero(const vector<double> &v, bool printEvenIfZero) {
//    string output = "Output vector: ";
//    bool print = false;
//    for (double val : v) {
//        output += to_string(val) + ", ";
//        if (val != 0.0) {
//            print = true;
//        }
//    }
//    if (print || printEvenIfZero) {
//        cout << output << endl;
//    }
//}

//* void testSend() {
//    shared_ptr<UDPStreamSender> udpSender;
//    if (!initializeUDPSender(udpSender, "10.162.15.57", 8888)) {
//        return;
//    }
//    if (!udpSender->send({0, 1, 2, 3, 4, 5})) {
//        cerr << "Failed to send data!" << endl;
//    } else {
//        cout << "Sent data successfully!" << endl;
//    }
//} */

//void readMessage(const vector<double> &data) {
//    cout << "Received message!" << endl;
//    for (auto &i : data) {
//        cout << i << ", ";
//    }
//    cout << endl;
//}

//void testReceive() {
//    shared_ptr<UDPStreamReceiver> udpReceiver;
//    if (!initializeUDPListener(udpReceiver, 34524, bind(&readMessage, placeholders::_1))) {
//        return;
//    }

//    while (true) {
//        this_thread::sleep_for(chrono::milliseconds(10));
//    }
//}

bool initializeUDPListener(
        shared_ptr<UDPStreamReceiver> &udpReceiver, unsigned receivePort,
        const std::function<void(std::vector<double> &)> &callbackFunction, unsigned bufferSize,
        unsigned timeoutSeconds, unsigned timeoutMicroSeconds, unsigned maxLostPackets) {
    udpReceiver = make_shared<UDPStreamReceiver>(
            receivePort, bufferSize, timeoutSeconds, timeoutMicroSeconds, maxLostPackets, callbackFunction);
    if (udpReceiver == nullptr) {
        cerr << "Failed to create udp stream receiver" << endl;
        return false;
    }
    if (!udpReceiver->connect()) {
        cerr << "Failed to connect udp stream receiver" << endl;
        return false;
    }
    assert(udpReceiver->is_running());
    return true;
}

bool initializeUDPSender(shared_ptr<UDPStreamSender> &udpSender, const string &sendIP, unsigned sendPort) {
    udpSender = make_shared<UDPStreamSender>(sendIP, sendPort);
    if (udpSender == nullptr) {
        cerr << "Failed to open udp stream sender" << endl;
        return false;
    }
    if (!udpSender->connect()) {
        cerr << "Failed to connect udp stream sender" << endl;
        return false;
    }
    return true;
}

//Vector3d quaternionToRPY(const Quaterniond &o) {
//    // first rotate around X, then round Y and finally around Z => R = Rx * Ry * Rz
//    // x points left, y points front, z point up
//    Quaternion q = o;
//    Vector3d euler;

//    double sqw = q.w() * q.w(), sqx = q.x() * q.x(), sqy = q.y() * q.y(), sqz = q.z() * q.z();

//    // If quaternion is normalised the unit is one, otherwise it is the correction factor
//    double unit = sqx + sqy + sqz + sqw, test = q.x() * q.z() + q.y() * q.w();

//    if (test > 0.4999 * unit) {
//        // 0.4999f OR 0.5f - EPSILON: Singularity at north pole
//        euler.x() = 0;                                                                         // Roll
//        euler.y() = M_PI * 0.5f;                                                      // Pitch
//        euler.z() = atan2(2 * q.w() * q.z() + 2 * q.x() * q.y(), sqw - sqx + sqy - sqz);  // Yaw
//    } else if (test < -0.4999f * unit) {
//        // -0.4999f OR -0.5f + EPSILON: Singularity at south pole
//        euler.x() = 0;                                                                         // Roll
//        euler.y() = -M_PI * 0.5f;                                                     // Pitch
//        euler.z() = atan2(2 * q.w() * q.z() + 2 * q.x() * q.y(), sqw - sqx + sqy - sqz);  // Yaw
//    } else {
//        euler.x() = atan2(2 * q.x() * q.w() - 2 * q.y() * q.z(), sqw - sqx - sqy + sqz);  // Roll
//        euler.y() = asin(2 * test / unit);                                         // Pitch
//        euler.z() = atan2(2 * q.z() * q.w() - 2 * q.x() * q.y(), sqw + sqx - sqy - sqz);  // Yaw
//    }

//    // return (float)(180 / M_PI) * euler;  // convert to degrees
//    return euler;
//}
