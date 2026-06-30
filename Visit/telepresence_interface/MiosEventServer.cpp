#include "MiosEventServer.hpp"
#include <functional>
#include <mutex>

using namespace msrm_utils;
using namespace std;

MiosEventServer::MiosEventServer(const bool is_leader, const unsigned ms_loop_delay)
        : handshakeData(), handshakeDataReceived(false) {
    cout << "MiosEventServer created" << endl;
    // "this" is the host side, meaning if "this" is the leader, the robot is the follower
    this->is_leader_ = is_leader;
    this->ms_loop_delay_ = ms_loop_delay;
    this->running_ = true;
}

void MiosEventServer::Stop() {
    this->running_ = false;
}

void MiosEventServer::getData(std::vector<double> &data) {
    this->handshakeData = data;
    //*
    cout << "HANDSHAKE POSE:" << endl;
    for (auto v : this->handshakeData) {
        cout << v << ", ";
    }
    cout << endl;
    //*/
    this->handshakeDataReceived = true;
}

nlohmann::json MiosEventServer::createHandshakeRequest(const TelepresenceMode &telepresence, const int handshakePort) {
    nlohmann::json request, requestContent;
    if (telepresence == TelepresenceMode::Joystick) {
        request["name"] = "handshake";
        request["content"] = requestContent;
    } else if (telepresence == TelepresenceMode::DirectCart) {
        cout << "Creating handshake request for DirectCart mode!" << endl;
        // shared_ptr<UDPStreamReceiver> udpReceiver;
        // if (!initializeUDPListener(udpReceiver, handshakePort,
        //                            bind(&MiosEventServer::getData, this, placeholders::_1))) {
        //     throw runtime_error("Can not initialize udp stream receiver needed for handshake!");
        // }
        // while (!this->handshakeDataReceived) {}
        // this->handshakeDataReceived = false;
        // udpReceiver->disconnect();
        // cout << "Created handshake request for DirectCart mode!" << endl;
        // cout << "HANDSHAKE POSE:" << endl;
        // for (auto v : this->handshakeData) {
        //     cout << v << ", ";
        // }
        // cout << endl;

        requestContent["O_T_EE_master"] = this->handshakeData;
        request["name"] = "handshake";
        request["content"] = requestContent;
        // request["O_T_EE_master"] = initialPose;
    } else if (telepresence == TelepresenceMode::DirectJoint) {
        shared_ptr<UDPStreamReceiver> udpReceiver;
        // initializeUDPListener(udpReceiver, handshakePort, bind(&MiosEventServer::getData, this, placeholders::_1));
        // while (!this->handshakeDataReceived) {}
        // this->handshakeDataReceived = false;
        // udpReceiver->disconnect();

        requestContent["q_master"] = this->handshakeData;
        request["name"] = "handshake";
        request["content"] = requestContent;
    } else {
        throw invalid_argument("Unknown telepresence mode " + to_string(telepresence));
    }
    return request;
}

bool MiosEventServer::InitTelepresence(const string &address, unsigned ws_port, unsigned ws_local_port, const string &endpoint,
                                       const TelepresenceMode &telepresence, const int handshakePort) {
    this->handshake_stage_ = 0;
    this->mios_synced_ = false;
    request_ = nullptr;
    nlohmann::json response;
    json_ws_server_ = make_unique<JsonWebsocketServer>("0.0.0.0", ws_local_port, endpoint);

    // initialization; not pert of handshake
    if (!json_ws_server_->start_listening()) {
        cerr << "Could not start json ws server" << endl;
    }
    if (!json_ws_server_->bind_method("post_event", bind(&MiosEventServer::WsCbPostEvent, this, placeholders::_1),
                                      {ArgPair("name", {}), ArgPair("content", nlohmann::json())})) {
        cerr << "Could not bind method post_event to ws_server" << endl;
    }
    cout << "After bind..." << endl;

    if (this->is_leader_) {
        cout << "MiosEventServer: is leader" << endl;
        nlohmann::json request = this->createHandshakeRequest(telepresence, handshakePort);
        while (this->running_) {
            if (this->handshake_stage_ == 0) {
                try {
                    cout << "Pre call_method stage 0..." << endl;
                    if (!JsonWebsocketClient::call_method(address, ws_port, "mios/core", "post_event", request,
                                                          response, 5)) {
                        response = false;
                    } else {
                        this->handshake_stage_ = 1;
                    }
                    cout << "Post call_method stage 0..." << endl;
                } catch (exception &e) {
                    cerr << "Exception: " << e.what() << "\n";
                }
            } else if (this->handshake_stage_ == 1) {
                cout << "At stage 1..." << endl;
                if (response.is_boolean()) {
                    cout << "Go to stage 0" << endl;
                    this->handshake_stage_ = 0;
                } else if (response.is_null()) {
                    cout << "Stay in stage 1" << endl;
                    continue;
                } else {
                    cout << "Go to stage 2" << endl;
                    this->handshake_stage_ = 2;
                }
            } else if (this->handshake_stage_ == 2) {
                cout << "At stage 2..." << endl;
                if (this->mios_synced_) {
                    response_ = response;
                    cout << "MiosEventServer: Telepresence successfully initiated" << endl;
                    json_ws_server_->stop_listening();
                    return true;
                }
            }
            this_thread::sleep_for(chrono::milliseconds(this->ms_loop_delay_));
        }
    } else {
        cout << "MiosEventServer: is not leader" << endl;
        while (this->running_) {
            if (this->handshake_stage_ == 0) { // waiting for incoming handshake request
                {
                    scoped_lock<mutex> lock(input_mutex_);
                    if (request_ != nullptr) {
                        try {
                            if (request_["name"] == "handshake") {
                                cout << "MiosEventServer: handshake received" << endl;
                                this->handshake_stage_ = 1;
                            }
                        } catch (exception &e) {
                            cerr << "Exception: " << e.what() << "\n";
                        }
                    }
                }
            }
            if (this->handshake_stage_ == 1) { // sending sync_done and waiting for result "true"
                cout << "MiosEventServer: sending sync_done" << endl;
                if (!JsonWebsocketClient::call_method(address, ws_port, "mios/core", "post_event",
                                                      {{"name", "sync_done"}}, response, 5)) {
                    response = false;
                } else {
                    this->handshake_stage_ = 2;
                }
            }
            if (this->handshake_stage_ == 2) {
                if (response.is_null()) {
                    cout << "MiosEventServer: got null response" << endl;
                    this->handshake_stage_ = 1;
                } else {
                    cout << "MioseventServer: Telepresence successfully initiated" << endl;
                    this->mios_synced_ = true;
                    return true;
                }
            }
            this_thread::sleep_for(chrono::milliseconds(this->ms_loop_delay_));
        }
        return false;
    }
    return true;
}

nlohmann::json MiosEventServer::WsCbPostEvent(const nlohmann::json &request) {
    scoped_lock<mutex> lock(input_mutex_);
    request_ = request;
    nlohmann::json response;
    string name;
    request["name"].get_to(name);
    cout << "MiosEventServer: request with name " + name + " received" << endl;
    response["result"] = true;
    if (name == "sync_done") {
        this->mios_synced_ = true;
    }
    return response;
}
