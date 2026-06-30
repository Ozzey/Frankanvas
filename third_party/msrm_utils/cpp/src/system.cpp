#include "msrm_utils/system.hpp"
#include "msrm_utils/output.hpp"

#include <unistd.h>
#include <linux/limits.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <memory>
#include <array>
#include <stdexcept>
#include <cstdio>
#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <random>


namespace msrm_utils {

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

std::string get_path_executable(char **argv){
    char path_save[PATH_MAX];
    char abs_exe_path[PATH_MAX];
    char *p;

    if(!(p = strrchr(argv[0], '/')))
        char* rtn=getcwd(abs_exe_path, sizeof(abs_exe_path));
    else
    {
        *p = '\0';
        char* rtn=getcwd(path_save, sizeof(path_save));
        int r=chdir(argv[0]);
        rtn=getcwd(abs_exe_path, sizeof(abs_exe_path));
        r=chdir(path_save);
    }
    return std::string(abs_exe_path);
}

bool process_is_running(const std::string &process){
    std::string cmd = "pidof -x " + process + " > /dev/null";
    return system(cmd.c_str()) == 0;
}

std::vector<int> get_video_device_id(const std::string &device_name){
    std::vector<int> valid_devices;
    unsigned i=0;
    while(true){
        int fd; // A file descriptor to the video device
        int device_id = i;
        std::string device = "/dev/video"  + std::to_string(device_id);
        i++;
        fd = open(device.c_str(),O_RDWR);
        if(fd < 0){
            perror("Failed to open device, OPEN");
            close(fd);
            return valid_devices;
        }

        // 2. Ask the device if it can capture frames
        v4l2_capability capability;
        if(ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0){
            // something went wrong... exit
            perror("Failed to get device capabilities, VIDIOC_QUERYCAP");
            close(fd);
            continue;
        }
        std::stringstream ss;
        ss<<capability.card;

        if(ss.str()!=device_name){
            std::cout<<device_name<<" is not "<<capability.card<<std::endl;
            close(fd);
            continue;
        }

        v4l2_format imageFormat;
        imageFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        imageFormat.fmt.pix.width = 1024;
        imageFormat.fmt.pix.height = 1024;
        imageFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        imageFormat.fmt.pix.field = V4L2_FIELD_NONE;
        // tell the device you are using this format
        if(ioctl(fd, VIDIOC_S_FMT, &imageFormat) < 0){
            perror("Device could not set format, VIDIOC_S_FMT");
            close(fd);
            continue;
        }


        // 4. Request Buffers from the device
        v4l2_requestbuffers requestBuffer = {0};
        requestBuffer.count = 1; // one request buffer
        requestBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // request a buffer wich we an use for capturing frames
        requestBuffer.memory = V4L2_MEMORY_MMAP;

        if(ioctl(fd, VIDIOC_REQBUFS, &requestBuffer) < 0){
            perror("Could not request buffer from device, VIDIOC_REQBUFS");
            close(fd);
            continue;
        }
        close(fd);
        valid_devices.push_back(device_id);
    }
    return valid_devices;
}

std::string generate_uuid(){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);
    std::uniform_int_distribution<> dis2(8, 11);
    std::stringstream ss;
    int i;
    ss << std::hex;
    for (i = 0; i < 8; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 4; i++) {
        ss << dis(gen);
    }
    ss << "-4";
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen);
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 12; i++) {
        ss << dis(gen);
    };
    return ss.str();
}

}
