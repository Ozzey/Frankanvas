#ifndef BVH_PARSER_H
#define BVH_PARSER_H

#include "bvh.h"
#include "joint.h"

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <ios>
#include <sstream>
#include <functional>
#include <locale>
#include <memory>
#include <iostream>
#include <stdlib.h>
namespace bf = boost::filesystem;

namespace {

const std::string kChannels = "CHANNELS";
const std::string kEnd = "End";
const std::string kEndSite = "End Site";
const std::string kFrame = "Frame";
const std::string kFrames = "Frames:";
const std::string kHierarchy = "HIERARCHY";
const std::string kJoint = "JOINT";
const std::string kMotion = "MOTION";
const std::string kOffset = "OFFSET";
const std::string kRoot = "ROOT";

const std::string kXpos = "Xposition";
const std::string kYpos = "Yposition";
const std::string kZpos = "Zposition";
const std::string kXrot = "Xrotation";
const std::string kYrot = "Yrotation";
const std::string kZrot = "Zrotation";

}

namespace bvh {

/** Bvh Parser class that is responsible for parsing .bvh file */
class Bvh_parser {
 public:
  /** Parses single bvh file and stored data into bvh structure
   *  @param  path  The path to file to be parsed
   *  @param  bvh   The pointer to bvh object where parsed data will be stored
   *  @return  0 if success, -1 otherwise
   */
    //##############################################################################
    // Main parse function
    //##############################################################################
    int button;
    bool first_flag = true;
    char MotionData_Temp[5000];
    int parse(const bf::path& path, Bvh* bvh) {
      //LOG(INFO) << "Parsing file : " << path;

      path_ = path;
      bvh_ = bvh;

      bf::ifstream file;
      file.open(path_);

      if (file.is_open()) {
        std::string token;

    #if MULTI_HIERARCHY == 1
        while (file.good()) {
    #endif
          file >> token;
          if (token == kHierarchy) {
            int ret = parse_hierarchy(file);
            if (ret)
              return ret;
          } else {
            //LOG(ERROR) << "Bad structure of .bvh file. " << kHierarchy
            //           << " should be on the top of the file";
            return -1;
          }
    #if MULTI_HIERARCHY == 1
        }
    #endif
      } else {
        //LOG(ERROR) << "Cannot open file to parse : " << path_;
        return -1;
      }
      file.close();
      //LOG(INFO) << "Successfully parsed file";
      return 0;
    }
    int parse_motion_package(const char* MotionData, int size) {
      int j;
      char separater[] = " :,\t\r";
      char *token;
      //std::cout << "Enter Parse Function" << std::endl;
      //std::cout << size << std::endl;
      memset(MotionData_Temp, 0, sizeof(MotionData_Temp));
      //std::cout << "Before memory copy" << std::endl;
      memcpy(MotionData_Temp, MotionData, size);
      bvh_->set_num_frames(1);
      //LOG(INFO) << "Parsing motion";
      //std::cout << "Get Motion Data" << MotionData_Temp << std::endl;
      token = strtok(MotionData_Temp, separater);
      //std::cout << "First Token (Counter)" << token << std::endl;
      token = strtok(NULL, separater);
      //std::cout << "Second Token (Timestamp)" << token << std::endl;
      token = strtok(NULL, separater);
      button = atoi(token);
      //std::cout << "Third Token (Button)" << token << std::endl;

      //std::cout << "Enter into the right parse function" << std::endl;

      //LOG(INFO) << "Num of frames : " << frames_num;
      //LOG(ERROR) << "Bad structure of .bvh file. Expected " << kFrames
      //           << ", but found \"" << token << "\"";
      bvh_->set_frame_time(0.016);
      double number;
      if (first_flag){
          first_flag = false;
      for (auto joint : bvh_->joints()) {
            std::vector <double> data;
            for (int j = 0; j < joint->num_channels(); j++) {
              token = strtok(NULL, separater);
              if (token == NULL) {
                  std::cout << "Wrong Motion Package" << std::endl;
              }
              number = atof(token);
              data.push_back(number);
            }
            // LOG(TRACE) << joint->name() << ": " << vtos(data);
            joint->add_frame_motion_data(data);
          }
      }
      else{
          for (auto joint : bvh_->joints()) {
                for (int j = 0; j < joint->num_channels(); j++) {
                  token = strtok(NULL, separater);
                  if (token == NULL) {
                      std::cout << "Wrong Motion Package" << std::endl;
                  }
                  number = atof(token);
                  joint->set_chananel_data(j, number);
                }
              }
          }
      //std::cout << "Exit Parse Function" << std::endl;
      return 0;
    }

 private:
  /** Parses single hierarchy in bvh file
   *  @param  file  The input stream that is needed for reading file content
   *  @return  0 if success, -1 otherwise
   */
    //##############################################################################
    // Function parsing hierarchy
    //##############################################################################
    int parse_hierarchy(std::ifstream& file) {
      //LOG(INFO) << "Parsing hierarchy";
      std::string token;
      int ret;

      if (file.good()) {
        file >> token;

        //##########################################################################
        // Parsing joints
        //##########################################################################
        if (token == kRoot) {
          std::shared_ptr <Joint> rootJoint;
          ret = parse_joint(file, nullptr, rootJoint);

          if (ret)
            return ret;

          //LOG(INFO) << "There is " << bvh_->num_channels() << " data channels in the"
          //          << " file";

          bvh_->set_root_joint(rootJoint);
        } else {
          //LOG(ERROR) << "Bad structure of .bvh file. Expected " << kRoot
          //           << ", but found \"" << token << "\"";
          return -1;
        }
      }

      if (file.good()) {
        file >> token;

        //##########################################################################
        // Parsing motion data
        //##########################################################################
        if (token == kMotion) {
            //ret = parse_motion(file);

            //if (ret)
            //return ret;
        } else {
          //LOG(ERROR) << "Bad structure of .bvh file. Expected " << kMotion
          //           << ", but found \"" << token << "\"";
          return -1;
        }
      }
      return 0;
    }

  /** Parses joint and its children in bvh file
   *  @param  file    The input stream that is needed for reading file content
   *  @param  parent  The pointer to parent joint
   *  @param  parsed  The output parameter, here will be stored parsed joint
   *  @return  0 if success, -1 otherwise
   */

    //##############################################################################
    // Function parsing joint
    //##############################################################################
    int parse_joint(std::ifstream& file,
        std::shared_ptr <Joint> parent, std::shared_ptr <Joint>& parsed) {

      //LOG(TRACE) << "Parsing joint";

      std::shared_ptr<Joint> joint = std::make_shared<Joint>();
      joint->set_parent(parent);

      std::string name;
      file >> name;

      //LOG(TRACE) << "Joint name : " << name;

      joint->set_name(name);

      std::string token;
      std::vector <std::shared_ptr <Joint>> children;
      int ret;

      file >> token;  // Consuming '{'
      file >> token;

      //############################################################################
      // Offset parsing
      //############################################################################
      if (token == kOffset) {
        Joint::Offset offset;

        try {
          file >> offset.x >> offset.y >> offset.z;
        } catch (const std::ios_base::failure e) {
          //LOG(ERROR) << "Failure while parsing offset";
          return -1;
        }

        joint->set_offset(offset);

        //LOG(TRACE) << "Offset x: " << offset.x << ", y: " << offset.y << ", z: "
                   //<< offset.z;

      } else {
        //LOG(ERROR) << "Bad structure of .bvh file. Expected " << kOffset << ", but "
        //           << "found \"" << token << "\"";

        return -1;
      }

      file >> token;

      //############################################################################
      // Channels parsing
      //############################################################################
      if (token == kChannels) {
        ret = parse_channel_order(file, joint);

        //LOG(TRACE) << "Joint has " << joint->num_channels() << " data channels";

        if (ret)
          return ret;
      } else {
        //LOG(ERROR) << "Bad structure of .bvh file. Expected " << kChannels
        //           << ", but found \"" << token << "\"";

        return -1;
      }

      file >> token;

      bvh_->add_joint(joint);

      //############################################################################
      // Children parsing
      //############################################################################

      while (file.good()) {
        //##########################################################################
        // Child joint parsing
        //##########################################################################
        if (token == kJoint) {
          std::shared_ptr <Joint> child;
          ret = parse_joint(file, joint, child);

          if (ret)
            return ret;

          children.push_back(child);

        //##########################################################################
        // Child joint parsing
        //##########################################################################
        } else if (token == kEnd) {
          file >> token >> token;  // Consuming "Site {"

          std::shared_ptr <Joint> tmp_joint = std::make_shared <Joint> ();

          tmp_joint->set_parent(joint);
          tmp_joint->set_name(kEndSite);
          children.push_back(tmp_joint);

          file >> token;

          //########################################################################
          // End site offset parsing
          //########################################################################
          if (token == kOffset) {
            Joint::Offset offset;

            try {
              file >> offset.x >> offset.y >> offset.z;
            } catch (const std::ios_base::failure e) {
            //  LOG(ERROR) << "Failure while parsing offset";
              return -1;
            }

            tmp_joint->set_offset(offset);

            //LOG(TRACE) << "Joint name : EndSite";
            //LOG(TRACE) << "Offset x: " << offset.x << ", y: " << offset.y << ", z: "
            //          << offset.z;

            file >> token;  // Consuming "}"

          } else {
            //LOG(ERROR) << "Bad structure of .bvh file. Expected " << kOffset
            //           << ", but found \"" << token << "\"";

            return -1;
          }

          bvh_->add_joint(tmp_joint);
        //##########################################################################
        // End joint parsing
        //##########################################################################
        } else if (token == "}") {
          joint->set_children(children);
          parsed = joint;
          return 0;
        }

        file >> token;
      }

      //LOG(ERROR) << "Cannot parse joint, unexpected end of file. Last token : "
      //           << token;
      return -1;
    }

  /** Parses order of channel for single joint
   *  @param  file    The input stream that is needed for reading file content
   *  @param  joint   The pointer to joint that channels order will be parsed
   *  @return  0 if success, -1 otherwise
   */
    //##############################################################################
    // Channels order parse function
    //##############################################################################
    int parse_channel_order(std::ifstream& file,
        std::shared_ptr <Joint> joint) {

      //LOG(TRACE) << "Parse channel order";

      int num;
      file >> num;
      //LOG(TRACE) << "Number of channels : " << num;

      std::vector <Joint::Channel> channels;
      std::string token;

      for (int i = 0; i < num; i++) {
        file >> token;
        if (token == kXpos)
          channels.push_back(Joint::Channel::XPOSITION);
        else if (token == kYpos)
          channels.push_back(Joint::Channel::YPOSITION);
        else if (token == kZpos)
          channels.push_back(Joint::Channel::ZPOSITION);
        else if (token == kXrot)
          channels.push_back(Joint::Channel::XROTATION);
        else if (token == kYrot)
          channels.push_back(Joint::Channel::YROTATION);
        else if (token == kZrot)
          channels.push_back(Joint::Channel::ZROTATION);
        else {
        //  LOG(ERROR) << "Not valid channel!";
          return -1;
        }
      }

      joint->set_channels_order(channels);
      return 0;
    }

  /** Parses motion part data
   *  @param  file    The input stream that is needed for reading file content
   *  @return  0 if success, -1 otherwise
   */
  //##############################################################################
  // Motion data parse function
  //##############################################################################
  int parse_motion(std::ifstream& file) {

    //LOG(INFO) << "Parsing motion";
    std::cout << "Enter into wrong parse function" << std::endl;

    std::string token;
    file >> token;

    int frames_num;

    if (token == kFrames) {
      file >> frames_num;
      bvh_->set_num_frames(frames_num);
      //LOG(INFO) << "Num of frames : " << frames_num;
    } else {
      //LOG(ERROR) << "Bad structure of .bvh file. Expected " << kFrames
      //           << ", but found \"" << token << "\"";

      return -1;
    }

    file >> token;

    double frame_time;

    if (token == kFrame) {
      file >> token;  // Consuming 'Time:'
      file >> frame_time;
      bvh_->set_frame_time(frame_time);
      //LOG(INFO) << "Frame time : " << frame_time;

      float number;
      for (int i = 0; i < frames_num; i++) {
        for (auto joint : bvh_->joints()) {
          std::vector <double> data;
          for (int j = 0; j < joint->num_channels(); j++) {
            file >> number;
            data.push_back(number);
          }
          // LOG(TRACE) << joint->name() << ": " << vtos(data);
          joint->add_frame_motion_data(data);
        }
      }
    } else {
      //LOG(ERROR) << "Bad structure of .bvh file. Expected " << kFrame
      //           << ", but found \"" << token << "\"";

      return -1;
    }

    return 0;
  }


  /** Trims the string, removes leading and trailing whitespace from it
   *  @param  s   The string, which leading and trailing whitespace will be
   *              trimmed
   */
  inline void trim(std::string &s) {
    s.erase( std::remove_if( s.begin(), s.end(),
        std::bind( std::isspace<char>, std::placeholders::_1,
        std::locale::classic() ) ), s.end() );
  }

  /** Converts the vector of float to string, ex. "el1, el2, el3"
   *  @param  vector  The data that will be converted to string
   *  @return  The string that will be created from input data
   */
  std::string vtos(const std::vector <float>& vector) {
    std::ostringstream oss;

    if (!vector.empty())
    {
      // Convert all but the last element to avoid a trailing ","
      std::copy(vector.begin(), vector.end()-1,
          std::ostream_iterator<float>(oss, ", "));

      // Now add the last element with no delimiter
      oss << vector.back();
    }

    return oss.str();
  }

  /** The path to file that was parsed previously */
  bf::path path_;

  /** The bvh object to store parsed data */
  Bvh* bvh_;
};

} // namespace
#endif  // BVH_PARSER_H
