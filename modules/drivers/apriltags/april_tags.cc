/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/drivers/apriltags/april_tags.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cyber/cyber.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <cstdlib>
#include <iostream>

#include <errno.h>

#include "modules/drivers/apriltags/proto/aprilTags.pb.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "modules/drivers/proto/sensor_image.pb.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::modules::drivers::apriltags::proto::apriltags;
using apollo::drivers::Image;


extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include <apriltag/common/getopt.h>
#include <apriltag/apriltag_pose.h>
}



cv::Mat image;

void MessageCallback(const std::shared_ptr<apollo::drivers::CompressedImage>& compressed_image) {

  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                           compressed_image->data().end());
  cv::Mat mat_image = cv::imdecode(compressed_raw_data, CV_LOAD_IMAGE_COLOR);
  cv::cvtColor(mat_image, mat_image, CV_BGR2RGB);

  image = mat_image;
}


bool aprilTags::Init() {
  AERROR << "Commontest component init";

  auto listener_node = apollo::cyber::CreateNode("listener_cam");
  // create listener
  auto listener =
      listener_node->CreateReader<apollo::drivers::CompressedImage>(
          "/apollo/sensor/camera/front_6mm/image/compressed", MessageCallback);
	
  auto talker_node_apriltags = apollo::cyber::CreateNode("apriltags_node");
  // create talker
  auto talker_apriltags = talker_node_apriltags->CreateWriter<apriltags>("channel/apriltags");


  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_family_t *tf = tag36h11_create();
  apriltag_detector_add_family(td, tf);

  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;

  cv::Mat frame, gray;
  double dist;
  int tagId,detec;
  double x_offset, z_offset, x_offset_old ,z_offset_old, x_offset_skit, z_offset_skit,phi,phi_old;
  double alpha = 0.1;
  bool first = true;
  
  while (apollo::cyber::OK()) {
    dist = 0;
    tagId = 0;
    detec = 0;


  if(image.data) {
      frame = image;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

      image_u8_t im = { .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
      };

      zarray_t *detections = apriltag_detector_detect(td, &im);

        // Draw detection outlines
      for (int i = 0; i < zarray_size(detections); i++) {

     
          detec = 1;
          
          apriltag_detection_t *det;
          zarray_get(detections, i, &det);

			    tagId = det->id; 
          // First create an apriltag_detection_info_t struct using your known parameters.
          
          apriltag_detection_info_t info;
          info.det = det;
          info.tagsize = 0.088; //size april tag (88 mm liten, 185 mm stor)
          info.fx = 1394.33;//1983.97376;
          info.fy = 1394.95;//1981.62916;
          info.cx = 964.117;//998.341216;
          info.cy = 537.659;//621.618227;

          // Then call estimate_tag_pose.
          apriltag_pose_t pose;
          double err = estimate_tag_pose(&info, &pose);



          //double dist_2D = sqrt(((pose.t->data[0])*(pose.t->data[0])) +   
          //                      ((pose.t->data[2])*(pose.t->data[2])) ); // 2-dim kordinatsystem x-z-planet engligt apriltags system
 
          double x = pose.t->data[0];
          double z = pose.t->data[2];
          phi = atan2(-pose.R->data[6],sqrt(pose.R->data[7]*pose.R->data[7]+pose.R->data[8]*pose.R->data[8]));

          x_offset = cos(phi)*x-sin(phi)*z;
          z_offset = sin(phi)*x+cos(phi)*z;
        

        //complementary filter
          if (first) {
            first = false;
            phi_old = phi;
            x_offset_old = x_offset;
            z_offset_old = z_offset;
          } else {
            phi = alpha * phi + (1- alpha) * phi_old;
            phi_old = phi;

            x_offset = x_offset *alpha + (1-alpha)*x_offset_old;
            x_offset_old  = x_offset;
      
            z_offset = z_offset *alpha + (1-alpha) * z_offset_old;
            z_offset_old  = z_offset;
          }

      }
      zarray_destroy(detections);

    }
    auto msg = std::make_shared<apriltags>();
  
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_detec(detec);
    msg->set_tag_id(tagId);
    msg->set_x_offset(x_offset);
    msg->set_z_offset(z_offset);
    msg->set_angle_offset(phi);
    talker_apriltags->Write(msg);
  }
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);

  return true;
}

bool aprilTags::Proc(const std::shared_ptr<Driver>& msg0,
                                 const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}
