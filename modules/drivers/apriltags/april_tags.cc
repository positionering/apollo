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
#include <iterator>
#include <vector>

#include "cyber/cyber.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <cstdlib>
#include <iostream>

#include <errno.h>

#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/drivers/apriltags/proto/aprilTags.pb.h"

#include <memory>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "modules/drivers/proto/sensor_image.pb.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::drivers::Image;
using apollo::modules::drivers::apriltags::proto::apriltags;

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/getopt.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
}

cv::Mat image;

void MessageCallback(
    const std::shared_ptr<apollo::drivers::CompressedImage> &compressed_image) {
  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                           compressed_image->data().end());
  cv::Mat mat_image = cv::imdecode(compressed_raw_data, CV_LOAD_IMAGE_COLOR);
  cv::cvtColor(mat_image, mat_image, CV_BGR2RGB);

  image = mat_image;
}

bool aprilTags::Init() {

   //  AERROR << "Commontest component init";
  auto listener_node = apollo::cyber::CreateNode("listener_cam");
  // create listener
  auto listener = listener_node->CreateReader<apollo::drivers::CompressedImage>(
      "/apollo/sensor/camera/front_6mm/image/compressed", MessageCallback);

  auto talker_node_apriltags = apollo::cyber::CreateNode("apriltags_node");
  // create talker
  auto talker_apriltags =
      talker_node_apriltags->CreateWriter<apriltags>("channel/apriltags");

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_family_t *tf = tag36h11_create();
  apriltag_pose_t pose;
  apriltag_detector_add_family(td, tf);

  td->quad_decimate = 2.0;
  td->quad_sigma = 0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;

  cv::Mat frame, gray;
  int tagId, detec;
  double x, y, z, x_old, y_old, z_old, x_filt, y_filt, z_filt, phi;
  double alpha = 0.1;
  bool first = true;

  while (apollo::cyber::OK()) {
    tagId = 0;
    detec = 0;

    if (image.data) {
      frame = image;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

      image_u8_t im = {.width = gray.cols,
                       .height = gray.rows,
                       .stride = gray.cols,
                       .buf = gray.data};

      zarray_t *detections = apriltag_detector_detect(td, &im);

      // Draw detection outlines
      for (int i = 0; i < zarray_size(detections); i++) {

        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        tagId = det->id;
        // First create an apriltag_detection_info_t struct using your known
        // parameters.

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = 0.1615;  // size april tag (88 mm liten, 185 mm stor) (0.1615 stor?)
        info.fx = 1660.70; // 1394.33;     // 1983.97376;
        info.fy = 1668.19; // 1394.95;     // 1981.62916;
        info.cx = 886.07; // 964.117;     // 998.341216;
        info.cy = 522.40; // 537.659;     // 621.618227;

        // Then call estimate_tag_pose.
        double err = estimate_tag_pose(&info, &pose);

        if (err < 5e0) {
          detec = 1;
          x = pose.t->data[0];
          y = pose.t->data[1];
          z = pose.t->data[2];
          //  phi =
          //  atan2(-pose.R->data[6],sqrt(pose.R->data[7]*pose.R->data[7]+pose.R->data[8]*pose.R->data[8]));

          //  x = cos(phi)*x-sin(phi)*z;
          //  z = sin(phi)*x+cos(phi)*z;

          // complementary filter
          if (first) {
            first = false;
            x_old = x;
            y_old = y;
            z_old = z;
          } else {
            x_filt = x * alpha + (1 - alpha) * x_old;
            x_old = x_filt;

            y_filt = y * alpha + (1 - alpha) * y_old;
            y_old = y_filt;

            z_filt = z * alpha + (1 - alpha) * z_old;
            z_old = z_filt;
          }
       // AERROR << "ERROR is this ---> " << err;
        // AERROR << "X is this ---> " << x;
        // AERROR << "Y is this ---> " << y;
        // AERROR << "Z is this ---> " << z;
        }
        // AERROR << "Z: " << z << " Z_filt: " << z_filt << " X: " << x
        //        << " X_filt: " << x_filt << " Phi " << phi
        //        << " Time: " << Time::Now().ToNanosecond();
      }
      zarray_destroy(detections);
    }
    auto msg = std::make_shared<apriltags>();

    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_detec(detec);
    msg->set_tag_id(tagId);
    msg->set_x(x);
    msg->set_y(y);
    msg->set_z(z);
    msg->set_angle_offset(phi);

    if (detec) {
      msg->mutable_rots()->Reserve(9);
      for (int i = 0; i < 9; i++) {
        msg->add_rots(pose.R->data[i]);
      }
    }
    talker_apriltags->Write(msg);
  }
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);

  return true;
}

bool aprilTags::Proc(const std::shared_ptr<Driver> &msg0,
                     const std::shared_ptr<Driver> &msg1) {
  // AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
  // << msg1->msg_id() << "]";
  return true;
}
