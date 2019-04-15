#include <proj_api.h>
#include "Eigen/Geometry"
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>


#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"


#include "modules/drivers/fakeGps/fake_gps.h"
#include "modules/drivers/fakeGps/proto/fakeGps.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/apriltags/proto/aprilTags.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/common/adapters/adapter_gflags.h"


using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::modules::drivers::fakeGps::proto::Chatter;
using apollo::localization::Gps;
using apollo::drivers::gnss::Ins;
using apollo::localization::CorrectedImu;

using MessagePtr = ::google::protobuf::Message *;

constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";


projPJ wgs84pj_source_ = pj_init_plus(WGS84_TEXT);
projPJ utm_target_ = pj_init_plus( "+proj=utm +zone=32 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs");

auto talker_node = apollo::cyber::CreateNode("talker");

std::shared_ptr<apollo::cyber::Writer<apollo::localization::Gps>>
      gps_writer_ = talker_node->CreateWriter<Gps>(FLAGS_gps_topic);
      
std::shared_ptr<apollo::cyber::Writer<apollo::localization::CorrectedImu>>
      corrimu_writer_ = talker_node->CreateWriter<CorrectedImu>(FLAGS_imu_topic);;

std::vector< std::vector<double> > tags;

double x, y, c_x, c_y, c_xOffset, c_yOffset, a_xOffset, a_zOffset, cameraAngle;

double angleOffset = 0;
double currHead = 0;
double cameraAngleOffset = 0;

int detec = 0;
int tagId = 0;

template <class T>
inline T *As(::google::protobuf::Message *message_ptr) {
  return dynamic_cast<T *>(message_ptr);
}



void theHeading(){
  
  if(detec) {
    currHead = angleOffset + tags[tagId][2]*DEG_TO_RAD_LOCAL;
    cameraAngleOffset = currHead - cameraAngle;
  }
  else {
    currHead = cameraAngleOffset+cameraAngle;
  }
}

void updatePos(){

  if(detec){
    x = tags[tagId][0] + a_xOffset;
    y = tags[tagId][1] + a_zOffset;
    c_xOffset = c_x;
    c_yOffset = c_y;
  } else {
    
    // clockwise
    x = x + (c_x - c_xOffset)*cos(cameraAngleOffset) + (c_y - c_yOffset)*sin(cameraAngleOffset);
    y = y + (-c_x - c_xOffset)*sin(cameraAngleOffset) + (c_y - c_yOffset)*cos(cameraAngleOffset);

    // counterclockwise beroende på ifall vinkeln är negativ? osäker:
    //x = x + (c_x - c_xOffset)*cos(cameraAngleOffset) + (c_y - c_yOffset)*sin(cameraAngleOffset);
    //y = y + (-c_x - c_xOffset)*sin(cameraAngleOffset) + (c_y - c_yOffset)*cos(cameraAngleOffset);

  }
}


void PublishOdometry(/*const MessagePtr message*/) {

  //Ins *ins = As<Ins>(message);
  auto gps = std::make_shared<apollo::localization::Gps>();
  //double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());

  gps->mutable_header()->set_timestamp_sec(Time::Now().ToSecond());
  auto *gps_msg = gps->mutable_localization();
  
  // 1. pose xyz
  
  //double x = 0; //ins->position().lon();
  //double y = 0; //ins->position().lat();

  //pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

  updatePos();
  gps_msg->mutable_position()->set_x(x);
  gps_msg->mutable_position()->set_y(y);
  gps_msg->mutable_position()->set_z(0);

  // 2. orientation
  /*Eigen::Quaterniond q =
  Eigen::AngleAxisd(ins->euler_angles().z() - 90 * DEG_TO_RAD_LOCAL,
                        Eigen::Vector3d::UnitZ()) *
  Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
  Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());
*/
  gps_msg->mutable_orientation()->set_qx(1/*q.x()*/);
  gps_msg->mutable_orientation()->set_qy(2/*q.y()*/);
  gps_msg->mutable_orientation()->set_qz(3/*q.z()*/);
  gps_msg->mutable_orientation()->set_qw(4/*q.w()*/);

  gps_msg->mutable_linear_velocity()->set_x(1/*ins->linear_velocity().x()*/);
  gps_msg->mutable_linear_velocity()->set_y(2/*ins->linear_velocity().y()*/);
  gps_msg->mutable_linear_velocity()->set_z(3/*ins->linear_velocity().z()*/);

  gps_writer_->Write(gps);
}


void PublishCorrimu(/*const MessagePtr message*/) {
  //Ins *ins = As<Ins>(message);
  auto imu = std::make_shared<CorrectedImu>();
  //double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());
  imu->mutable_header()->set_timestamp_sec(Time::Now().ToSecond());

  auto *imu_msg = imu->mutable_imu();
  imu_msg->mutable_linear_acceleration()->set_x(1
      /*-ins->linear_acceleration().y()*/);
  imu_msg->mutable_linear_acceleration()->set_y(1/*ins->linear_acceleration().x()*/);
  imu_msg->mutable_linear_acceleration()->set_z(1/*ins->linear_acceleration().z()*/);

  imu_msg->mutable_angular_velocity()->set_x(1/*-ins->angular_velocity().y()*/);
  imu_msg->mutable_angular_velocity()->set_y(1/*ins->angular_velocity().x()*/);
  imu_msg->mutable_angular_velocity()->set_z(1/*ins->angular_velocity().z()*/);

  imu_msg->mutable_euler_angles()->set_x(1/*ins->euler_angles().x()*/);
  imu_msg->mutable_euler_angles()->set_y(1/*-ins->euler_angles().y()*/);
  
  //HEADING
  imu_msg->mutable_euler_angles()->set_z(currHead /* - 90 * DEG_TO_RAD_LOCAL*/);
//

  corrimu_writer_->Write(imu);
}

void aprilCallBack(
    const std::shared_ptr<apollo::modules::drivers::apriltags::proto::apriltags>& msg) {

  tagId = msg->tag_id();
  detec = msg->detec();
  a_xOffset = msg->x_offset();
  a_zOffset = msg->z_offset();
  angleOffset = msg->angle_offset();

}

bool fakeGps::Init() {
  
  AERROR << "Commontest component init";

  std::ifstream file("/apollo/modules/drivers/fakeGps/file.txt");
  std::ifstream filec("/apollo/modules/drivers/fakeGps/file.txt");

  int sz = 1+std::count(std::istreambuf_iterator<char>(filec), 
           std::istreambuf_iterator<char>(), '\n');

  AERROR<<sz;
  if(file.is_open())
  {
      double dubletas;
      for (int i = 0; i < sz; i++)
      {
          std::vector<double> temp;
          for (int i = 0; i < 3; i++)
          {
              file >> dubletas;
              temp.push_back(dubletas);
          }
          tags.push_back(temp);
      }

  }
  
  // create talker
  auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
  Rate rate(0.1);


  auto listener_node = apollo::cyber::CreateNode("listener");

  auto listener = listener_node->CreateReader<apollo::modules::drivers::apriltags::proto::apriltags>(
                                                     "channel/apriltags", aprilCallBack);

  int MAX_BUF = 128;
  int fd;
  std::string myfifo = "/tmp/myfifo";
  char buf[MAX_BUF];

  while (apollo::cyber::OK()) {
    fd = open(myfifo.c_str(), O_RDONLY /* | O_NONBLOCK */);
    read(fd, buf, MAX_BUF);
    close(fd);
    std::string bufS (buf);
    std::size_t sz1,sz2;  

  
    c_x = std::stod (bufS,&sz1);
    c_y = std::stod (bufS.substr(sz1), &sz2);
    cameraAngle = std::stod (bufS.substr(sz1 + sz2));
     
    
    theHeading();
    updatePos();
    
    AERROR << " Heading -> " << currHead*RAD_TO_DEG << " X-led -> " << x << " Y-led -> " << y;

    PublishOdometry();
	  PublishCorrimu();
	  
    static uint64_t seq = 0;
    auto msg = std::make_shared<Chatter>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_lidar_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq++);
    msg->set_content(bufS);
    talker->Write(msg);

  }
  return true;
}

bool fakeGps::Proc(const std::shared_ptr<Driver>& msg0,
                                 const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}