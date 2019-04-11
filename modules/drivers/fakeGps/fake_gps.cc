
#include "modules/drivers/fakeGps/fake_gps.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "cyber/cyber.h"


#include <cstdlib>
#include <iostream>
#include <string>

#include "modules/drivers/fakeGps/proto/fakeGps.pb.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"


#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"

#include "Eigen/Geometry"
#include <proj_api.h>

#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"

#include "modules/common/adapters/adapter_gflags.h"

#include "modules/drivers/apriltags/proto/aprilTags.pb.h"

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

double a[12][3] = {  
   {677498.87, 6397920.123, 0} ,   /*  ARRAY AV X, Y COORDINATER */
   {4, 5, 0} ,   /*  initializers for row indexed by 1 */
   {8, 9, 0} ,   /*  initializers for row indexed by 2 */
   {1, 2, 0} ,
   {1, 2, 0} ,
   {1, 2, 0} ,
   {677498.87, 6397920.123, 0} ,   /*  ARRAY AV X, Y COORDINATER */
   {4, 5, 0} ,   /*  initializers for row indexed by 1 */
   {8, 9, 0} ,   /*  initializers for row indexed by 2 */
   {1, 2, 0} ,
   {1, 2, 0} ,
   {1, 2, 0}
};

template <class T>
inline T *As(::google::protobuf::Message *message_ptr) {
  return dynamic_cast<T *>(message_ptr);
}

double x = 0;
double y = 0;
int aprilTag = 0;
double tagDist  = 0; 

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

  gps_msg->mutable_position()->set_x(a[aprilTag][0]);
  gps_msg->mutable_position()->set_y(a[aprilTag][1]);
  gps_msg->mutable_position()->set_z(a[aprilTag][2]);

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
  imu_msg->mutable_euler_angles()->set_z(1/*ins->euler_angles().z()*/ -
                                         90 * DEG_TO_RAD_LOCAL);

  corrimu_writer_->Write(imu);
}

void aprilCallBack(
    const std::shared_ptr<apollo::modules::drivers::apriltags::proto::apriltags>& msg) {
  AERROR << "Distance -> " << msg->total_dist() << " ID -> " << msg->tag_id();
  int tagId = msg->tag_id();
  int tagDist = msg->total_dist();
}

bool fakeGps::Init() {
  AERROR << "Commontest component init";
  
  // create talker
auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
  Rate rate(0.1);


 auto listener_node = apollo::cyber::CreateNode("listener");

  auto listener =
      listener_node->CreateReader<apollo::modules::drivers::apriltags::proto::apriltags>(
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

  
  //AERROR << bufS << " omvandlad " << buf;

  float xPos = std::stof (bufS,&sz1);
  float zPos = std::stof (bufS.substr(sz1), &sz2);
  float yRot = std::stof (bufS.substr(sz1 + sz2));

AERROR<<"X pos -> "<<xPos << " z Pos -> "<<zPos<<" y rot -> "<<yRot; 

  PublishOdometry();
	PublishCorrimu();
	static uint64_t seq = 0;
    auto msg = std::make_shared<Chatter>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_lidar_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq++);
    msg->set_content(bufS);
    talker->Write(msg);
    AINFO << "talker sent a message!";

  }
  return true;
}

bool fakeGps::Proc(const std::shared_ptr<Driver>& msg0,
                                 const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}
