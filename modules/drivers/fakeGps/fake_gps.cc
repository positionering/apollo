#include <proj_api.h>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/drivers/apriltags/proto/aprilTags.pb.h"
#include "modules/drivers/fakeGps/fake_gps.h"
#include "modules/drivers/fakeGps/proto/fakeGps.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"

/**
 * ----------------------------------------------------
 * AXLAR STANDARDISERAS TILL RFU, 
 * RIGHT-FORWARD-UP, DVS X -> HÖGER, Y -> FRAMÅT, Z -> UPPÅT
 * 
 * EULER VINKLAR STANDARDISERAS TILL EFTER HUR BILEN GÖR
 * X -> PITCH, Y -> ROLL,  Z -> YAW
 * Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y.
 * in world coordinate (East/North/Up)
 * The roll, in (-pi/2, pi/2), corresponds to a rotation around the y-axis.
 * The pitch, in [-pi, pi), corresponds to a rotation around the x-axis.
 * The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
 * The direction of rotation follows the right-hand rule.
 * 
 * GLOBALA AXLAR ÄR EAST/NORTH/UP
 * The heading is zero when the car is facing East and positive when facing North.
 * 
 * https://github.com/ApolloAuto/apollo/blob/master/docs/specs/coordination.pdf?fbclid=IwAR1I_QIscIiIrL10o414f2dK_4AgsPq9Evo06zlqFi77ha1R6DVQFn3bFNk
 * https://github.com/ApolloAuto/apollo/blob/master/modules/localization/proto/pose.proto
 * 
 * -----------------------------------------------------------------------------------
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * T265: X -> HÖGER(MOT USB PORTEN), Y -> FRAMMÅT, Z -> UPPÅT
 * EULER VINKLARNA; Z -> YAW, X -> PITCH, Y -> ROLL
 * https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
 * 
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * ======================================================================
 * APRILTAG 3:
**/



using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::drivers::gnss::Ins;
using apollo::localization::CorrectedImu;
using apollo::localization::Gps;
using apollo::modules::drivers::fakeGps::proto::Chatter;

using MessagePtr = ::google::protobuf::Message *;

constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";

projPJ wgs84pj_source_ = pj_init_plus(WGS84_TEXT);
projPJ utm_target_ = pj_init_plus(
    "+proj=utm +zone=32 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs");

auto talker_node = apollo::cyber::CreateNode("talker");

std::shared_ptr<apollo::cyber::Writer<apollo::localization::Gps>> gps_writer_ =
    talker_node->CreateWriter<Gps>(FLAGS_gps_topic);

std::shared_ptr<apollo::cyber::Writer<apollo::localization::CorrectedImu>>
    corrimu_writer_ = talker_node->CreateWriter<CorrectedImu>(FLAGS_imu_topic);

std::vector<std::vector<double>> tags;
int detec = 0;
int tagId = 0;

template <class T>
inline T *As(::google::protobuf::Message *message_ptr) {
  return dynamic_cast<T *>(message_ptr);
}

struct Cord {
  Eigen::Vector3d trans;   //  x;y;z : (column vektor)
  Eigen::Vector3d angles;  //  pitch (x), roll (y), yaw (z)
  Eigen::Matrix3d rot;     //
};

Cord aprilToD435;

void rotFromAngles(Cord& c) {
  c.rot = Eigen::AngleAxisd(c.angles(2), Eigen::Vector3d::UnitZ()) *  // YAW
          Eigen::AngleAxisd(c.angles(0), Eigen::Vector3d::UnitX()) *  // PITCH
          Eigen::AngleAxisd(c.angles(1), Eigen::Vector3d::UnitY());   // ROLL
}

void anglesFromRot(Cord& c) {
  c.angles =
      c.rot.eulerAngles(0, 1, 2);  // 0 = roll(x), 2 = pitch(y), 1 = yaw(z)(???)
}

void anglesFromRot2(Cord& c){
  
  double roll = atan2(c.rot(2,1), c.rot(2,2));
  double pitch = atan2(-c.rot(2,0),sqrt(c.rot(2,1)*c.rot(2,1) + c.rot(2,2)*c.rot(2,2)));
  double yaw = atan2(c.rot(1,0), c.rot(0,0));

  c.angles << roll,pitch,yaw;
}


void logMatrix(std::string s, std::vector<std::vector<double>> l) {
  AERROR << "******************";
  AERROR<<s;
  for (int i = 0; i < 3; i++) {
    AERROR << l[i][0] << " " << l[i][1] << " " << l[i][2];
  }
}

void logMatrix(std::string s, Cord l) {
  AERROR << "******************";
  AERROR<<s;
  for (int i = 0; i < 3; i++) {
    AERROR << l.rot(i,0) << " " << l.rot(i,1) << " " << l.rot(i,2);
  }
}

void logVector(std::string s, std::vector<double> l) {
  AERROR << "---------------------------";
  AERROR << s << " " << l[0] << " " << l[1] << " " << l[2];
}

void logVector(std::string s, Eigen::Vector3d l) {
  AERROR << "---------------------------";
  AERROR << s << " " << l(0) << " " << l(1) << " " << l(2);
}


void PublishOdometry(Cord p /*const MessagePtr message*/) {
  // Ins *ins = As<Ins>(message);
  auto gps = std::make_shared<apollo::localization::Gps>();
  // double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());

  gps->mutable_header()->set_timestamp_sec(Time::Now().ToSecond());
  auto *gps_msg = gps->mutable_localization();

  // 1. translationT265e xyz

  // double x = 0; //ins->translationT265ition().lon();
  // double y = 0; //ins->translationT265ition().lat();

  // pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

  gps_msg->mutable_position()->set_x(1);
  gps_msg->mutable_position()->set_y(1);
  gps_msg->mutable_position()->set_z(1);

  // logVector(p);

  // 2. orientation
  /*Eigen::Quaterniond q =
  Eigen::AngleAxisd(ins->euler_T265AngleToTwizy().z() - 90 * DEG_TO_RAD_LOCAL,
                        Eigen::Vector3d::UnitZ()) *
  Eigen::AngleAxisd(-ins->euler_T265AngleToTwizy().y(),
  Eigen::Vector3d::UnitX()) *
  Eigen::AngleAxisd(ins->euler_T265AngleToTwizy().x(),
  Eigen::Vector3d::UnitY());
*/
  gps_msg->mutable_orientation()->set_qx(1 /*q.x()*/);
  gps_msg->mutable_orientation()->set_qy(2 /*q.y()*/);
  gps_msg->mutable_orientation()->set_qz(3 /*q.z()*/);
  gps_msg->mutable_orientation()->set_qw(4 /*q.w()*/);

  gps_msg->mutable_linear_velocity()->set_x(1 /*ins->linear_velocity().x()*/);
  gps_msg->mutable_linear_velocity()->set_y(2 /*ins->linear_velocity().y()*/);
  gps_msg->mutable_linear_velocity()->set_z(3 /*ins->linear_velocity().z()*/);

  gps_writer_->Write(gps);
}

void PublishCorrimu(Cord h /*const MessagePtr message*/) {
  // Ins *ins = As<Ins>(message);
  auto imu = std::make_shared<CorrectedImu>();
  // double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());
  imu->mutable_header()->set_timestamp_sec(Time::Now().ToSecond());

  auto *imu_msg = imu->mutable_imu();
  imu_msg->mutable_linear_acceleration()->set_x(
      1
      /*-ins->linear_acceleration().y()*/);
  imu_msg->mutable_linear_acceleration()->set_y(
      1 /*ins->linear_acceleration().x()*/);
  imu_msg->mutable_linear_acceleration()->set_z(
      1 /*ins->linear_acceleration().z()*/);

  imu_msg->mutable_angular_velocity()->set_x(
      1 /*-ins->angular_velocity().y()*/);
  imu_msg->mutable_angular_velocity()->set_y(1 /*ins->angular_velocity().x()*/);
  imu_msg->mutable_angular_velocity()->set_z(1 /*ins->angular_velocity().z()*/);

  imu_msg->mutable_euler_angles()->set_x(1 /*ins->euler_angles().x()*/);
  imu_msg->mutable_euler_angles()->set_y(1 /*-ins->euler_angles().y()*/);

  // HEADING
  imu_msg->mutable_euler_angles()->set_z(1 /* - 90 * DEG_TO_RAD_LOCAL*/);
  //

  // logVector(h);

  corrimu_writer_->Write(imu);
}

void aprilCallBack(
    const std::shared_ptr<apollo::modules::drivers::apriltags::proto::apriltags>
        &msg) {
  detec = msg->detec();

  if (tagId == 0 || detec) {
    tagId = msg->tag_id();
  }

  //Vi byter bilens koordinatsystem 
  aprilToD435.trans << msg->x(), msg->z(), -msg->y();    // x till x, z till y, -y till z

  if (detec) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        aprilToD435.rot(i,j) = msg->rots(3 * i + j);
      }
    }
    
    //För att få rotationsmatrisen i rätt kordinatsystem
    Eigen::Matrix3d rot_fix1; 
    rot_fix1 << 1, 0, 0,
                0, 0, 1,
                0, -1, 0;
  
    //För att få tillbaka resultatet i rätt kordinatsystem 
    //(Borde vara samma som rot_fix1 men rotationsmatrisen byter tecken på y)
    Eigen::Matrix3d rot_fix2;
    rot_fix2 << 1, 0, 0,
               0, 0, 1,
               0, 1, 0;           
      
    aprilToD435.rot = rot_fix1*aprilToD435.rot*rot_fix2.transpose();

    
  }
}


void ekv6(Cord& G_KS, const Cord& G_A, const Cord& A_B, const Cord& KS_K) {
	G_KS.rot = G_A.rot * A_B.rot * KS_K.rot.transpose();
	anglesFromRot(G_KS);
}

void ekv8(Cord& G_KS,const Cord& G_A,const Cord& A_B,const Cord& KS_K) {
	G_KS.trans = G_A.trans + G_A.rot.transpose() * A_B.trans - G_KS.rot.transpose() * KS_K.trans;
}


void initFile() {
  std::ifstream file("/apollo/modules/drivers/fakeGps/file.txt");

  int sz = 1 + std::count(std::istreambuf_iterator<char>(file),
                          std::istreambuf_iterator<char>(), '\n');

  file.clear();
  file.seekg(0, std::ios::beg);

  if (file.is_open()) {
    double dubletas;
    for (int i = 0; i < sz; i++) {
      std::vector<double> temp;
      for (int i = 0; i < 4; i++) {
        file >> dubletas;
        temp.push_back(dubletas);
      }
      tags.push_back(temp);
    }
  }
}

bool fakeGps::Init() {
  AERROR << "Commontest component init";

  initFile();

  auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
  auto listener_node = apollo::cyber::CreateNode("listener");

  auto listener =
      listener_node
          ->CreateReader<apollo::modules::drivers::apriltags::proto::apriltags>(
              "channel/apriltags", aprilCallBack);

  int MAX_BUF = 128;
  int fd;
  std::string myfifo = "/tmp/myfifo";
  char buf[MAX_BUF];

  // aprilToD435
  Cord t, T265, globalToT265s, D435ToTwizy, globalToApril, T265ToTwizy;// aprilToD435(definerad globalt)

  while (apollo::cyber::OK()) {
    fd = open(myfifo.c_str(), O_RDONLY /* | O_NONBLOCK */);
    read(fd, buf, MAX_BUF);
    close(fd);
    std::string bufS(buf);
    std::size_t sz1, sz2, sz3, sz4, sz5, sz6, sz7, sz8;

    T265.trans(0) = std::stod(bufS, &sz1);
    T265.trans(2) = std::stod(bufS.substr(sz1), &sz2);
    T265.trans(1) = -std::stod(bufS.substr(sz1 + sz2), &sz3);

    T265.angles(1) = -std::stod(bufS.substr(sz1 + sz2 + sz3), &sz4);  // pitch
    T265.angles(2) =
        M_PI - std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4), &sz5);  // yaw
    T265.angles(0) =
        M_PI_2 - std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4 + sz5), &sz6);  // roll

    rotFromAngles(T265);


    // double speedz = std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4 + sz5 + sz6), &sz7);
    // double accX = std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4 + sz5 + sz6 + sz7), &sz8);
    // double accZ = std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4 + sz5 + sz6 + sz7 + sz8));

//    AERROR << speedz << " " << accX << " " << accZ;

    globalToApril.trans <<  tags[tagId][0], tags[tagId][1], tags[tagId][2];
    globalToApril.angles << 0, 0, tags[tagId][3] * (M_PI / 180); //vinkel konverterad till radianer
    rotFromAngles(globalToApril);
    
    ekv6(globalToT265s, globalToApril, aprilToD435, T265);
    logMatrix("Ekvation 6 ",globalToT265s);

    ekv8(globalToT265s, globalToApril, aprilToD435, T265);
    logVector("Ekvation 6 ",globalToT265s.trans);
    
    //logMatrix("t265 matrix", T265);
    //logVector("t265 vinkel",T265.angles);
    //logVector("t265 trans",T265.trans);


    logVector("D435 trans",globalToApril.rot.transpose()*aprilToD435.rot.transpose()*aprilToD435.trans+globalToApril.trans);
    //logVector("apriltag vinkel vec ",rotMatrixToOrientation(rotationMatrixAprilToD435));

    logVector("T265 trans",globalToT265s.rot.transpose()*T265.trans+globalToT265s.trans);


    //logMatrix("apriltag matrix",aprilToD435);
    //logMatrix("apriltag matrix vec",rotationMatrixAprilToD435);

    //logMatrix("apriltag matrix ",globalToApril);


    PublishOdometry(t);
    PublishCorrimu(t);


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

bool fakeGps::Proc(const std::shared_ptr<Driver> &msg0,
                   const std::shared_ptr<Driver> &msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}