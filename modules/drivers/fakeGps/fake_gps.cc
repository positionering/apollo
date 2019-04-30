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
 * The heading is zero when the car is facing East and positive when facing
 *North.
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

/*-----------------------------------------------*/
/* START OF SECTION FOR DEFFENISION OF VARIABLES */
/*-----------------------------------------------*/

Eigen::Matrix3d R_GA, R_AB, R_KB, R_KsK, R_GKs;
Eigen::Matrix3d Rg_GA,Rg_AB,Rg_KB,Rg_KsK, Rg_GKs;
Eigen::Vector3d t_GA, t_BA, t_KB, t_KsK, t_GKs;

/*-----------------------------------------------*/
/*  END OF SECTION FOR DEFFENISION OF VARIABLES  */
/*-----------------------------------------------*/

Eigen::Matrix3d rotFromAngles(double yaw, double pitch, double roll) {
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *    // YAW
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *  // PITCH
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY());    // ROLL
  return rot;
}

void rotFromAngles(Cord &c) {
  c.rot = rotFromAngles(c.angles(2), c.angles(0), c.angles(1));
}

void anglesFromRot(Cord &c) {
  c.angles =
      c.rot.eulerAngles(0, 1, 2);  // 0 = roll(x), 2 = pitch(y), 1 = yaw(z)(???)
}

void anglesFromRot2(Cord &c) {
  double roll = atan2(c.rot(2, 1), c.rot(2, 2));
  double pitch = atan2(-c.rot(2, 0), sqrt(c.rot(2, 1) * c.rot(2, 1) +
                                          c.rot(2, 2) * c.rot(2, 2)));
  double yaw = atan2(c.rot(1, 0), c.rot(0, 0));

  c.angles << roll, pitch, yaw;
}

void logMatrix(std::string s, std::vector<std::vector<double>> l) {
  AERROR << "******************";
  AERROR << s;
  for (int i = 0; i < 3; i++) {
    AERROR << l[i][0] << " " << l[i][1] << " " << l[i][2];
  }
}

void logMatrix(std::string s, Eigen::Matrix3d l) {
  AERROR << "******************";
  AERROR << s;
  for (int i = 0; i < 3; i++) {
    AERROR << std::fixed << std::setprecision(3) << l(i, 0) << " " << l(i, 1) << " " << l(i, 2);
  }
}

void logVector(std::string s, std::vector<double> l) {
  AERROR << "---------------------------";
  AERROR << s << " " << l[0] << " " << l[1] << " " << l[2];
}

void logVector(std::string s, Eigen::Vector3d l) {
  AERROR << "---------------------------";
  AERROR << std::fixed << std::setprecision(3) << s << " " << l(0) << " " << l(1) << " " << l(2);
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


/*--------------------------------------------------*/
/*    START PÅ DEFENITION AV ROTATIONSFUNKTIONER    */
/*--------------------------------------------------*/

Eigen::Matrix3d R_AB_glob(Eigen::Matrix3d R_GA, Eigen::Matrix3d R_AB) {
  Eigen::Matrix3d rot;
  rot =  R_GA.transpose() * R_AB * R_GA;
  return rot;
}

Eigen::Matrix3d R_KB_glob(Eigen::Matrix3d R_GA, Eigen::Matrix3d R_AB, Eigen::Matrix3d R_KB) {
  Eigen::Matrix3d rot;
  rot =  R_GA.transpose() * R_AB.transpose() * R_KB * R_AB * R_GA;
  return rot;
}

Eigen::Matrix3d R_KsK_glob(Eigen::Matrix3d R_GA, Eigen::Matrix3d R_AB, Eigen::Matrix3d R_KB, Eigen::Matrix3d R_KsK) {
  Eigen::Matrix3d rot;
  rot = R_GA.transpose() * R_AB.transpose() * R_KB * R_KsK * R_KB.transpose() * R_AB * R_GA;
  return rot;
}

Eigen::Matrix3d ekv6(Eigen::Matrix3d R_GA, Eigen::Matrix3d R_AB, Eigen::Matrix3d R_KB, Eigen::Matrix3d R_KsK) {
  Eigen::Matrix3d rot,Rg_GA,Rg_AB,Rg_KB,Rg_KsK;
  Rg_GA = R_GA;
  Rg_AB = R_AB_glob(Rg_GA, R_AB);
  Rg_KB = R_KB_glob(Rg_GA, Rg_AB, R_KB);
  Rg_KsK = R_KsK_glob(Rg_GA, Rg_AB, Rg_KB, R_KsK);
  rot =  Rg_KsK.transpose() * Rg_KB.transpose() * Rg_AB * Rg_GA;
  return rot;
}

Eigen::Vector3d ekv8(Eigen::Vector3d t_GA, Eigen::Vector3d t_BA, Eigen::Vector3d t_KB, Eigen::Vector3d t_KsK,
                     Eigen::Matrix3d R_GA, Eigen::Matrix3d R_AB, Eigen::Matrix3d R_KB, Eigen::Matrix3d R_KsK) {
  Eigen::Vector3d trans;
  Eigen::Matrix3d Rg_GA,Rg_AB,Rg_KB,Rg_KsK;
  Rg_GA = R_GA;
  Rg_AB = R_AB_glob(Rg_GA, R_AB);
  Rg_KB = R_KB_glob(Rg_GA, Rg_AB, R_KB);
  Rg_KsK = R_KsK_glob(Rg_GA, Rg_AB, Rg_KB, R_KsK);

  trans = t_GA - R_GA * R_AB * (t_BA + R_KB.transpose() * (t_KB + R_KsK.transpose() * t_KsK));
  return trans;
}


/*-------------------------------------------------*/
/*    SLUT PÅ DEFENITION AV ROTATIONSFUNKTIONER    */
/*-------------------------------------------------*/


/*-----------------------------------------------*/
/*    START PÅ INLÄSNING AV DATA FRÅN APRILTAG   */
/*-----------------------------------------------*/

void aprilCallBack(
    const std::shared_ptr<apollo::modules::drivers::apriltags::proto::apriltags>
        &msg) {
  detec = msg->detec();

  if (tagId == 0 || detec) {
    tagId = msg->tag_id();
  }

  if (detec) {

    // Vi byter bilens koordinatsystem
    t_BA << msg->x(), msg->z(), -msg->y();  // x till x, z till y, -y till z
    //logVector("t_BA", t_BA);
  
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        R_AB(i, j) = msg->rots(3 * i + j);
      }
    }

    // För att få rotationsmatrisen i rätt kordinatsystem
    Eigen::Matrix3d rot_fix1;
    rot_fix1 << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    Eigen::Matrix3d rot_fix2;
    rot_fix2 << -1, 0, 0, 0, 0, -1, 0, -1, 0;

    R_AB = rot_fix1 * R_AB * rot_fix2.transpose();
    Rg_AB = R_AB_glob(Rg_GA, R_AB);
    //logMatrix("R_AB",R_AB);
  }

}
/*-----------------------------------------------*/
/*    SLUT PÅ INLÄSNING AV DATA FRÅN APRILTAG    */
/*-----------------------------------------------*/


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

  Cord t;

  /*---------------------------------------------------*/
  /*    START PÅ INLÄSNING AV DATA FRÅN KAMERAFILEN    */
  /*---------------------------------------------------*/

  // Placeholder tills detta läses in från fil
  t_KB << 0, 0, 0;
  //logVector("t_KB",t_KB);

  R_KB << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Rg_KB = R_KB_glob(Rg_GA, Rg_AB, R_KB);
  //logMatrix("R_KB",R_KB);

  /*---------------------------------------------------*/
  /*    SLUT PÅ INLÄSNING AV DATA FRÅN KAMERAFILEN    */
  /*---------------------------------------------------*/

  while (apollo::cyber::OK()) {
    fd = open(myfifo.c_str(), O_RDONLY /* | O_NONBLOCK */);
    read(fd, buf, MAX_BUF);
    close(fd);
    std::string bufS(buf);
    std::size_t sz1, sz2, sz3, sz4, sz5, sz6;  //, sz7, sz8;

    /*---------------------------------------------------*/
    /*    START PÅ INLÄSNING AV DATA FRÅN T265 KAMERAN   */
    /*---------------------------------------------------*/

    t_KsK(0) = std::stod(bufS, &sz1);
    t_KsK(2) = std::stod(bufS.substr(sz1), &sz2);
    t_KsK(1) = -std::stod(bufS.substr(sz1 + sz2), &sz3);

    //logVector("t_KsK", t_KsK);

    double roll = -std::stod(bufS.substr(sz1 + sz2 + sz3), &sz4);  // pitch
    double yaw = M_PI - std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4), &sz5);  // yaw
    double pitch = M_PI_2 - std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4 + sz5), &sz6);  // roll

    // Ska det vara transponat på denna kanske? NEJ
    R_KsK = rotFromAngles(yaw, pitch, roll);
    Rg_KsK = R_KsK_glob(Rg_GA, Rg_AB, Rg_KB, R_KsK);
    // logMatrix("R_KsK",R_KsK);

    // AERROR << "YAW: " << yaw;
    // AERROR << "PITCH: " << pitch;
    // AERROR << "ROLL: " << roll;
    //logMatrix("R_KsK", R_KsK);

    /*--------------------------------------------------*/
    /*    SLUT PÅ INLÄSNING AV DATA FRÅN T265 KAMERAN   */
    /*--------------------------------------------------*/

    // double speedz = std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4 + sz5 + sz6),
    // &sz7); double accX = std::stod(bufS.substr(sz1 + sz2 + sz3 + sz4 + sz5 +
    // sz6 + sz7), &sz8); double accZ = std::stod(bufS.substr(sz1 + sz2 + sz3 +
    // sz4 + sz5 + sz6 + sz7 + sz8));

    // AERROR << speedz << " " << accX << " " << accZ;

    /*----------------------------------------------------*/
    /*    START PÅ INLÄSNING AV DATA FRÅN APRILTAGFILEN   */
    /*----------------------------------------------------*/
    // NOTE: Själva inläsningen görs inte här men det är här den inlästa datan
    // används

    if (detec) {
      t_GA << tags[tagId][0], tags[tagId][1], tags[tagId][2];
      //logVector("t_GA", t_GA);

      double theta_april = tags[tagId][3] * (M_PI / 180);  // vinkel konverterad till radianer
      R_GA = rotFromAngles(theta_april, 0, 0);
      Rg_GA = R_GA;
      //logMatrix("R_GA", R_GA);
    }

    // HÄR HAR VI HÅRDKODAT IN MEDANS APRILTAGSYSTEMET ÄR TRASIGT
    t_GA << tags[12][0], tags[12][1], tags[12][2];
    double theta_april = tags[12][3] * (M_PI / 180);  // vinkel konverterad till radianer
    R_GA = rotFromAngles(theta_april, 0, 0);
    Rg_GA = R_GA;

     
    //logMatrix("R_GA", R_GA);
      t_BA <<  0,1,0;
      R_AB << -1,0,0,
              0,-1,0,
              0,0,1;
      Rg_AB = R_AB_glob(Rg_GA, R_AB);
 // HÄR SLUTAR HÅRDKODNINGEN

    /*----------------------------------------------------*/
    /*    SLUT PÅ INLÄSNING AV DATA FRÅN APRILTAGFILEN    */
    /*----------------------------------------------------*/

    /*---------------------------------------------*/
    /*    START PÅ BERÄKNING AV R_GKs OCH t_GKs    */
    /*---------------------------------------------*/

    if (detec) {
      R_GKs = ekv6(R_GA, R_AB, R_KB, R_KsK);
      //logMatrix("R_GKs", R_GKs);

      t_GKs = ekv8(t_GA, t_BA, t_KB, t_KsK,
                   R_GA, R_AB, R_KB, R_KsK);
      //logVector("t_GKs", t_GKs);
    }

    // HÄR HAR VI HÅRDKODAT IN MEDANS APRILTAGSYSTEMET ÄR TRASIGT
    R_GKs = ekv6(R_GA, R_AB, R_KB, R_KsK);
    Rg_GKs = R_GKs;

    
    //logMatrix("R_KsK", R_KsK);
    //logMatrix("R_GKs", R_GKs);
    //logMatrix("R_KsK*R_GKs", Rg_KsK * R_GKs);


    t_GKs = ekv8(t_GA, t_BA, t_KB, t_KsK,
                 R_GA, R_AB, R_KB, R_KsK);
    logVector("t_GKs", t_GKs);
    logVector("t_KsK", t_KsK);
    logVector("t_GKs + R_GKs * t_KsK",t_GKs + R_GKs * t_KsK);
    // HÄR SLUTAR DET HÅRDKODADE

    /*---------------------------------------------*/
    /*    SLUT PÅ BERÄKNING AV R_GKs OCH t_GKs     */
    /*---------------------------------------------*/


    //logVector("t_GKs fake",  -R_AB.transpose() * (t_BA + R_KsK * t_KsK));
    //logMatrix("R_KsK", R_KsK);
    //logVector("t_KKs (K)",  R_KsK.transpose() * -t_KsK);
    //logVector("t_KsK",  t_KsK);
    //logVector("0", t_KsK - R_KsK * (R_KsK.transpose() * t_KsK));
    //logVector("t_GKs",  t_GKs);

    //logVector("Bilen via april", t_GA - R_GA.transpose() * R_AB.transpose() * t_BA);

    // Transponat på R_KsK
    //logVector("Bilen via kamera",t_GKs + R_GKs.transpose() * (t_KsK + R_KsK.transpose() * t_KB));

    //logVector("t_KsK (G)",R_GKs * t_KsK);
    //logVector("t_KsK (G)",R_GKs.transpose() * t_KsK);
    //logVector("t_KsK (Ks)",t_KsK);


    /* KOMENTARER FRÅN JOHAN 25/4 19:30
    Just nu tycks apriltagen fungera som den ska. Den är fortfarande inte utförligt testad men det tycks som att det fungerar.
    Bilens position via apriltagen blir rätt.
    Jag har kollat ekvation 6 och den verkar ge rätt sak.
    Ekvation 8 där emot ser ut att fungera, nästan. Jag efter att ha undersökt saken verkar -R_KsK * t_KsK vara det som är problemet.
    Om man istället räknar med -R_KsK.transpose() * t_KsK verkar den räkna rätt och då verkar även ekvation 8 fungera som den ska.
    Då kvarsåt att hitta bilen via T265 kameran. Jag kom fram till att t_GKs + R_GKs.transpose() * (t_KsK + R_KsK * t_KB) borde vara
    ekvationen som gäller, detta är kasnke inte sant dock. Jag har inte räknat på det 100% ordentligt. Det fungerar ganska bra men
    inte tillräckligt. För vissa rotationer på t265an gör det inte riktigt som det ska. Mer än bara numerisk instabilitet (och felet
    t265 kameran borde inte spela roll då detta är till för att kalibrera just det felet). Då det behövdes ett R_KsK.transpose() i ekv8
    ska det kasnke vara ett med här (vet inte varför dock och test säger att det inte hjälper). 
    
    TILLLÄG 19:50
    t_KsK (G) = R_GKs.transpose() * t_KsK tycks vara problemet. Den gör rätt tills dess att t265an roterar. Då bli den fel. (Både med och
    utan transponat)
    
    TILLÄG 20:05
    Föregående komentar tog inte hänsyn till hur KsK eventuelt flyttat sig. Detta skulle göra att saker inte beter sig som man förväntar sig.
    */
    

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

/* JOHANS "PSUDO" FÖR ATT GÖR BRA SAKER

Läs in apriltagsen från filen
Läs in från kamerafilen
Definera rotationsmatriserna: R_GA, R_AB, R_KB, R_KsK & R_GKs
Definera translationsvektorerna: t_BA, t_GA, t_KsK, t_KB & t_GKs

uppdatera R_KB med värde från fil
uppdatera t_KB med värde från fil

medans fake_gps_OK

  Vänta på detektion

  Vid detektion
    uppdatera R_GA med info från apriltagfilen tillsamans med avlästa idet
    uppdatera R_AB med info från avlästa apriltagens

















*/