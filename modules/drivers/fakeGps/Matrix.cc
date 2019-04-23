#include <proj_api.h>
#include "Eigen/Geometry"
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <ios>


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

std::vector<double> translationAprilToD435(3,0);

double x, y, c_xOffset, c_yOffset, a_xOffset, a_zOffset;

double angleOffset = 0;
double currHead = 0;
double cameraAngleOffset = 0;

int detec = 0;
int tagId = 0;

std::vector< std::vector<double> >rotationMatrixAprilToD435(3,std::vector<double> (3,0));;

template <class T>
inline T *As(::google::protobuf::Message *message_ptr) {
  return dynamic_cast<T *>(message_ptr);
}





void theHeading(){
  /*if(detec) {
    currHead = angleOffset + tags[tagId][2]*DEG_TO_RAD_LOCAL;
    cameraAngleOffset = currHead - cameraangle[1];
  }
  else {
    currHead = cameraAngleOffset+cameraangle[1];
  }*/
}


void updatePos(){
/*
  if(detec){
    x = tags[tagId][0] + a_xOffset;
    y = tags[tagId][1] + a_zOffset;
    c_xOffset = c_x;
    c_yOffset = c_y;
  } else {
    
    // clockwise
    
    if(cameraAngleOffset > 0){
      x = x + (c_x - c_xOffset)*cos(cameraAngleOffset) + (c_y - c_yOffset)*sin(cameraAngleOffset);
      y = y + (-c_x - c_xOffset)*sin(cameraAngleOffset) + (c_y - c_yOffset)*cos(cameraAngleOffset);
    } else {
      // counterclockwise beroende på ifall vinkeln är negativ? osäker:
      x = x + (c_x - c_xOffset)*cos(cameraAngleOffset) + (c_y - c_yOffset)*sin(cameraAngleOffset);
      y = y + (-c_x - c_xOffset)*sin(cameraAngleOffset) + (c_y - c_yOffset)*cos(cameraAngleOffset);
    }    

  }*/
}


void PublishOdometry(std::vector<double> p/*const MessagePtr message*/) {

  //Ins *ins = As<Ins>(message);
  auto gps = std::make_shared<apollo::localization::Gps>();
  //double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());

  gps->mutable_header()->set_timestamp_sec(Time::Now().ToSecond());
  auto *gps_msg = gps->mutable_localization();
  
  // 1. translationT265e xyz
  
  //double x = 0; //ins->translationT265ition().lon();
  //double y = 0; //ins->translationT265ition().lat();

  //pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

  gps_msg->mutable_position()->set_x(p[0]);
  gps_msg->mutable_position()->set_y(p[1]);
  gps_msg->mutable_position()->set_z(p[2]);

  //logVector(p);

  // 2. orientation
  /*Eigen::Quaterniond q =
  Eigen::AngleAxisd(ins->euler_T265AngleToTwizy().z() - 90 * DEG_TO_RAD_LOCAL,
                        Eigen::Vector3d::UnitZ()) *
  Eigen::AngleAxisd(-ins->euler_T265AngleToTwizy().y(), Eigen::Vector3d::UnitX()) *
  Eigen::AngleAxisd(ins->euler_T265AngleToTwizy().x(), Eigen::Vector3d::UnitY());
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


void PublishCorrimu( std::vector<double> h /*const MessagePtr message*/) {
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

  imu_msg->mutable_euler_angles()->set_x(h[0]/*ins->euler_angles().x()*/);
  imu_msg->mutable_euler_angles()->set_y(h[1]/*-ins->euler_angles().y()*/);
  
  //HEADING
  imu_msg->mutable_euler_angles()->set_z(h[2] /* - 90 * DEG_TO_RAD_LOCAL*/);
//

  //logVector(h);

  corrimu_writer_->Write(imu);
}


void logMatrix(std::string s, std::vector<std::vector<double>> l){
  AERROR<<"";
  AERROR<<"*******"<< s <<"***********";
  for(int i = 0; i<3; i++){
    AERROR<<l[i][0]<<" "<<l[i][1]<<" "<<l[i][2];
  }
}


void logVector(std::string s, std::vector<double> l){
  AERROR<<"";
  AERROR<<"------------"<<s<<"---------------";
  AERROR<<l[0]<<" "<<l[1]<<" "<<l[2];
}


std::vector<std::vector<double>> multiplyMatrices(std::vector<std::vector<double>> firstMatrix, 
                          std::vector<std::vector<double>> secondMatrix) {

  std::vector<std::vector<double>> resultMatrix(3,std::vector<double> (3,0));

	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
		{
			for(int k=0; k<3; ++k)
			{
				resultMatrix[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
	
	return resultMatrix;
}


std::vector<std::vector<double>> transtranslationT265eMatrice(std::vector<std::vector<double>> v){
  
  std::vector<std::vector<double>> res(3,std::vector<double> (3,0));

  for(int i = 0; i<3; i++) {
    for(int j=0; j<3; j++) {
        res[j][i] = v[i][j];
    }
  }

  return res;

}


std::vector<double> multiplyMatrices2d(std::vector<std::vector<double>> firstMatrix, 
                          std::vector<double> secondMatrix) {

	int i, j;
  std::vector<double> resultMatrix(3,0);
	for(i = 0; i < 3; ++i) {
		for(j = 0; j < 3; ++j) {
				resultMatrix[i] += firstMatrix[i][j] * secondMatrix[j];
		}
	}
	
	return resultMatrix;
}


std::vector<double> rotMatrixToOrientation(std::vector<std::vector<double>> m){
  std::vector<double> res(3,0);
  
  double roll = atan2(m[2][1], m[2][2]);
  double pitch = atan2(-m[2][0],sqrt(m[2][1]*m[2][1] + m[2][2]*m[2][2]));
  double yaw = atan2(m[1][0], m[0][0]);

  res = {roll,pitch,yaw};

  return res;
}


std::vector<std::vector<double>> angleToRotation(std::vector<double> angle){

  std::vector<std::vector<double>> roll =  {{1,0,0},
                                                {0, cos(angle[0]), -sin(angle[0])},
                                                {0, sin(angle[0]), cos(angle[0])}};

  std::vector<std::vector<double>> pitch = {{cos(angle[1]),0,sin(angle[1])},
                                              {0,1,0},
                                              {-sin(angle[1]),0,cos(angle[1])}};

  std::vector<std::vector<double>> yaw = {{cos(angle[2]), -sin(angle[2]),0},
                                              {sin(angle[2]), cos(angle[2]), 0},
                                              {0,0,1}};


  return multiplyMatrices(yaw , multiplyMatrices(pitch, roll));
}

std::vector<double> addVectors(std::vector<double> v1,std::vector<double> v2){
  std::vector<double> res = {v1[0]+v2[0],v1[1]+v2[1],v1[2]+v2[2]};
  return res;
}

void aprilCallBack(
    const std::shared_ptr<apollo::modules::drivers::apriltags::proto::apriltags>& msg) {
  detec = msg->detec();
  
  if(tagId == 0 || detec){
    tagId = msg->tag_id();
  }
 
  translationAprilToD435[0] = msg->x_offset();
  translationAprilToD435[1] = msg->z_offset();

  angleOffset = msg->angle_offset();
  if(detec){
      for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
              rotationMatrixAprilToD435[i][j] = msg->rots(3*i+j);
          }
      }
    std::vector<double> testq = rotMatrixToOrientation(rotationMatrixAprilToD435);
    std::vector<double> tempsak(3,0);
    //korrigerar koordinatsystem från apriltags system till "Z uppåt, y frammåt, x höger"
    tempsak[0] = 0;//testq[0];
    tempsak[1] = 0;//testq[2];
    tempsak[2] = testq[1];
    rotationMatrixAprilToD435 = angleToRotation(tempsak);
  }
}


void initFile(){
  std::ifstream file("/apollo/modules/drivers/fakeGps/file.txt");

  int sz = 1+std::count(std::istreambuf_iterator<char>(file), 
           std::istreambuf_iterator<char>(), '\n');

  file.clear();
  file.seekg(0, std::ios::beg);
    
  if(file.is_open()) {
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

  auto listener = listener_node->CreateReader<apollo::modules::drivers::apriltags::proto::apriltags>(
                                                     "channel/apriltags", aprilCallBack);

  int MAX_BUF = 128;
  int fd;
  std::string myfifo = "/tmp/myfifo";
  char buf[MAX_BUF];


  //TRANSLATIONS
//std::vector<double> translationAprilToD435
  std::vector<double> translationT265(3,0);
  std::vector<double> translationT265ToTwizy(3,0);
  std::vector<double> translationD435ToTwizy(3,0);
  std::vector<double> translationGlobalToApril(3,0);
  ///////////////

  //ANGLES
  std::vector<double> T265Angle(3,0);
  std::vector<double> globalToAprilAngle(3,0);
  std::vector<double> D435ToTwizyAngle(3,0);
  std::vector<double> T265ToTwizyAngle(3,0);
  ///////////

  //ROTATIONMATRICES
//std::vector<std::vector<double>> rotationMatrixAprilToD435;
  std::vector<std::vector<double>> rotationMatrixD435ToTwizy(3,std::vector<double> (3,0));
  std::vector<std::vector<double>> rotationMatrixT265ToTwizy(3,std::vector<double> (3,0));
  std::vector<std::vector<double>> rotationMatrixT265(3,std::vector<double> (3,0));
  std::vector<std::vector<double>> rotationMatrixGlobalToApril(3,std::vector<double> (3,0));
  //////////////

  // INITS ifall det behöver justeras
  translationT265ToTwizy = {0,0,0};
  translationD435ToTwizy = {0,0,0};

  D435ToTwizyAngle = {0,0,0};
  rotationMatrixD435ToTwizy = angleToRotation(D435ToTwizyAngle);

  T265ToTwizyAngle = {0,0,0};
  rotationMatrixT265ToTwizy = angleToRotation(T265ToTwizyAngle);
  ////////////////


  while (apollo::cyber::OK()) {
    fd = open(myfifo.c_str(), O_RDONLY /* | O_NONBLOCK */);
    read(fd, buf, MAX_BUF);
    close(fd);
    std::string bufS (buf);
    std::size_t sz1,sz2,sz3,sz4,sz5;  

    translationT265[0] = std::stod (bufS,&sz1);
    translationT265[2] = std::stod (bufS.substr(sz1), &sz2);
    translationT265[1] = -std::stod (bufS.substr(sz1+sz2), &sz3);

    // Kordinatsystemet defineras om för att matcha bil-aplikationen bättre
   // translationT265[0] = -std::stod (bufS.substr(sz1+sz2), &sz3);  // x = -z
   // translationT265[1] = -std::stod (bufS,&sz1);                   // y = -x
   // translationT265[2] = std::stod (bufS.substr(sz1), &sz2);        // z = y (Detta gäller nog inte längre)


    T265Angle[1] = -std::stod (bufS.substr(sz1 + sz2 + sz3), &sz4);       // pitch
    T265Angle[2] = M_PI-std::stod (bufS.substr(sz1 + sz2 + sz3 + sz4),&sz5);  // yaw
    T265Angle[0] = M_PI_2-std::stod (bufS.substr(sz1 + sz2 + sz3 + sz4 + sz5)); // roll

    globalToAprilAngle = {0,0,tags[tagId][3] * (M_PI/180)};
    translationGlobalToApril = {tags[tagId][0], tags[tagId][1], tags[tagId][2]};


    rotationMatrixT265 = angleToRotation(T265Angle);
    rotationMatrixGlobalToApril = angleToRotation(globalToAprilAngle);

    theHeading();
    updatePos();

/*
*
* TEST GREJER
*
*/
    std::vector<std::vector<double>> inregrej =multiplyMatrices(rotationMatrixGlobalToApril, rotationMatrixAprilToD435);

    std::vector<double> test_rotgrej = rotMatrixToOrientation(inregrej);

    std::vector<double> test_temp =  multiplyMatrices2d( inregrej, translationAprilToD435);
    test_temp = addVectors(test_temp,translationGlobalToApril);


    double strlk = sqrt(test_temp[0]*test_temp[0]+test_temp[1]*test_temp[1]+test_temp[2]*test_temp[2]);
    double strlk2 = sqrt(translationAprilToD435[0]*translationAprilToD435[0]+
                          translationAprilToD435[1]*translationAprilToD435[1]+
                          translationAprilToD435[2]*translationAprilToD435[2]);

    //AERROR<<"storlek roterad " << strlk << " icke roterad " << strlk2;  
    logVector("apriltag trans ",test_rotgrej);
    logVector("apriltag trans ",translationAprilToD435);
    logVector("global trans ",test_temp);
    //logMatrix(inregrej);

/**
 * 
 * SLUT PÅ TEST
 * 
 * */
    PublishOdometry({0,0,0});
	  PublishCorrimu({0,0,0});
	  
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