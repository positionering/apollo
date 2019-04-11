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
#include "modules/drivers/gnss2/heading_gnss.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include <iostream>

#include "cyber/cyber.h"

#include "src/TCPClient.h"
#include <signal.h>

#include "cyber/time/rate.h"
#include "cyber/time/time.h"


#include "modules/drivers/gnss2/libsbp/sbp.h"
#include "modules/drivers/gnss2/libsbp/system.h"
#include "modules/drivers/gnss2/libsbp/orientation.h"
#include "modules/drivers/gnss2/libsbp/navigation.h"
#include "modules/drivers/gnss2/proto/headingGnss.pb.h"


using apollo::cyber::Rate;
using apollo::cyber::Time;

TCPClient tcp;
static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
static sbp_msg_callbacks_node_t baseline_heading_callback_node;
int socket_desc = -1;
double lat;
double lon;
double heading_gps;

//auto talker_node = apollo::cyber::CreateNode("talker");
//auto talker = talker_node->CreateWriter<apollo::modules::drivers::gnss2::proto::Chatter>("channel/gps2");
  
/*
void writeGpsMsg(){
  	static uint64_t seq = 0;
		auto msg = std::make_shared<apollo::modules::drivers::gnss2::proto::Chatter>();
		msg->set_timestamp(Time::Now().ToNanosecond());
		msg->set_seq(seq++);
		msg->set_long2(lon);
		msg->set_lat2(lat);
		talker->Write(msg);
}
*/

void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  //writeGpsMsg();
  (void)sender_id, (void)len, (void)msg, (void)context;
 // AERROR<<"HEARTBEAT!";
}


bool pos_llh_bestpos(msg_pos_llh_t* data){

//	AERROR<<"LATITUDE: "<<data->lat << " LONTITUDE: "<<data->lon;

	lat = data->lat;
	lon = data->lon;
	return true;
}

void baseline_heading_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_baseline_heading_t* data = (msg_baseline_heading_t*) msg;
	//u8 antenna_present = extract_flags(data->flags, 31, 31);

  heading_gps =(data->heading)/1000;

	AERROR<<"Baseline heading callback ->: "<<((data->heading)/1000);
}

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	(void) sender_id, (void) len, (void) msg, (void) context;

	msg_pos_llh_t* data = (msg_pos_llh_t*) msg;

	if(pos_llh_bestpos(data)){
		return;
	}
}


u32 socket_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  u32 result;
  result = read(socket_desc, buff, n);

  return result;
}


bool headingGnss::Init(){


 struct sockaddr_in server;
  socket_desc = socket(AF_INET , SOCK_STREAM , 0);
  if (socket_desc == -1)
  {
    AERROR<<"Could not create socket";
  }

  memset(&server, '0', sizeof(server));
  server.sin_addr.s_addr = inet_addr("192.168.0.223");
  server.sin_family = AF_INET;
  server.sin_port = htons(atoi("55555"));

  if (connect(socket_desc, (struct sockaddr *)&server , sizeof(server)) < 0)
  {
    AERROR<<"Connection error";
  }

	sbp_state_t s;

  sbp_state_init(&s);
  sbp_state_set_io_context(&s, this);
  sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL,
                        &heartbeat_callback_node);
  sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_llh_callback, NULL, 
  						&pos_llh_callback_node);	
  sbp_register_callback(&s, SBP_MSG_BASELINE_HEADING, &baseline_heading_callback, NULL, &baseline_heading_callback_node);

	


std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("start_node"));
  auto server2 = node->CreateService<apollo::modules::drivers::gnss2::proto::Chatter, apollo::modules::drivers::gnss2::proto::Chatter>(
      "test_server", [](const std::shared_ptr<apollo::modules::drivers::gnss2::proto::Chatter>& request,
                        std::shared_ptr<apollo::modules::drivers::gnss2::proto::Chatter>& response) {
     //   AERROR<<"long "<<lon<<" lat: "<<lat;
        response->set_long2(lon);
        response->set_lat2(lat);
        response->set_heading_gps(heading_gps);
      }); 

	while(apollo::cyber::OK()){

		sbp_process(&s, &socket_read);
	}
  return true;
}


bool headingGnss::Proc(const std::shared_ptr<Driver>& msg0,
                               const std::shared_ptr<Driver>& msg1) {
                               
       AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
       << msg1->msg_id() << "]";
                               
  return true;
}
