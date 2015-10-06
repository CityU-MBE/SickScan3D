/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
// %EndTag(MSG_HEADER)%

#include <sstream>
#include <math.h>
//#include <math.h>
#include "epos.h"

#define ORIGIN_OFFSET 0.06
#define ROTATION_ANGLE 2400

int EPOS_initialize(long &pos, long &origin_pos);

class MotorControl
{
  public:
  MotorControl()
  {
     pos = -99;
     origin_pos = 0;
     angle_ratio = M_PI/(double)4800;
     angle = 0;
     motorangle_pub = n.advertise<std_msgs::Float64>("motorangle", 10);
     EPOS_initialize(pos, origin_pos);
     timer_read = n.createTimer(ros::Duration(0.05), boost::bind(&MotorControl::read_timerCallback, this, _1));
//     timer_pub = n.createTimer(ros::Duration(0.001), boost::bind(&MotorControl::pub_timerCallback, this, _1));
  }

  ~MotorControl()
  {
      printEPOSstate();
      closeEPOS();
      printf("\n\n");
  }

 void read_timerCallback(const ros::TimerEvent &e)
{
    std_msgs::String msg_display;
    std::stringstream ss_display;
    std_msgs::Float64 msg;

// control the motor and publish the real actual position
        ROS_INFO("startread");
	readActualPosition(&pos);
        ROS_INFO("endread");

        angle = (double)(pos-origin_pos)*angle_ratio;	

	transform.setOrigin( tf::Vector3(0, -ORIGIN_OFFSET*sin(angle), -ORIGIN_OFFSET*cos(angle)) );
	q.setRPY(-angle, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "laser"));

//	printf("\nEPOS position: %ld.\n", pos);
	if(pos==origin_pos+ROTATION_ANGLE)
		moveAbsolute(origin_pos-ROTATION_ANGLE);
	else if(pos==origin_pos-ROTATION_ANGLE)
		moveAbsolute(origin_pos+ROTATION_ANGLE);


    ss_display << "EPOS position: " << pos;
    msg_display.data = ss_display.str(); 
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg_display.data.c_str());

    msg.data = pos;                                                                                                                                                  

// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    motorangle_pub.publish(msg);
}

  private:
     ros::NodeHandle n;
     tf::TransformBroadcaster br;
     tf::Transform transform;
     tf::Quaternion q;
     ros::Publisher motorangle_pub;
     ros::Timer timer_read;
//     ros::Timer timer_pub;

     long pos;
     long origin_pos;
     double angle_ratio;
     double angle;

};



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "motorcontrol");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
//  ros::NodeHandle n_;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%

// %EndTag(PUBLISHER)%

// create tf broadcaster

   MotorControl motorcontrol;

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(100.0);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%




int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%

   count++;
   ROS_INFO("whileloop:%d", count);

// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
  }

  /*****************************************/
  /*  all done, shutting off               */
  /*****************************************/
  
/*   printf("\ninitiating shutdown "); */
/*   changeEPOSstate(2); */


  return 0;
}
// %EndTag(FULLTEXT)%

// EPOS motor initialize 
int EPOS_initialize(long &pos, long &origin_pos){
  
  char name[128];
  int  n;
  WORD w = 0x0, estatus=0x0;
  //  WORD dw[2] = {0x0, 0x0};
  

  printf("\n   *** EPOS Motor Control ***\n\n");
  printf("\n an alternating '/', '\' char means that the program is waiting\n");
  printf(" for the EPOS to respond.\n");
 
 
  /* open EPOS device */
  if ( openEPOS("/dev/ttyUSB2") < 0) exit(-1);        



  /*  read manufactor device name */
  if ( readDeviceName(name) < 0) {
    fprintf(stderr, "ERROR: cannot read device name!\n");
  }
  else {
    printf("\ndevice name is: %s\n", name);
  }



  //  ask for software version (get HEX answer!!!)
  printf("\nsoftware version: %x\n", readSWversion() );
  

  //  RS232 timeout (14.1.35)
  if ( ( n = readRS232timeout() ) < 0) checkEPOSerror();
  else printf("\nRS232 timeout is: %d ms\n", n );
  


  // EPOS Status
  estatus = 0x0;
  if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror();
  else printf("\nEPOS status is: %#06x \n", estatus );
  
  //printEPOSstatusword(estatus);
  //printEPOSstate();

  
  /******************************************************
            switch on  
  ******************************************************/
  
  // 1. check if we are in FAULT state

  n = checkEPOSstate();
  if ( n == 11) {
    printf("EPOS is in FAULT state, doing FAULT RESET ");
    // should check which fault this is, assume it's a CAN error from
    // power-on
       
    
    // 1a. reset FAULT
    changeEPOSstate(6);
    
    
    // 1b. check status
    if ( checkEPOSstate() == 11 ){
      fprintf(stderr, "\nEPOS still in FAULT state, quit!\n");
      exit(1);
    }
    else printf("success!\n");
  }
  else if (n != 4 && n!= 7) { // EPOS not running, issue a quick stop
    //printf("\nsending QUICK STOP\n");
    changeEPOSstate(3);
    
    // 2. we should now be in 'switch on disabled' (2)
    n = checkEPOSstate();
    if ( n != 2 ){
      printf("EPOS is NOT in 'switch on disabled' state, quit!\n");
      printf("(%s(), %s line %d)\n", __func__, __FILE__, __LINE__);
      exit(1);
    }
    else {  // EPOS is in 'switch on disabled'
      //printf("EPOS is in 'switch on disabled' state, doing shutdown. \n");
      changeEPOSstate(0); // issue a 'shutdown'
      //printEPOSstate();
    }

    // 3. switch on
    printf("\nswitching on ");
    changeEPOSstate(1);
    //printEPOSstate();

    
    // 4. enable operation
    printf("\nenable operation " );
    changeEPOSstate(5);
  }
  //printEPOSstate();



  // position window
  unsigned long win;
  readPositionWindow(&win);
  printf("\nEPOS position window is %lu. ", win);
  if (win == 4294967295 ) printf("-> position window is switched off!\n");
  else printf("\n");
 


  // actual position
  readActualPosition(&pos);
  printf("\nEPOS position SHOULD be %ld. Starting... \n", pos);
  origin_pos = pos;

// check, if we are in Profile Position Mode
 // if (readOpMode() != E_PROFPOS) {
    if( setOpMode(E_PROFPOS) ) {
      fprintf(stderr, "ERROR: problem at %s; %s line %d\n",
	      __func__, __FILE__, __LINE__);
      return(-1);
    }
 // }

  moveAbsolute(origin_pos+ROTATION_ANGLE);
//  int sum = 0;

  return 0;
}
