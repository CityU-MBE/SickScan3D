///////////////////////////////////////////////////////////////////////////////
// this program just uses sicktoolbox to get laser scans, and then publishes
// them as ROS messages
//
// Copyright (C) 2014, Fangyi Zhang
//
// I am distributing this code under the BSD license:
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <csignal>
#include <cstdio>
#include <math.h>
#include <limits>
//#include <sicktoolbox/SickLMS2xx.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "laser_geometry/laser_geometry.h"
#include "laser_assembler/base_assembler.h"
#include "filters/filter_chain.h"
//#include <sensor_msgs/point_cloud_conversion.h>
//#include <diagnostic_updater/diagnostic_updater.h> // Publishing over the diagnostics channels.
//#include <diagnostic_updater/publisher.h>
//using namespace SickToolbox;

using namespace laser_geometry;
using namespace std;

namespace laser_assembler
{

/**
* \brief Maintains a history of laser scans and generates a point cloud upon request
*/
    class LaserScanAssembler : public BaseAssembler<sensor_msgs::LaserScan>
    {
	public:
	    LaserScanAssembler() : BaseAssembler<sensor_msgs::LaserScan>("max_scans"), filter_chain_("sensor_msgs::LaserScan")
	    {
            // ***** Set Laser Projection Method *****
            private_ns_.param("ignore_laser_skew", ignore_laser_skew_, true);

            // configure the filter chain from the parameter server
            filter_chain_.configure("filters", private_ns_);
            newMotorangle = false;

            // Have different callbacks, depending on whether or not we want to ignore laser skews.
            if (ignore_laser_skew_)
                start("scan");
            else
            {
                start();
                skew_scan_sub_ = n_.subscribe("scan", 10, &LaserScanAssembler::scanCallback, this);
            }
            cloud_pub = n_.advertise<sensor_msgs::PointCloud>("cloud", 10);
            motorangle_sub = n_.subscribe("motorangle", 10, &LaserScanAssembler::motorangleCallback, this);
	    }

	    ~LaserScanAssembler()
	    {

	    }

	    unsigned int GetPointsInScan(const sensor_msgs::LaserScan& scan)
	    {
		    return (scan.ranges.size ());
	    }

	    void ConvertToCloud(const string& fixed_frame_id, const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud& cloud_out)
	    {
            // apply filters on laser scan
            filter_chain_.update (scan_in, scan_filtered_);

            // convert laser scan to point cloud
            if (ignore_laser_skew_)  // Do it the fast (approximate) way
            {
                projector_.projectLaser(scan_filtered_, cloud_out);
                if (cloud_out.header.frame_id != fixed_frame_id)
                tf_->transformPointCloud(fixed_frame_id, cloud_out, cloud_out);
            }
            else                     // Do it the slower (more accurate) way
            {
                int mask = laser_geometry::channel_option::Intensity +
                laser_geometry::channel_option::Distance +
                laser_geometry::channel_option::Index +
                laser_geometry::channel_option::Timestamp;
                projector_.transformLaserScanToPointCloud (fixed_frame_id, scan_filtered_, cloud_out, *tf_, mask);
            }
            return;
	    }

	    void motorangleCallback(const std_msgs::Float64::ConstPtr& msg)
	    {
            newMotorangle = true;
            //ROS_INFO("motorangle: [%f]", msg->data);
	    }

	    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
	    {
            if (!ignore_laser_skew_)
            {
                ros::Duration cur_tolerance = ros::Duration(laser_scan->time_increment * laser_scan->ranges.size());
                if (cur_tolerance > max_tolerance_)
                {
                ROS_DEBUG("Upping tf tolerance from [%.4fs] to [%.4fs]", max_tolerance_.toSec(), cur_tolerance.toSec());
                assert(tf_filter_);
                tf_filter_->setTolerance(cur_tolerance);
                max_tolerance_ = cur_tolerance;
                }
                tf_filter_->add(laser_scan);
            }
            //ROS_INFO("scantime: [%f]", laser_scan->scan_time);
            //if (newMotorangle == true)
            //{
            newMotorangle = false;
            ConvertToCloud("laser", (*laser_scan), cloud);
            cloud_pub.publish(cloud);
            //}
            //merge_cloud = merge_cloud + cloud;
            //ROS_INFO("merge_cloud.points.size: [%d]", merge_cloud.points.size());
            /*if(merge_cloud.points.size()==0)
                merge_cloud = cloud;
            else
            {
                merge_cloud.points.resize(merge_cloud.points.size() + cloud.points.size());
                std::copy(cloud.points.begin(), cloud.points.end(), merge_cloud.points.begin() + merge_cloud.points.size());
            }
            */
	    }

	private:
	    bool ignore_laser_skew_;
	    bool newMotorangle;
	    laser_geometry::LaserProjection projector_;

	    ros::Subscriber skew_scan_sub_;
	    ros::Subscriber motorangle_sub;
	    ros::Duration max_tolerance_;   // The longest tolerance we've needed on a scan so far

	    filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
	    mutable sensor_msgs::LaserScan scan_filtered_;
	    sensor_msgs::PointCloud cloud;
	    //sensor_msgs::PointCloud merge_cloud;
	    ros::Publisher cloud_pub;

    };

}

using namespace laser_assembler;

// %Tag(CALLBACK)%
/*void motorangleCallback(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_INFO("motorangle: [%f]", msg->data);
}
*/
//void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
//{
//  ROS_INFO("scantime: [%f]", msg->scan_time);
//}
// %EndTag(CALLBACK)%

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
    ros::init(argc, argv, "scan3d");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    //ros::NodeHandle n;

    LaserScanAssembler laser_assembler;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    // %Tag(SUBSCRIBER)%
    //ros::Subscriber motorangle_sub = n.subscribe("motorangle", 10, motorangleCallback);

    //laser_assembler.ConvertToCloud("world", const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud& cloud_out)
    //  ros::Subscriber scan_sub = n.subscribe("scan", 10, scanCallback);
    //  ros::Subscriber scan_sub = n.subscribe("scan", 10, &LaserScanAssembler::scanCallback);
    // %EndTag(SUBSCRIBER)%

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    // %Tag(SPIN)%
    ros::spin();
    // %EndTag(SPIN)%

    return 0;
}
// %EndTag(FULLTEXT)%
