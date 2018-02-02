/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Pandar40 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

namespace pandar_pointcloud
{
/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new pandar_rawdata::RawData())
{
    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    output_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<pandar_pointcloud::
           CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<pandar_pointcloud::CloudNodeConfig>::
    CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);
    
    double start_angle;
    private_nh.param("start_angle", start_angle, 0.0);
    lidarRotationStartAngle = int(start_angle * 100);
    pandar_scan_ =
         node.subscribe("pandar_packets", 100,
                        &Convert::processScan, (Convert *) this,
                        ros::TransportHints().tcpNoDelay(true));
}


void Convert::callback(pandar_pointcloud::CloudNodeConfig &config,
                       uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    data_->setParameters(config.min_range, config.max_range, config.view_direction,
                         config.view_width);
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg)
{
    if (output_.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    pandar_rawdata::PPointCloud::Ptr outMsg(new pandar_rawdata::PPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    // TODO
    //outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
    // outMsg->is_dense = false;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    //process each packet provided by the driver
    double firstStamp = 0.0f;
    int ret = data_->unpack(scanMsg, *outMsg , gps1 , gps2 , firstStamp, lidarRotationStartAngle);

    // publish the accumulated cloud message
	ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
					 << " Pandar40 points, time: " << outMsg->header.stamp);

    if(ret == 1)
    {
        //pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
        pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
        output_.publish(outMsg);
    }
}


} // namespace pandar_pointcloud
