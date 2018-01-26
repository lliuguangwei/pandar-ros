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

    // subscribe to PandarScan packets
    pandar_scan_ =
        node.subscribe("pandar_packets", 100,
                       &Convert::processScan, (Convert *) this,
                       ros::TransportHints().tcpNoDelay(true));
    pandar_gps_ =
        node.subscribe("pandar_gps", 1,
                       &Convert::processGps, (Convert *) this,
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
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    // pcl_conversions::toPCL(ros::Time::now(), outMsg->header.stamp);
    outMsg->is_dense = false;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
    // for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    // {
    //     data_->unpack(scanMsg->packets[i], *outMsg);
    // }
    double firstStamp = 0.0001f;  //TODO
    int ret = data_->unpack(scanMsg, *outMsg , gps1 , gps2 , firstStamp);

    // publish the accumulated cloud message
	ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
					 << " Pandar40 points, time: " << outMsg->header.stamp);


    if(ret == 1)
    {
        ROS_ERROR("stamp : %lf " , firstStamp);
        pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
        output_.publish(outMsg);
    }
}

void Convert::processGps(const pandar_msgs::PandarGps::ConstPtr &gpsMsg)
{
    if (output_.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work
    
    // ROS_ERROR("second : %d %d " , gpsMsg->second , gps2.gps);
    struct tm t;
    t.tm_sec = gpsMsg->second;
    t.tm_min = gpsMsg->minute;
    t.tm_hour = gpsMsg->hour;
    t.tm_mday = gpsMsg->day;
    t.tm_mon = gpsMsg->month - 1;
    t.tm_year = gpsMsg->year + 2000 - 1900;
    t.tm_isdst = 0;  
    if(lastGPSSecond != (mktime(&t) + 1))
    {
        lastGPSSecond = (mktime(&t) + 1);
        gps2.gps = mktime(&t) + 1; // the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
        gps2.used = 0;
    }
}

} // namespace pandar_pointcloud
