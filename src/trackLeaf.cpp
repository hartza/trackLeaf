#include "trackLeaf.h"
#include <tf/transform_broadcaster.h>
  
trackLeaf::trackLeaf()  
{
  //pmdCC_ = new pmd_camcube::PmdCamcube();
  integration_time_= PMD_INTEGRATION_TIME;
  calibration_on_ = true;
  initialised_ = false;
  poseFilter_ = new PoseEstimationFilter();
}
  
trackLeaf::~trackLeaf() {
  if (initialised_) {
    //pmdCC_->close();
    //delete pmdCC_;
    delete poseFilter_;
  }
}
  
void trackLeaf::initialize() {
  //pmdCC_->open(integration_time_,calibration_on_);
  ROS_INFO("Open camera connection");
  initialised_ = true;
} 

void trackLeaf::initializeFilterRegion() { 
  // Create the filtering object
  pass_.setFilterFieldName ("x");
  pass_.setFilterLimits (-0.03, 0.03);
  
  pass2_.setFilterFieldName ("y");
  pass2_.setFilterLimits (-0.03, 0.03);
}

void trackLeaf::filterRegion(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::PointXYZ> & cloud_filtered) {
  pcl::PointCloud<pcl::PointXYZ> tmp;
  
  pass_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
  pass_.filter (tmp);
  
  pass2_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(tmp));
  pass2_.filter (cloud_filtered); 
}

void trackLeaf::getData(sensor_msgs::PointCloud &cloud) {
     //get the rgb image
   ros::spinOnce();
  //pmdCC_->readData(cloud);
}

void trackLeaf::trackLeafCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
   ROS_INFO("New image!!");
}

int main( int argc, char** argv )
{
//TODO: sembla que en Ros la c�mera hauria de ser un servei apart!
  PoseEstimationFilter * poseFilter_ = new PoseEstimationFilter();
  bool saveResult=false;
  
  Vector3f axis3D;
  axis3D<<1,0,0;
  
  ros::init(argc, argv, "point_clouds");
  ros::NodeHandle n;
//  ros::Publisher marker_pub = n.advertise<sensor_msgs::PointCloud>("visualization_point_cloud", 10);
  ros::Publisher marker_pub = n.advertise<sensor_msgs::PointCloud2>("visualization_point_cloud", 10);
  ros::Publisher marker_pub2 = n.advertise<sensor_msgs::PointCloud2>("planar_point_cloud", 10);
  ros::Publisher marker_pub3 = n.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 10);
  ros::Publisher marker_pub4 = n.advertise<visualization_msgs::Marker>("planar_normal", 10);
  ros::Rate r(30);

  trackLeaf * tracker_ = new trackLeaf();
  tracker_->initialize(); 
  tracker_->initializeFilterRegion();
  //a veure si funciona amb pointclouds
  //pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  //pcl::PCDReader reader;
  //reader.read ("/home/galenya/iri/code/ros/stacks/point_cloud_perception/pcl/test/sac_plane_test.pcd", cloud2);
  sensor_msgs::PointCloud2 cloud2, cloud2_p;
  sensor_msgs::PointCloud cloud;
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ, cloudXYZ_filtered, cloudXYZ_p;
  // Fill in the cloud data
  cloudXYZ.header.frame_id = "/my_frame";
  cloud.header.frame_id = "/my_frame";
  
  
    // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int image_counter=0;
  
 
  //init arrow shape
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.lifetime = ros::Duration();

  marker.ns = "plane_normal";
  marker.id = 0;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.10;
  marker.scale.z = 0.10;
  
  //prepare for segment a plane
  
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
   
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  //seg.setModelType (pcl::SACMODEL_PLANE);
    
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE); 
  seg.setAxis(axis3D);

  
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  //inicialitza el tf
    static tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Subscriber sub = n.subscribe("pointcloud/cloud2_raw", 2, &trackLeaf::trackLeafCallback, tracker_);

  ros::spin();
/*
  while (ros::ok())
  {
    
    
    //tracker_->getData(cloud);
    //una mica recargolat no?
    //    point_cloud::toMsg (cloudXYZ,cloud2);
    //point_cloud::toMsg (cloudXYZ,cloud2);
    //point_cloud_converter::convert(cloud2,cloud);
    //sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
    pcl::fromROSMsg(cloud2,cloudXYZ);
    
    tracker_->filterRegion(cloudXYZ,cloudXYZ_filtered);
    
    //segment a plane
    seg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloudXYZ_filtered));
    seg.segment (inliers, coefficients);
    if (inliers.indices.size () == 0)
    {
      ROS_ERROR ("Could not estimate a planar model for the given dataset.");
      //return (-1);
    }
    else {
      
      // Extract the inliers
      extract.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloudXYZ_filtered));
      extract.setIndices (boost::make_shared<pcl::PointIndices> (inliers));
      extract.setNegative (false);
      extract.filter (cloudXYZ_p);
      //Find the max and min point pos
      float min_x=10000, max_x=-10000,min_y=10000,max_y=-10000;
      for (unsigned int i=0;i<cloudXYZ_p.size();i++) {
	if (cloudXYZ_p.points[i].x>max_x) max_x = cloudXYZ_p.points[i].x;
	if (cloudXYZ_p.points[i].x<min_x) min_x = cloudXYZ_p.points[i].x;
	if (cloudXYZ_p.points[i].y>max_y) max_y = cloudXYZ_p.points[i].y;
	if (cloudXYZ_p.points[i].y<min_y) min_y = cloudXYZ_p.points[i].y;
      }
      
      //loop the kalman filter
      BFL::ColumnVector measurement(MEAS_SIZE);
      BFL::ColumnVector mean(STATE_SIZE);
      BFL::ColumnVector motion(STATE_SIZE);

      motion = 0.0;
 
     motion(1) = 0.0;
     motion(2) = 0.0;
     motion(3) = 0.0;
     motion(4) = 0.0;

     poseFilter_->updateFromMotion(motion);
      
      measurement = 0.0;
 
     measurement(1) = coefficients.values[0];
     measurement(2) = coefficients.values[1];
     measurement(3) = coefficients.values[2];
     measurement(4) = coefficients.values[3];
     
      poseFilter_->updateFromMeasurement(measurement);
      
      poseFilter_->getPosteriorMean(mean);

      BFL::SymmetricMatrix cov(4);
      poseFilter_->getPosteriorCovariance(cov);
      
      //trobar el quaternio que representa la rotació
        Vector3f s(0,0,1);
	//Vector3f d(marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z);
//	Vector3f d(coefficients.values[0],coefficients.values[1],coefficients.values[2]);
	Vector3f d(mean(1),mean(2),mean(3));
	Vector3f v=(s+d);
      v.normalize();
      //El producte vectorial representa l'eix de rotació i l'escalar la rotació al voltant de l'eix
      Vector3f sxd=s.cross(d);
      
      
//      transform.setOrigin( tf::Vector3(marker.pose.position.x,marker.pose.position.y , marker.pose.position.z) );
      transform.setOrigin( tf::Vector3((max_x+min_x)/2,(max_y+min_y)/2,-mean(4)/mean(3)) );
  transform.setRotation(tf::Quaternion(sxd(0),sxd(1),sxd(2),s.dot(d)));
//  transform.setRotation( tf::Quaternion(msg->theta, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "my_frame", "planar_frame"));
      
  //publish the result
      pcl::toROSMsg(cloudXYZ_p,cloud2_p);
      marker_pub2.publish(cloud2_p);

      
//      std::cerr << "Model coefficients: " <<measurement(1)<< " " <<  measurement(2)<< " " <<  measurement(3)<< " " <<  mean(1)<< " " << mean(2)<< " " << mean(3)<< std::endl;
//      std::cerr << "Model coefficients: " << mean(1)<< " " << mean(2)<< " " << mean(3)<< " " << mean(4)<< std::endl;
      std::cerr << "Model coefficients: " << cov(1,1)<< " " << cov(2,2)<< " " << cov(3,3)<< " " << cov(4,4)<< std::endl;


      marker_pub4.publish(marker);
      
      sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
      //pcl::toROSMsg(cloudXYZ,cloud2);
      marker_pub.publish(cloud2);
      // Convert to the templated message type
      //    point_cloud::fromMsg(cloud2,cloudXYZ);
      //    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloudXYZ);
      if (saveResult) {
	std::string file_name("test_pcd"); 
	std::stringstream ss;
	ss << image_counter++;
	
	file_name+=ss.str();
	file_name+=".pcd";
	pcl::io::savePCDFileASCII (file_name, cloud2);
	ROS_INFO ("Saved %d data points to %s.", (int)cloud.points.size (),file_name.c_str());
      }
    }
    r.sleep();
    //ros::Duration(.1).sleep();
    
  }
  */
  ROS_INFO("Close camera connection");
  delete tracker_;
  return(0);
}

