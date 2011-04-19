#include "trackLeaf.h"


trackLeaf::trackLeaf (ros::NodeHandle n):firstLoop_ (true), calibration_on_ (true), initialised_ (false)
{
  //pmdCC_ = new pmd_camcube::PmdCamcube();
  integration_time_ = PMD_INTEGRATION_TIME;
  calibration_on_ = true;
  initialised_ = false;
  poseFilter_ = new PoseEstimationFilter ();

  // this->initializeFilterRegion();

  //Initialize the container box parameters
  //in the beggining the leaf is assumed to be centered, at a 60cm distance, 
  // and size_z is initlaized big to allow finding the leaf
  box_center_ << 0.0, 0.0, 0.6;
  box_size_ << 0.05, 0.05, 0.5;
  box_change_ << 0.01, 0.01, 0.01;

  pass_.setFilterFieldName ("x");
  //pass_.setFilterLimits (box_center_.x()-box_size_.x()-box_change_.x(), box_center_.x()+box_size_.x()+box_change_.x());

  pass2_.setFilterFieldName ("y");
  //pass2_.setFilterLimits (box_center_.y()-box_size_.y()-box_change_.y(), box_center_.y()+box_size_.y()+box_change_.y());

  pass3_.setFilterFieldName ("z");
  //pass3_.setFilterLimits (box_center_.z()-box_size_.z()-box_change_.z(), box_center_.z()+box_size_.z()+box_change_.z());

  // Initilize the planar segmentation object
  Vector3f axis3D;
  axis3D << 1, 0, 0;
  // Optional
  seg_.setOptimizeCoefficients (true);
  //seg_.setModelType (pcl::SACMODEL_PARALLEL_PLANE); 
  seg_.setModelType (pcl::SACMODEL_PLANE);
  // seg_.setAxis(axis3D);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setDistanceThreshold (0.01);

  //publishers

  cloudXYZ_.header.frame_id = cloud_.header.frame_id = "/my_frame";

  pub_ = n.advertise < sensor_msgs::PointCloud2 > ("visualization_point_cloud", 10);
  pub2_ = n.advertise < sensor_msgs::PointCloud2 > ("planar_point_cloud", 10);
//  pub3_ = n.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 10);
//  pub4_ = n.advertise<visualization_msgs::Marker>("planar_normal", 10);
}

trackLeaf::~trackLeaf ()
{
  if (initialised_)
  {
    //pmdCC_->close();
    //delete pmdCC_;
    delete poseFilter_;
  }
}

void
trackLeaf::initialize ()
{
  //pmdCC_->open(integration_time_,calibration_on_);
  ROS_INFO ("Open camera connection");
  initialised_ = true;
}

void
trackLeaf::initializeFilterRegion ()
{
  // Create the filtering object
  pass_.setFilterFieldName ("x");
  pass_.setFilterLimits (-0.05, 0.05);

  pass2_.setFilterFieldName ("y");
  pass2_.setFilterLimits (-0.05, 0.05);
}

void
trackLeaf::filterRegion (pcl::PointCloud < pcl::PointXYZ > cloud, pcl::PointCloud < pcl::PointXYZ > &cloud_filtered)
{
  pcl::PointCloud < pcl::PointXYZ > tmp;

  pass_.setInputCloud (boost::make_shared < pcl::PointCloud < pcl::PointXYZ > >(cloud));
  pass_.filter (tmp);

  pass2_.setInputCloud (boost::make_shared < pcl::PointCloud < pcl::PointXYZ > >(tmp));
  pass2_.filter (cloud_filtered);
}

void
trackLeaf::getData (sensor_msgs::PointCloud & cloud)
{
  //get the rgb image
  ros::spinOnce ();
  //pmdCC_->readData(cloud);
}

void
trackLeaf::initTracker ()
{
  filterRegion (cloudXYZ_, cloudXYZ_filtered_);
  if (cloudXYZ_filtered_.size () == 0)
    ROS_ERROR ("No points in the interest region!.");
  else
  {
    //segment a plane and set he initial coeficients
    seg_.setInputCloud (boost::make_shared < pcl::PointCloud < pcl::PointXYZ > >(cloudXYZ_filtered_));
    seg_.segment (inliers_, coefficients_);
    if (inliers_.indices.size () == 0)
    {
      ROS_ERROR ("Could not estimate a planar model for the given dataset.");
      //return (-1);
    }
  }
}


void
trackLeaf::segmentNeighbors (const pcl::PointCloud < pcl::PointXYZ > &input_cloudXYZ_,
                             pcl::PointCloud < pcl::PointXYZ > &output_cloudXYZ_)
{
  pcl::PointCloud < pcl::PointXYZ > tmp;

  //drop points out of the last box + delta
  //
  //  seg_.selectWithinDistance (const Eigen3::VectorXf &model_coefficients, double threshold, std::vector< int > &inliers)
  //pcl::SampleConsensusModelPlane<pcl::PointXYZ> seg2_(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloudXYZ_filtered_));
  //pass_.setFilterFieldName ("x");
  pass_.setFilterLimits (box_center_.x () - box_size_.x () - box_change_.x (),
                         box_center_.x () + box_size_.x () + box_change_.x ());
  pass_.setInputCloud (boost::make_shared < pcl::PointCloud < pcl::PointXYZ > >(input_cloudXYZ_));
  pass_.filter (tmp);

  //pass2_.setFilterFieldName ("y");
  pass2_.setFilterLimits (box_center_.y () - box_size_.y () - box_change_.y (),
                          box_center_.y () + box_size_.y () + box_change_.y ());
  pass2_.setInputCloud (boost::make_shared < pcl::PointCloud < pcl::PointXYZ > >(tmp));
  pass2_.filter (output_cloudXYZ_);

  //de moment faig la profunditat com una capsa, pero si cal més precisió es pot fer amb la distància dels punts al planar
  //pass3_.setFilterFieldName ("z");
  pass3_.setFilterLimits (box_center_.z () - box_size_.z () - box_change_.z (),
                          box_center_.z () + box_size_.z () + box_change_.z ());
  pass3_.setInputCloud (boost::make_shared < pcl::PointCloud < pcl::PointXYZ > >(output_cloudXYZ_));
  pass3_.filter (output_cloudXYZ_);

/*  for (unsigned int i=0;i<cloud_filtered_.size();i++) {
    //compute distance of point to plane
  }
  
  if (inliers_.indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
    //return (-1);
  }  */

}

void
trackLeaf::trackLeafCallback (const sensor_msgs::PointCloud2 & msg)
{
  //ROS_INFO("New image!!");    
  pcl::fromROSMsg (msg, cloudXYZ_);

//   if (firstLoop_) 
//   {
//      //initTracker();
//      firstLoop_=false;
//   }

  ros::Time time = ros::Time::now ();
  cloud2_p_.header.stamp = time;

  //make algorithm
  //1. From last plane model extract neighbor points using the SAC classe method
  segmentNeighbors (cloudXYZ_, cloudXYZ_filtered_);
  if (cloudXYZ_filtered_.size () < 10)
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
  else
  {
    //2. segment a plane on the remaining points
    seg_.setInputCloud (boost::make_shared < pcl::PointCloud < pcl::PointXYZ > >(cloudXYZ_filtered_));
    seg_.segment (inliers_, coefficients_);
    if (inliers_.indices.size () == 0)
    {
      ROS_ERROR ("Could not estimate a planar model for the given dataset.");
      //return (-1);
    }
    else
    {

      // Extract the inliers
      extract_.setInputCloud (boost::make_shared < pcl::PointCloud < pcl::PointXYZ > >(cloudXYZ_filtered_));
      extract_.setIndices (boost::make_shared < pcl::PointIndices > (inliers_));
      extract_.setNegative (false);
      extract_.filter (cloudXYZ_p_);
      //Find the max and min point pos to determine the centroid
      float min_x = 10000, max_x = -10000, min_y = 10000, max_y = -10000, min_z = 10000, max_z = -10000;
      for (unsigned int i = 0; i < cloudXYZ_p_.size (); i++)
      {
        if (cloudXYZ_p_.points[i].x > max_x)
          max_x = cloudXYZ_p_.points[i].x;
        if (cloudXYZ_p_.points[i].x < min_x)
          min_x = cloudXYZ_p_.points[i].x;
        if (cloudXYZ_p_.points[i].y > max_y)
          max_y = cloudXYZ_p_.points[i].y;
        if (cloudXYZ_p_.points[i].y < min_y)
          min_y = cloudXYZ_p_.points[i].y;
        if (cloudXYZ_p_.points[i].z > max_z)
          max_z = cloudXYZ_p_.points[i].z;
        if (cloudXYZ_p_.points[i].z < min_z)
          min_z = cloudXYZ_p_.points[i].z;
      }

      //recompute position and size of the box as the new observation
      //put a maximum size
      box_size_ (0) = (max_x - min_x);
      if (box_size_ (0) > MAX_X_BOX_SIZE)
        box_size_ (0) = MAX_X_BOX_SIZE;
      box_size_ (1) = (max_y - min_y);
      if (box_size_ (1) > MAX_Y_BOX_SIZE)
        box_size_ (1) = MAX_Y_BOX_SIZE;
      box_size_ (2) = (max_z - min_z);
      if (box_size_ (2) > MAX_Z_BOX_SIZE)
        box_size_ (2) = MAX_Z_BOX_SIZE;

      box_center_ (0) = min_x + (max_x - min_x) / 2;
      box_center_ (1) = min_y + (max_y - min_y) / 2;
      box_center_ (2) = min_z + (max_z - min_z) / 2;
      //loop the kalman filter
      //kalman filtering
      BFL::ColumnVector measurement_ (MEAS_SIZE);
      BFL::ColumnVector mean_ (STATE_SIZE);
      BFL::ColumnVector motion_ (STATE_SIZE);
      BFL::SymmetricMatrix cov_ (STATE_SIZE);

      motion_ = 0.0;

      motion_ (1) = 0.0;
      motion_ (2) = 0.0;
      motion_ (3) = 0.0;
      motion_ (4) = 0.0;

      poseFilter_->updateFromMotion (motion_);

      measurement_ = 0.0;

      measurement_ (1) = coefficients_.values[0];
      measurement_ (2) = coefficients_.values[1];
      measurement_ (3) = coefficients_.values[2];
      measurement_ (4) = coefficients_.values[3];

      poseFilter_->updateFromMeasurement (measurement_);

      poseFilter_->getPosteriorMean (mean_);

      poseFilter_->getPosteriorCovariance (cov_);

      //trobar el quaternio que representa la rotació
      Vector3f s (0, 0, 1);
      //Vector3f d(marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z);
      //  Vector3f d(coefficients.values[0],coefficients.values[1],coefficients.values[2]);
      Vector3f d (mean_ (1), mean_ (2), mean_ (3));
      Vector3f v = (s + d);
      v.normalize ();
      //El producte vectorial representa l'eix de rotació i l'escalar la rotació al voltant de l'eix
      Vector3f sxd = v.cross (d);

      Vector3f d0 (coefficients_.values[0], coefficients_.values[1], coefficients_.values[2]);
      Vector3f v0 = (s + d0);
      v0.normalize ();
      Vector3f sxd0 = v0.cross (d);
      //show results
      static tf::TransformBroadcaster br;

      //      transform.setOrigin( tf::Vector3(marker.pose.position.x,marker.pose.position.y , marker.pose.position.z) );
      transform_.setOrigin (tf::Vector3 ((max_x + min_x) / 2, (max_y + min_y) / 2,
                                         -coefficients_.values[3] / coefficients_.values[2]));
      transform_.setRotation (tf::Quaternion (sxd0 (0), sxd0 (1), sxd0 (2), v0.dot (d0)));
      transformFilter_.setOrigin (tf::Vector3 ((max_x + min_x) / 2, (max_y + min_y) / 2, -mean_ (4) / mean_ (3)));
      transformFilter_.setRotation (tf::Quaternion (sxd (0), sxd (1), sxd (2), v.dot (d)));
      //  transform.setRotation( tf::Quaternion(msg->theta, 0, 0) );
      br.sendTransform (tf::StampedTransform (transform_, ros::Time::now (), "my_frame", "planar_frame"));
      br.sendTransform (tf::StampedTransform (transformFilter_, ros::Time::now (), "my_frame", "planar_frame_filter"));
//      br.sendTransform (tf::StampedTransform (tf::Transform (tf::Quaternion (0, 0, 0, 1), tf::Vector3 (0, 0, 0)),
//                                              ros::Time::now (), "my_frame", "kinect_depth"));
      // relate the original pointcloud tf to this one to allow rviz visualization
      br.sendTransform (tf::StampedTransform (tf::Transform (tf::Quaternion (0, 0, 0, 1), tf::Vector3 (0, 0, 0)),
                                              ros::Time::now (), "my_frame", msg.header.frame_id));


      //      std::cerr << "Model coefficients: " <<measurement(1)<< " " <<  measurement(2)<< " " <<  measurement(3)<< " " <<  mean(1)<< " " << mean(2)<< " " << mean(3)<< std::endl;
      //      std::cerr << "Model coefficients: " << mean(1)<< " " << mean(2)<< " " << mean(3)<< " " << mean(4)<< std::endl;
      //      std::cerr << "Model coefficients: " << cov(1,1)<< " " << cov(2,2)<< " " << cov(3,3)<< " " << cov(4,4)<< std::endl;



      pcl::toROSMsg (cloudXYZ_p_, cloud2_p_);

      // if (pub2_.getNumSubscribers () > 0)
      cloud2_p_.header.frame_id = "/my_frame";
      pub_.publish (msg);
      pub2_.publish (cloud2_p_);
      //       if (saveResult) {
      //        std::string file_name("test_pcd"); 
      //        std::stringstream ss;
      //        ss << image_counter++;
      //        
      //        file_name+=ss.str();
      //        file_name+=".pcd";
      //        pcl::io::savePCDFileASCII (file_name, cloud2);
      //        ROS_INFO ("Saved %d data points to %s.", (int)cloud.points.size (),file_name.c_str());
      //       }
    }
  }
}

int
main (int argc, char **argv)
{
  //bool saveResult=false;

  ros::init (argc, argv, "trackLeaf");
  ros::NodeHandle n ("trackLeaf");

  trackLeaf *tracker_ = new trackLeaf (n);

  ros::Subscriber sub = n.subscribe ("input", 2, &trackLeaf::trackLeafCallback, tracker_);

  ros::spin ();
  
  ROS_INFO ("Close camera connection");
  delete tracker_;
  return (0);
}
