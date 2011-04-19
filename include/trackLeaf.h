#ifndef TRACKLEAF_H
#define TRACKLEAF_H

#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include "pcl/ModelCoefficients.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
//CAMCUBE

//#include "camcube.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "sensor_msgs/point_cloud_conversion.h"
//#include <point_cloud_converter/conversion.h>

//plane segmentation
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
//filter
#include "pcl/filters/passthrough.h"
//extract indices
#include "pcl/filters/extract_indices.h"

#include "kfFilter.h"


//operacions amb matrius
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <Eigen3/Array>
// import most common Eigen types 
using namespace Eigen;

//    using namespace MatrixWrapper;
//   using namespace BFL;
//   using namespace std;

#include <cmath>

static const int PMD_INTEGRATION_TIME = 600;    ///600 by default (close objects)

/**
* segmenta una fulla com un pla a partir d'un punt x,y,z donat,
* fa tracking del model del pla de la fulla amb un KF
* observacions: es filtren els punts que queden lluny del model en t-1
*/
class trackLeaf
{
public:
  trackLeaf (ros::NodeHandle n);
  ~trackLeaf ();

    /**
    * inicialitza la camera i les variables necessaries
    * @TODO: la càmera ha de ser un node independent i nomes fer una subscripcio
    */
  void initialize ();

    /**
    * retorna el nuvol de punts de la càmera
    */
  void getData (sensor_msgs::PointCloud & cloud);

  void initializeFilterRegion ();
    /**
    * filtra els punts que es troben fora del quadrat central
    */
  void filterRegion (pcl::PointCloud < pcl::PointXYZ > cloud, pcl::PointCloud < pcl::PointXYZ > &cloud_filtered);

    /**
    * Receives the Point clouds and images, processes and publish
    */
  void trackLeafCallback (const sensor_msgs::PointCloud2 & msg);
private:
    /**
    * init the tracker by finding the initial plane coeficients
    * uses a small patch in the center of the image to determine the target
    * @todo use a point to determine the initial pose
    */
  void initTracker ();

    /**
    * Helper function for the observation
    * prune all points out of the container box
    */
  void segmentNeighbors (const pcl::PointCloud < pcl::PointXYZ > &input_cloudXYZ_,
                         pcl::PointCloud < pcl::PointXYZ > &output_cloudXYZ_);

  int integration_time_;
  bool calibration_on_;
  bool initialised_;            ///if the camera has been initialised should be cleanly closed
  bool firstLoop_;              ///to control the initialization of the filter
  //to handle camcuble
  //pmd_camcube::PmdCamcube * pmdCC_; 

  sensor_msgs::PointCloud2 cloud2_, cloud2_p_;
  sensor_msgs::PointCloud cloud_;
  pcl::PointCloud < pcl::PointXYZ > cloudXYZ_, cloudXYZ_filtered_, cloudXYZ_p_;

  //filter Region
  //Això és eficient? no es pot fer un filtre amb les tres condicions i no fer 3 passades diferents?
  pcl::PassThrough < pcl::PointXYZ > pass_;
  pcl::PassThrough < pcl::PointXYZ > pass2_;
  pcl::PassThrough < pcl::PointXYZ > pass3_;

  PoseEstimationFilter *poseFilter_;

  ros::Publisher pub_, pub2_, pub3_, pub4_;
// filters
  pcl::ModelCoefficients coefficients_;
  pcl::PointIndices inliers_;

  pcl::SACSegmentation < pcl::PointXYZ > seg_;

  // Create the filtering object
  pcl::ExtractIndices < pcl::PointXYZ > extract_;
  
  tf::Transform transform_, transformFilter_;

  Eigen::Vector3f box_size_; ///stores the size of the container box used to segment points with the plane coeficients
  Eigen::Vector3f box_center_;       ///the center point of the container box
  Eigen::Vector3f box_change_;       ///delta to add to the new boxed search space
  static const float MAX_X_BOX_SIZE = 0.05;
  static const float MAX_Y_BOX_SIZE = 0.05;
  static const float MAX_Z_BOX_SIZE = 0.02;
};

#endif
