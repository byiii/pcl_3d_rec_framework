#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <utils/readparameters.h>

// pre
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;


typedef pcl::FPFHSignature33 FeatureT;
//typedef pcl::SHOT352 FeatureT;

typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
//typedef pcl::SHOTEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;

typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  // parameter reader
  readParameters param_reader;
  param_reader.setConfigureFile("param.cfg");

  // Get input object and scene
  if (argc != 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }

  // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
    pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (1);
  }

  //------------------------------------------------------------
  // pre translation the object model
  bool pre_transform_p = false;
  param_reader.get<bool>("pre_transform_model_p", pre_transform_p);
  if(pre_transform_p)
  {
    Eigen::Vector4f scene_center(0.0f, 0.0f, 0.0f, 0.0f);
    pcl::compute3DCentroid(*scene, scene_center);

    // generate the rotation matrix:
    // define the rotation angle and rotation axis
    float rotation_angle = (float)0.0/180.0*M_PI;
    Eigen::Vector3f rotation_axis(0.0f, 0.0f, 1.0f);
    Eigen::AngleAxisf rotation_mg (rotation_angle, rotation_axis);

    // generate the whole tranform:
    // translate the model to the origin first, and then do the rotation.
    Eigen::Vector3f tmpVec3f(scene_center(0),
                  scene_center(1),
                  scene_center(2));
    //    tmpVec3f = tmpVec3f*0.09;
    // translate first, then rotation
    Eigen::Affine3f transform_mg (
      rotation_mg*Eigen::Translation3f((-1) * tmpVec3f));
    // transform the model cloud
    pcl::transformPointCloud(*object, *object, transform_mg);
  }
  //------------------------------------------------------------

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  float leaf = 0.003f;
  param_reader.get<float>("leaf_size", leaf);

  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);

  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;

  float normal_search_radius = 0.015;
  param_reader.get<float>("normal_search_radius", normal_search_radius);
  nest.setRadiusSearch (normal_search_radius);
  nest.setInputCloud (scene);
  nest.compute (*scene);

  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;

  float feature_search_radius=0.01;
  param_reader.get<float>("feature_search_radius", feature_search_radius);
  fest.setRadiusSearch (feature_search_radius);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;

  int number_of_samples = 4;
  int k_nearest_features = 3;
  float inlier_threshold = 0.5;
  param_reader.get<int>("sample_for_generating_a_pose", number_of_samples);
  param_reader.get<int>("k_nearest_features", k_nearest_features);
  param_reader.get<float>("inlier_threshold",inlier_threshold);

  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (number_of_samples); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (k_nearest_features); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (inlier_threshold); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }

  return (0);
}
