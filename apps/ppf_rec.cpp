#include "pc_source/source.h"
#include "tools/render_views_tesselated_sphere.h"
#include "feature_wrapper/normal_estimator.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <utils/vtk_model_sampling.h>
#include <boost/function.hpp>
#include <vtkTransformPolyDataFilter.h>


#include <pcl/features/ppf.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ppf_registration.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


using namespace pcl;
using namespace std;

const Eigen::Vector4f subsampling_leaf_size (0.01f, 0.01f, 0.01f, 0.0f);
const float normal_estimation_search_radius = 0.05f;


PointCloud<PointNormal>::Ptr
subsampleAndCalculateNormals (PointCloud<PointXYZ>::Ptr cloud)
{
    PointCloud<PointXYZ>::Ptr cloud_subsampled (new PointCloud<PointXYZ> ());
    VoxelGrid<PointXYZ> subsampling_filter;
    subsampling_filter.setInputCloud (cloud);
    subsampling_filter.setLeafSize (subsampling_leaf_size);
    subsampling_filter.filter (*cloud_subsampled);

    PointCloud<Normal>::Ptr cloud_subsampled_normals (new PointCloud<Normal> ());
    NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
    normal_estimation_filter.setInputCloud (cloud_subsampled);
    search::KdTree<PointXYZ>::Ptr search_tree (new search::KdTree<PointXYZ>);
    normal_estimation_filter.setSearchMethod (search_tree);
    normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
    normal_estimation_filter.compute (*cloud_subsampled_normals);

    PointCloud<PointNormal>::Ptr cloud_subsampled_with_normals (new PointCloud<PointNormal> ());
    concatenateFields (*cloud_subsampled, *cloud_subsampled_normals, *cloud_subsampled_with_normals);

    PCL_INFO ("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size (), cloud_subsampled->points.size ());
    return cloud_subsampled_with_normals;
}

int
main (int argc, char** argv)
{
    if (argc != 3)
    {
        PCL_ERROR ("Syntax: ./ppf_object_recognition pcd_model_list pcd_scene\n");
        return -1;
    }

    ///////////////////////////////////////////////////////////////
    /// read point clouds from HDD
    PCL_INFO ("Reading scene ...\n");
    PointCloud<PointXYZ>::Ptr cloud_scene (new PointCloud<PointXYZ> ());
    PCDReader pcd_reader;
    pcd_reader.read (argv[2], *cloud_scene);
    PCL_INFO ("Scene read: %s\n", argv[2]);

    ///////////////////////////////////////////////////////////////
    // generate tesselated sphere views of the model
    // parameters
    int resolution_ = 256;
    bool useVertices_ = false;
    float radius_sphere_ = 1.5f;
    bool computeEntropies_ = true;
    int tes_level_ = 1;
    float view_angle_ = 57.0f;
    bool gen_organized_ = true;
    float model_scale_ = 0.001f;

    //load PLY model and scale it
    std::vector<PointCloud<PointXYZ>::Ptr> cloud_models;
    //vector<PointCloud<PointXYZ>::Ptr > cloud_models;
    std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> entropies;

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
    reader->SetFileName (argv[1]);

    vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New ();
    trans->Scale (model_scale_, model_scale_, model_scale_);
    trans->Modified ();
    trans->Update ();

    vtkSmartPointer<vtkTransformPolyDataFilter> filter_scale = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
    filter_scale->SetTransform (trans);
    filter_scale->SetInputConnection (reader->GetOutputPort ());
    filter_scale->Update ();

    vtkSmartPointer<vtkPolyData> mapper = filter_scale->GetOutput ();

    //generate views
    pcl::apps::RenderViewsTesselatedSphere render_views;
    render_views.setResolution (resolution_);
    render_views.setUseVertices (useVertices_);
    render_views.setRadiusSphere (radius_sphere_);
    render_views.setComputeEntropies (computeEntropies_);
    render_views.setTesselationLevel (tes_level_);
    render_views.setViewAngle (view_angle_);
    render_views.setGenOrganized (gen_organized_);

    render_views.addModelFromPolyData (mapper);
    render_views.generateViews ();

    render_views.getViews (cloud_models);
    render_views.getPoses (poses);
    render_views.getEntropies (entropies);

    std::vector<float>::iterator entropies_iter = entropies.begin();
    while(entropies_iter!=entropies.end())
    {
        if(*entropies_iter<0.3)
        {
            int step = entropies_iter-entropies.begin();
            cloud_models.erase(cloud_models.begin()+step);
            poses.erase(poses.begin()+step);
            entropies_iter = entropies.erase(entropies_iter);
        }
        else
            ++entropies_iter;
    }

    PCL_INFO("Finished generate models, models count: %d\n", cloud_models.size());

    ///////////////////////////////////////////////////////////////
    ///

    boost::shared_ptr<pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal> > normal_estimator;
    normal_estimator.reset (new pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal>);
    normal_estimator->setCMR (false);
    normal_estimator->setDoVoxelGrid (true);
    normal_estimator->setRemoveOutliers (true);
    normal_estimator->setValuesForCMRFalse (0.003f, 0.012f);

    //estimate (PointInTPtr & in, PointInTPtr & out, pcl::PointCloud<pcl::Normal>::Ptr & normals)

    PointCloud<PointXYZ>::Ptr cloud_scene_processed(new PointCloud<PointXYZ>);
    PointCloud<Normal>::Ptr cloud_scene_normal(new PointCloud<Normal>);
    normal_estimator->estimate(cloud_scene, cloud_scene_processed, cloud_scene_normal);
    std::cout << "processed scene size: " << cloud_scene_processed->points.size() << std::endl;

    PointCloud<PointNormal>::Ptr cloud_scene_input(new PointCloud<PointNormal>);
    concatenateFields (*cloud_scene_processed, *cloud_scene_normal, *cloud_scene_input);

    vector<PointCloud<PointNormal>::Ptr > cloud_models_with_normals;
    normal_estimator->setValuesForCMRFalse (0.015f, 0.045f);

    PCL_INFO ("Training models ...\n");
    vector<PPFHashMapSearch::Ptr> hashmap_search_vector;
    for (size_t model_i = 0; model_i < cloud_models.size (); ++model_i)
    {
        PointCloud<PointXYZ>::Ptr tmp_model = cloud_models[model_i];
        PointCloud<PointXYZ>::Ptr tmp_model_processed(new PointCloud<PointXYZ>);
        PointCloud<Normal>::Ptr tmp_model_normal(new PointCloud<Normal>);
        normal_estimator->estimate(tmp_model, tmp_model_processed, tmp_model_normal);

        PointCloud<PointNormal>::Ptr cloud_model_input(new PointCloud<PointNormal>); // = subsampleAndCalculateNormals (cloud_models[model_i]);
        concatenateFields (*tmp_model_processed, *tmp_model_normal, *cloud_model_input);
        cloud_models_with_normals.push_back (cloud_model_input);

        PointCloud<PPFSignature>::Ptr cloud_model_ppf (new PointCloud<PPFSignature> ());
        PPFEstimation<PointXYZ, Normal, PPFSignature> ppf_estimator;
        ppf_estimator.setRadiusSearch(0.0);
        ppf_estimator.setKSearch(50);
        ppf_estimator.setInputCloud (tmp_model_processed);
        ppf_estimator.setInputNormals (tmp_model_normal);
        ppf_estimator.compute (*cloud_model_ppf);

        PPFHashMapSearch::Ptr hashmap_search (new PPFHashMapSearch (12.0f / 180.0f * float (M_PI),
                                                                    0.05f));
        hashmap_search->setInputFeatureCloud (cloud_model_ppf);
        hashmap_search_vector.push_back (hashmap_search);
        std::cout << "model processed size: " << tmp_model_processed->points.size() << std::endl;
        std::cout << "model id " << model_i << " processed. "<< std::endl;
        tmp_model_processed->points.clear();
        tmp_model_processed->clear();
        tmp_model_normal->points.clear();
        tmp_model_normal->clear();
    }

    visualization::PCLVisualizer viewer ("PPF Object Recognition - Results");
    viewer.setBackgroundColor (0.2, 0.3, 0.3);
    viewer.addPointCloud (cloud_scene);
    viewer.spinOnce (10);
    PCL_INFO ("Registering models to scene ...\n");

    pcl::PLYWriter writer;
    for (size_t model_i = 0; model_i < cloud_models.size (); ++model_i)
    {

        PPFRegistration<PointNormal, PointNormal> ppf_registration;
        // set parameters for the PPF registration procedure
        ppf_registration.setSceneReferencePointSamplingRate (10);
        ppf_registration.setPositionClusteringThreshold (0.2f);
        ppf_registration.setRotationClusteringThreshold (30.0f / 180.0f * float (M_PI));
        ppf_registration.setSearchMethod (hashmap_search_vector[model_i]);
        ppf_registration.setInputSource (cloud_models_with_normals[model_i]);
        ppf_registration.setInputTarget (cloud_scene_input);

        PointCloud<PointNormal> cloud_output_subsampled;
        ppf_registration.align (cloud_output_subsampled);

        Eigen::Matrix4f mat = ppf_registration.getFinalTransformation ();
        Eigen::Affine3f final_transformation (mat);

        //  io::savePCDFileASCII ("output_subsampled_registered.pcd", cloud_output_subsampled);

        PointCloud<PointXYZ>::Ptr tmp_model = cloud_models[model_i];
        PointCloud<PointXYZ>::Ptr cloud_output (new PointCloud<PointXYZ> ());
        pcl::transformPointCloud (*tmp_model, *cloud_output, final_transformation);

        stringstream ss; ss << "model_" << model_i << ".ply";
        visualization::PointCloudColorHandlerRandom<PointXYZ> random_color (cloud_output->makeShared ());
        viewer.addPointCloud (cloud_output->makeShared(), random_color, ss.str ());

        //writer.write(ss.str().c_str(), *cloud_output);

        PCL_INFO ("Showing model %s\n", ss.str ().c_str ());
        cloud_output->points.clear();
        cloud_output->clear();
        cloud_output_subsampled.clear();
    }

    PCL_INFO ("All models have been registered!\n");


    while (!viewer.wasStopped ())
    {
        viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
