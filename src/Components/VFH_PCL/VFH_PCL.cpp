/*!
 * \file
 * \brief
 * \author Jan Figat
 */

#include <memory>
#include <string>

#include "VFH_PCL.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/io/pcd_io.h>


typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::VFHSignature308 DescriptorType;

namespace Processors {
namespace VFH_PCL {

VFH_PCL::VFH_PCL(const std::string & name) :
		Base::Component(name)  {

}

VFH_PCL::~VFH_PCL() {
}

void VFH_PCL::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register data streams
	registerStream("in_depth", &in_depth);
//	registerStream("in_pcl", &in_pcl);
//	registerStream("in_color", &in_color);
	registerStream("in_camera_info", &in_camera_info);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_normals", &out_normals);
	registerStream("out_keypoints", &out_keypoints);

	// Register handlers
	h_process_VFH.setup(boost::bind(&VFH_PCL::process_VFH, this));
	registerHandler("process_VFH", &h_process_VFH);
	addDependency("process_VFH", &in_depth);
	addDependency("process_VFH", &in_camera_info);
	//addDependency("process_VFH", &in_pcl);




}

bool VFH_PCL::onInit() {
	LOG(LTRACE) << "VFH_PCL::onInit\n";

	return true;
}

bool VFH_PCL::onFinish() {
	LOG(LTRACE) << "VFH_PCL::onFinish\n";
	return true;
}

bool VFH_PCL::onStop() {
	LOG(LTRACE) << "VFH_PCL::onStop\n";
	return true;
}

bool VFH_PCL::onStart() {
	LOG(LTRACE) << "VFH_PCL::onStart\n";
	return true;
}

void VFH_PCL::process_VFH() {
	LOG(LTRACE) << "VFH_PCL::process_VFH\n";

	//pcl::PointCloud<PointType>::Ptr cloud = in_pcl.read();

	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(camera_info.width(), camera_info.height()));

	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[0]);

	int row_step = depth.step1();
	for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud->width; ++u) {
			pcl::PointXYZ& pt = *pt_iter++;
			uint16_t depth = depth_row[u];

			// Missing points denoted by NaNs
			if (depth == 0) {
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}

			// Fill in XYZ
			pt.x = (u - cx_d) * depth * fx_d;
			pt.y = (v - cy_d) * depth * fy_d;
			pt.z = depth * 0.001;
		}
	}


	pcl::PointCloud<PointType>::Ptr keypoints (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<DescriptorType>::Ptr descriptors (new pcl::PointCloud<DescriptorType> ());

	float model_ss_ = 0.01f;
	float descr_rad_ = 0.02f;
	std::vector<int> aux_indices;

	pcl::removeNaNFromPointCloud (*cloud, *cloud, aux_indices); // removing wrong points from the cloud
	pcl::io::savePCDFileASCII ("/home/jfigat/DCL/PCL/my_cloud.pcd", *cloud);
	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	//pcl::NormalEstimation<PointType,NormalType> ne;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud (cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setKSearch (0); //[pcl::IntegralImagesNormalEstimation::compute] Both radius (0.500000) and K (1) defined! Set one of them to zero first and then re-run compute ().
	ne.setRadiusSearch (0.01); //1cm


	cout<<" compute normals"<<endl;
	ne.compute (*normals);
	pcl::io::savePCDFileASCII ("/home/jfigat/DCL/PCL/my_normal.pcd", *normals);
	cout<<" copy point from the cloud to the cloud with normals"<<endl;
	pcl::copyPointCloud (*cloud, *normals);
	out_normals.write(normals);
	out_cloud_xyz.write(cloud);
//	cout<<" clean normals"<<endl;

//	pcl::removeNaNFromPointCloud (*normals, *normals, aux_indices); // removing wrong points from the cloud with normals
//	pcl::removeNaNNormalsFromPointCloud (*normals, *normals, aux_indices); //removing wrong normals from the cloud with normals



	//cout<<" normals"<<endl;
	//ne.compute (*normals);


	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (normals);
	// alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		//		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_ (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
		//		vfh.setSearchMethod(tree_);
	//pcl::Search::Kdtree<pcl::PointXYZ>::ConstPtr tree_ (new pcl::Search::Kdtree<pcl::PointXYZ> ());
	//vfh.setSearchMethod(tree_);

	// Output datasets
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
	// Compute the features
	vfh.compute (*vfhs);

	pcl::io::savePCDFileASCII ("/home/jfigat/DCL/PCL/my_first_VFH.pcd", *vfhs);
	pcl::io::savePCDFileASCII ("my_first_VFH.pcd", *vfhs);


	//out_keypoints.write(*vfhs);  //nie działa
//	out_cloud_xyz.write(*vfhs);

/*
	  //indices
	  pcl::PointCloud<int> sampled_indices;
	  pcl::UniformSampling<PointType> uniform_sampling;
	  uniform_sampling.setInputCloud (cloud);
	  uniform_sampling.setRadiusSearch (model_ss_);
	  uniform_sampling.compute (sampled_indices);
	  pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
	  std::cout << "Model total points: " << cloud->size () << "; Selected Keypoints: " << keypoints->size () << std::endl;
	cout<<"4"<<endl;

	  //VFH
	  pcl::VFHEstimation<PointType, NormalType, DescriptorType> descr_est;
	  descr_est.setRadiusSearch (descr_rad_);
	cout<<"5"<<endl;
	  descr_est.setInputCloud (keypoints);
	  descr_est.setInputNormals (normals);
	  descr_est.setSearchSurface (cloud);
	  descr_est.compute (*descriptors);

	  pcl::io::savePCDFileASCII ("/home/jfigat/DCL/PCL/Deskryptory.pcd", *descriptors);

	  out_keypoints.write(keypoints);
*/

	/*Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(camera_info.width(), camera_info.height()));

//	cloud.width = camera_info.width();
//	cloud.height = camera_info.height();
//	cloud.points.resize(cloud.width * cloud.height);

	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[0]);

	int row_step = depth.step1();
	for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud->width; ++u) {
			pcl::PointXYZ& pt = *pt_iter++;
			uint16_t depth = depth_row[u];

			// Missing points denoted by NaNs
			if (depth == 0) {
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}

			// Fill in XYZ
			pt.x = (u - cx_d) * depth * fx_d;
			pt.y = (v - cy_d) * depth * fy_d;
			pt.z = depth * 0.001;
		}
	}
	out_cloud_xyz.write(cloud);
	pcl::io::savePCDFileASCII ("/CLOUD.pcd", *cloud);



	 * VFH

	//--load pcd from h.d.d
		pcl::io::loadPCDFile ("/CLOUD.pcd", *cloud);
		cloud->width = (int) cloud->points.size ();
		cloud->height = 1;

		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (cloud);
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZ> ());
		//ne.setSearchMethod (tree_normal);
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);

		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		ne.setRadiusSearch (0.03);
		ne.compute (*cloud_normals);

		// Create the VFH estimation class, and pass the input dataset+normals to it
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
		vfh.setInputCloud (cloud);
		vfh.setInputNormals (cloud_normals);
		// alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_ (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
//		vfh.setSearchMethod(tree_);
		pcl::Search::Kdtree<pcl::PointXYZ>::ConstPtr tree_ (new pcl::Search::Kdtree<pcl::PointXYZ> ());
		vfh.setSearchMethod(tree_);

		// Output datasets
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

		// Compute the features
		vfh.compute (*vfhs);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);//.makeShared()

	          // Create an empty kdtree representation, and pass it to the normal estimation object.
	          // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).

	          pcl::search::KdTree<pcl::PointXYZ>::Ptr cloud_normal_tree (new pcl::search::KdTree<pcl::PointXYZ>());
	          ne.setSearchMethod (cloud_normal_tree);

	          // Output datasets
	          pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	          // Use all neighbors in a sphere of radius 5cm
	          ne.setRadiusSearch (0.05);

	          // Compute the features
	          ne.compute (*normals);

	          // Create the VFH estimation class, and pass the input dataset+normals to it
	          pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	          vfh.setInputCloud (cloud);//.makeShared ()
	          vfh.setInputNormals (normals);
	          // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

	          // Create an empty kdtree representation, and pass it to the FPFH estimation object.
	          // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	          vfh.setSearchMethod (tree);

	          // Output datasets
	          pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

	          // Compute the features
	          vfh.compute (*vfhs);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

		... read, pass in or create a point cloud with normals ...
	  	  ... (note: you can create a single PointCloud<PointNormal> if you want) ...

	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (normals);
	// alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the VFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
	vfh.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	vfh.setRadiusSearch (0.05);

	// Compute the features
	vfh.compute (*vfhs);

	// pfhs->points.size () should have the same size as the input cloud->points.size ()*

	pcl::io::savePCDFileASCII ("my_first_VFH.pcd", *vfhs);


	//	out_features.write(*vfhs);
//	CLOG(LNOTICE) << "XYZ ";
	pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
	CLOG(LNOTICE) << "Saved " << cloud->points.size () << " data points to test_pcd.pcd.";*/
}




} //: namespace VFH_PCL
} //: namespace Processors
