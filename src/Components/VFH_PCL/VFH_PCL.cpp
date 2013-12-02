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
#include <pcl/io/io.h>


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
        registerStream("in_cloud_xyz", &in_cloud_xyz);
        registerStream("in_cloud_normals", &in_cloud_normals);
	registerStream("out_keypoints", &out_keypoints);
	registerStream("out_features", &out_features);

	// Register handlers
	h_process_VFH.setup(boost::bind(&VFH_PCL::process_VFH, this));
	registerHandler("process_VFH", &h_process_VFH);
	addDependency("process_VFH", &in_cloud_xyz);
	addDependency("process_VFH", &in_cloud_normals);

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
	int hsize;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::Normal>::Ptr normals = in_cloud_normals.read();

	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (normals);
	// alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	vfh.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
	// Compute the features
	vfh.compute (*vfhs);
	CLOG(LTRACE) << "VFH computed\n";

	pcl::io::savePCDFileASCII ("/home/jfigat/DCL/PCL/VFH.pcd", *vfhs);

	out_features.write(vfhs);

	// vfhs->points.size () should be of size 1*

	//visualization
	pcl::visualization::PCLHistogramVisualizer histogram;
	hsize= cloud->points.size();//vfhs->points.size();
	histogram.addFeatureHistogram (*vfhs, hsize /*vfhs.points()*//*50000*/, "histogram", 640, 200);// (const pcl::PointCloud< PointT > &cloud, int hsize, const std::string &id="cloud", int win_width=640, int win_height=200)
	//histogram.spin();
	histogram.spinOnce(1000);
}


} //: namespace VFH_PCL
} //: namespace Processors
