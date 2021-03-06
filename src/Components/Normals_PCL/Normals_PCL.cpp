/*!
 * \file
 * \brief
 * \author Jan Figat
 */

#include <memory>
#include <string>

#include "Normals_PCL.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/io/pcd_io.h>


typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;

namespace Processors {
namespace Normals_PCL {

Normals_PCL::Normals_PCL(const std::string & name) :
		Base::Component(name)  {

}

Normals_PCL::~Normals_PCL() {
}

void Normals_PCL::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register data streams
	registerStream("in_depth", &in_depth);
	registerStream("in_color", &in_color);
	registerStream("in_camera_info", &in_camera_info);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_normals", &out_normals);

	// Register handlers
	h_process_Normals.setup(boost::bind(&Normals_PCL::process_Normals, this));
	registerHandler("process_Normals", &h_process_Normals);
	addDependency("process_Normals", &in_depth);
	addDependency("process_Normals", &in_color);
	addDependency("process_Normals", &in_camera_info);
}

bool Normals_PCL::onInit() {
	LOG(LTRACE) << "Normals_PCL::onInit\n";
	return true;
}

bool Normals_PCL::onFinish() {
	LOG(LTRACE) << "Normals_PCL::onFinish\n";
	return true;
}

bool Normals_PCL::onStop() {
	LOG(LTRACE) << "Normals_PCL::onStop\n";
	return true;
}

bool Normals_PCL::onStart() {
	LOG(LTRACE) << "Normals_PCL::onStart\n";
	return true;
}

void Normals_PCL::process_Normals() {
	LOG(LTRACE) << "Normals_PCL::process_Normals\n";

	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();
	cv::Mat color = in_color.read();

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
	ne.setKSearch (10); //Use 10 nearest neighbors
	ne.setRadiusSearch (0.0); // 0[cm] (0.0*10 [cm])

	ne.compute (*normals);
	CLOG(LTRACE) << "Normals computed\n";
	pcl::copyPointCloud(*cloud, *normals);

	//pcl::removeNaNFromPointCloud (*normals, *normals, aux_indices); // removing wrong points from the cloud with normals
    //    pcl::removeNaNNormalsFromPointCloud (*normals, *normals, aux_indices); //removing wrong normals from the cloud with normals
	      //[addPointCloudNormals] The number of points differs from the number of normals!

	pcl::io::savePCDFileASCII ("/home/jfigat/DCL/PCL/my_cloud_normals.pcd", *normals);
	pcl::copyPointCloud (*cloud, *normals); //copy point from the cloud to the cloud with normals
	out_normals.write(normals);
	out_cloud_xyz.write(cloud);
}


} //: namespace Normals_PCL
} //: namespace Processors
