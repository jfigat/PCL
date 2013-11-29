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
typedef pcl::VFHSignature308 DescriptorType;

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

	//pcl::PointCloud<PointType>::Ptr cloud = in_pcl.read();

	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();
	cv::Mat color = in_color.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(camera_info.width(), camera_info.height()));
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>(camera_info.width(), camera_info.height()));

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
	CLOG(LTRACE) << "Cloud readed\n";

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
	ne.setKSearch (20); //Use 20 nearest neighbors
	ne.setRadiusSearch (0);
	//ne.setKSearch (0); //[pcl::IntegralImagesNormalEstimation::compute] Both radius (0.500000) and K (1) defined! Set one of them to zero first and then re-run compute ().
	//ne.setRadiusSearch (1);//0.10); //10cm

	ne.compute (*normals);
	CLOG(LTRACE) << "Normals computed\n";
	pcl::copyPointCloud(*cloud, *normals);

	//pcl::removeNaNFromPointCloud (*normals, *normals, aux_indices); // removing wrong points from the cloud with normals
    //    pcl::removeNaNNormalsFromPointCloud (*normals, *normals, aux_indices); //removing wrong normals from the cloud with normals
	      //[addPointCloudNormals] The number of points differs from the number of normals!

       // CLOG(LNOTICE) <<" compute normals"<<normals->points.size ()<<" test";

	pcl::io::savePCDFileASCII ("/home/jfigat/DCL/PCL/my_normal.pcd", *normals);
//	cout<<" copy point from the cloud to the cloud with normals"<<endl;
//	pcl::copyPointCloud (*cloud, *normals);
	out_normals.write(normals);
	out_cloud_xyz.write(cloud);
	CLOG(LTRACE) << "Cloud written\n";




        /*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cloud_rgb->width = 1; //Dimensions must be initialized to use 2-D indexing
        cloud_rgb->height = 1;
        cloud_rgb->resize(cloud_rgb->width*cloud_rgb->height);

        pcl::PointXYZRGBA p = pcl::PointXYZRGBA();
        p.b = 132;
        p.g = 232;
        p.r = 164;
        p.x = 0;
        p.y = 0;
        p.z = 5;

        cloud_rgb->push_back(p);

        pcl::PointCloud<pcl::Normal>::Ptr normals_rgb (new pcl::PointCloud<pcl::Normal>); //Output normals
        //Create search tree
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBA>);

        //Estimate
        pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
        normal_estimator.setInputCloud (cloud_rgb);
        normal_estimator.setSearchMethod (tree2);
        normal_estimator.setKSearch (20); //Use 20 nearest neighbors
        normal_estimator.compute (*normals_rgb);

        pcl::visualization::PCLVisualizer pclviewer("PCL Viewer");
        pclviewer.setBackgroundColor (0.0, 0.0, 0.5);
        pclviewer.addPointCloud<pcl::PointXYZRGBA>(cloud_rgb);

        pclviewer.addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloud_rgb, normals_rgb);

	out_cloud_xyzrgba.write(cloud_rgb);*/

}




} //: namespace Normals_PCL
} //: namespace Processors
