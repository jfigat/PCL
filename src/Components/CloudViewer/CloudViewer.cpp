/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#include <memory>
#include <string>

#include "CloudViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/io/pcd_io.h>

namespace Processors {
namespace CloudViewer {

CloudViewer::CloudViewer(const std::string & name) :
		Base::Component(name), viewer(NULL)   {

}

CloudViewer::~CloudViewer() {
}

void CloudViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register data streams
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_normals", &in_cloud_normals);

	// Register handlers
	h_on_cloud_xyz.setup(boost::bind(&CloudViewer::on_cloud_xyz, this));
	registerHandler("on_cloud_xyz", &h_on_cloud_xyz);
	addDependency("on_cloud_xyz", &in_cloud_xyz);
	h_on_cloud_xyzrgb.setup(boost::bind(&CloudViewer::on_cloud_xyzrgb, this));
	registerHandler("on_cloud_xyzrgb", &h_on_cloud_xyzrgb);
	addDependency("on_cloud_xyzrgb", &in_cloud_xyzrgb);
	h_on_cloud_normals.setup(boost::bind(&CloudViewer::on_cloud_normals, this));
	registerHandler("on_cloud_normals", &h_on_cloud_normals);
	addDependency("on_cloud_normals", &in_cloud_normals);
	h_on_spin.setup(boost::bind(&CloudViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool CloudViewer::onInit() {

	/*viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");
    viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->spin();*/


	return true;
}

bool CloudViewer::onFinish() {
	return true;
}

bool CloudViewer::onStop() {
	return true;
}

bool CloudViewer::onStart() {
	return true;
}

void CloudViewer::on_cloud_xyz() {
	// -----Open 3D viewer and add point cloud-----
	static bool initd = false;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	//viewer->updatePointCloud<pcl::PointXYZ> (cloud, "sample cloud");

	if(!initd) {
			CLOG(LDEBUG) << "Init!";
			initd = true;
			CLOG(LDEBUG) << "Init!";
			viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");
			CLOG(LDEBUG) << "Init!";
			viewer->setBackgroundColor (1, 1, 0); // (0, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
			CLOG(LDEBUG) << "Init!";
			viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");
			//viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
			CLOG(LDEBUG) << "Init!";
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
			CLOG(LDEBUG) << "Init!";
			viewer->addCoordinateSystem (1.0,0);
			CLOG(LDEBUG) << "Init!";
			viewer->initCameraParameters ();
			CLOG(LDEBUG) << "Init!";
			//viewer->resetCameraViewpoint();
		} else {
			viewer->updatePointCloud<pcl::PointXYZ> (cloud, "cloud");

		}
}

void CloudViewer::on_cloud_xyzrgb() {
	// -----Open 3D viewer and add point cloud-----
	static bool initd = false;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB = in_cloud_xyzrgb.read();
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudRGB);

	if(!initd) {
		//CLOG(LDEBUG) << "Init!";
		initd = true;
		//CLOG(LDEBUG) << "Init!";
		viewer = new pcl::visualization::PCLVisualizer ("3D Viewer - RGB");
		//CLOG(LDEBUG) << "Init!";
		viewer->setBackgroundColor (1,1,1);//(0, 0, 0);
		//CLOG(LDEBUG) << "Init!";
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		//CLOG(LDEBUG) << "Init!";
		viewer->addPointCloud<pcl::PointXYZRGB> (cloudRGB, rgb, "cloud RGB");
		//CLOG(LDEBUG) << "Init!";
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud RGB");
		//CLOG(LDEBUG) << "Init!";
		viewer->addCoordinateSystem (1.0);
		//CLOG(LDEBUG) << "Init!";
		viewer->initCameraParameters ();
		//CLOG(LDEBUG) << "Init!";
	} else {
		viewer->updatePointCloud<pcl::PointXYZRGB> (cloudRGB, rgb, "cloud RGB");
	}
}

void CloudViewer::on_cloud_normals() {
	// -----Open 3D viewer and add point cloud and normals-----

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::Normal>::Ptr normals = in_cloud_normals.read();

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");

	  viewer->setBackgroundColor (0, 0, 0);
	  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();
	  //return (viewer);
	  viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	  //viewer->updatePointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");

}

void CloudViewer::on_spin() {
	if(viewer)viewer->spinOnce (1);
}



} //: namespace CloudViewer
} //: namespace Processors
