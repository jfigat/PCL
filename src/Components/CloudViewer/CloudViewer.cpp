/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 * modified by Jan Figat
 *
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
	registerStream("in_cloud_xyz_normals", &in_cloud_xyz_normals);

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
	h_on_cloud_xyz_normals.setup(boost::bind(&CloudViewer::on_cloud_xyz_normals, this));
	        registerHandler("on_cloud_xyz_normals", &h_on_cloud_xyz_normals);
	        addDependency("on_cloud_xyz_normals", &in_cloud_xyz_normals);
	h_on_spin.setup(boost::bind(&CloudViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool CloudViewer::onInit() {
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
  LOG(LTRACE) << "CloudViewer::on_cloud_xyz\n";
	// -----Open 3D viewer and add point cloud-----
	static bool initd = false;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	//viewer->updatePointCloud<pcl::PointXYZ> (cloud, "sample cloud");

	if(!initd) {
	    Eigen::Vector4f origin;
	    Eigen::Quaternionf orientation;
	    Eigen::Matrix3f rotation;
			CLOG(LDEBUG) << "Init!";
			initd = true;
			//CLOG(LDEBUG) << "Init!";
			viewer = new pcl::visualization::PCLVisualizer ("3D Cloud Viewer");
			//CLOG(LDEBUG) << "Init!";

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
			//CLOG(LDEBUG) << "Init!";
			viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");
			//viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
			//CLOG(LDEBUG) << "Init!";
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
			//CLOG(LDEBUG) << "Init!";
			viewer->addCoordinateSystem (1.0,0);
			//CLOG(LDEBUG) << "Init!";

			viewer->initCameraParameters ();
			//CLOG(LDEBUG) << "Init!";
			//viewer->setCameraPosition(0, 0, 0, 0, -1, 0); ///changing orientation
			                                                //setCameraPosition (double posX,double posY, double posZ, double viewX, double viewY, double viewZ);
			                                                //X (red), Y (green) and Z (blue) axes
			viewer->setCameraPosition(0, 0, -1, 0, -1, 0);//changing orientation
			viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");
		} else {
		        CLOG(LTRACE) << "Cloud XYZ - readed\n";
		        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
		        viewer->removePointCloud("cloud", 0); //removes cloud
		        viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");
			//viewer->updatePointCloud<pcl::PointXYZ> (cloud, "cloud");

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
		viewer->setBackgroundColor (0.0, 0.0, 0.5);//1,1,1);
		CLOG(LDEBUG) << "Init!";
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>::Ptr handler;// rgb(cloud);
		handler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (cloudRGB));
		//CLOG(LDEBUG) << "Init!";
		viewer->addPointCloud<pcl::PointXYZRGB> (cloudRGB, *handler /*rgb*/, "RGB cloud ");
		//CLOG(LDEBUG) << "Init!";
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud RGB");
		//CLOG(LDEBUG) << "Init!";
		viewer->addCoordinateSystem (1.0);
		//CLOG(LDEBUG) << "Init!";
		viewer->initCameraParameters ();
		CLOG(LDEBUG) << "Init!";
		viewer->setCameraPosition(0, 0, -1, 0, -1, 0);
	} else {
		viewer->updatePointCloud<pcl::PointXYZRGB> (cloudRGB, rgb, "cloud RGB");
	}
}

void CloudViewer::on_cloud_normals() {
  LOG(LTRACE) << "CloudViewer::on_cloud_normals\n";
	// -----Open 3D viewer and add point cloud and normals-----

          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
          pcl::PointCloud<pcl::Normal>::Ptr normals = in_cloud_normals.read();
          static bool initd = false;
          CLOG(LTRACE) << "Cloud with normals - readed\n";
          if(!initd) {
                      Eigen::Vector4f origin;
                      Eigen::Quaternionf orientation;
                      Eigen::Matrix3f rotation;
                                  CLOG(LDEBUG) << "Init!";
                                  initd = true;
                                  //CLOG(LDEBUG) << "Init!";
                                  viewer = new pcl::visualization::PCLVisualizer ("Normals Viewer");
                                  //CLOG(LDEBUG) << "Init!";

                                  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
                                  //CLOG(LDEBUG) << "Init!";
                                  // visualize normals
                                            viewer->setBackgroundColor (0.0, 0.0, 0.0);
                                            //viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
                                  //CLOG(LDEBUG) << "Init!";
                                  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
                                  //CLOG(LDEBUG) << "Init!";
                                  viewer->addCoordinateSystem (1.0,0);
                                  //CLOG(LDEBUG) << "Init!";

                                  viewer->initCameraParameters ();
                                  //CLOG(LDEBUG) << "Init!";
                                  //viewer->setCameraPosition(0, 0, 0, 0, -1, 0); ///changing orientation
                                                                                  //setCameraPosition (double posX,double posY, double posZ, double viewX, double viewY, double viewZ);
                                                                                  //X (red), Y (green) and Z (blue) axes
                                  viewer->setCameraPosition(0, 0, -1, 0, -1, 0);//changing orientation
                                  //viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud ");
                                  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud2");
                                  viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
                          } else {
                              if(!viewer->updatePointCloud<pcl::PointXYZ> (cloud, "cloud"))
                                {
                                    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
                                } else {
                                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
                                    viewer->removePointCloud("cloud", 0); //removePointCloud (const std::string &id="cloud", int viewport=0)
                                    //viewer->initCameraParameters (); ///
                                    //viewer->setCameraPosition(0, 0, -1, 0, -1, 0); ///
                                    viewer->updatePointCloud<pcl::PointXYZ> (cloud, single_color, "cloud2");
                                    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals); //(const pcl::PointCloud< PointT > &cloud, const pcl::PointCloud< PointNT > &normals, int level=100, double scale=0.02, const std::string &id="cloud", int viewport=0)
                                }

                              viewer->spinOnce(0, true);//(int time=1, bool force_redraw=false)


                          }



	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::Normal>::Ptr normals = in_cloud_normals.read();

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");

	  viewer->setBackgroundColor (0, 0, 0);
	  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.100.05, "normals");
	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();
	  //return (viewer);
	  viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	  //viewer->updatePointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");*/

}

void CloudViewer::on_cloud_xyz_normals() {
        // -----Open 3D viewer and add point cloud and normals-----

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
        pcl::PointCloud<pcl::Normal>::Ptr normals = in_cloud_normals.read();

        //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");

          viewer->setBackgroundColor (0, 0, 0);
          //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
          CLOG(LDEBUG) << "Init!";
          viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
          viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.10/*0.05*/, "normals");
          viewer->addCoordinateSystem (1.0);
          viewer->initCameraParameters ();
          //return (viewer);
          viewer->updatePointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
          //viewer->updatePointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");

}


void CloudViewer::on_spin() {
	if(viewer)viewer->spinOnce (1);
}



} //: namespace CloudViewer
} //: namespace Processors
