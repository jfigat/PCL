/*!
 * \file
 * \brief
 * \author Jan Figat
 */

#ifndef Normals_PCL_HPP_
#define Normals_PCL_HPP_


#include "Component_Aux.hpp"
#include "Component.hpp"
//#include "Panel_Empty.hpp" ///
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"


#include <Types/CameraInfo.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>  // for removeNaNFromPointCloud
#include <pcl/features/integral_image_normal.h> //for IntegralImageNormalEstimation

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d.h> //for normal estimation
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <pcl/visualization/pcl_visualizer.h> //viewer


namespace Processors {
namespace Normals_PCL {

/*!
 * \class Normals_PCL
 * \brief Normals_PCL processor class.
 *
 * Conversion between depth map and pointcloud
 */
class Normals_PCL: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Normals_PCL(const std::string & name = "Normals_PCL");

	/*!
	 * Destructor
	 */
	virtual ~Normals_PCL();

	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	// Data streams
//	Base::DataStreamIn<cv::Mat> in_color;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr > out_cloud_xyz;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > out_cloud_xyzrgba;


	// Input data streams
	Base::DataStreamIn<cv::Mat> in_depth;
	Base::DataStreamIn<cv::Mat> in_color;
	Base::DataStreamIn<Types::CameraInfo> in_camera_info;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::Normal>::Ptr> out_normals;

	// Handlers
	Base::EventHandler2 h_process_Normals;

	// Handlers
	void process_Normals();

};

} //: namespace Normals_PCL
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Normals_PCL", Processors::Normals_PCL::Normals_PCL)

#endif /* Normals_PCL_HPP_ */
