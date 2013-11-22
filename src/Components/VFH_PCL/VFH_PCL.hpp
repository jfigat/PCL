/*!
 * \file
 * \brief
 * \author Jan Figat
 */

#ifndef VFH_PCL_HPP_
#define VFH_PCL_HPP_


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

#include <pcl/features/vfh.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <pcl/visualization/pcl_visualizer.h>


namespace Processors {
namespace VFH_PCL {

/*!
 * \class VFH_PCL
 * \brief VFH_PCL processor class.
 *
 * Conversion between depth map and pointcloud
 */
class VFH_PCL: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	VFH_PCL(const std::string & name = "VFH_PCL");

	/*!
	 * Destructor
	 */
	virtual ~VFH_PCL();

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
	Base::DataStreamOut<pcl::PointCloud<pcl::VFHSignature308>::Ptr > out_features;


	// Input data streams
	Base::DataStreamIn<cv::Mat> in_depth;
	Base::DataStreamIn<Types::CameraInfo> in_camera_info;
//	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_pcl;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_keypoints;
	Base::DataStreamOut<pcl::PointCloud<pcl::Normal>::Ptr> out_normals;

	// Handlers
	Base::EventHandler2 h_process_VFH;
	Base::EventHandler2 h_process_all;


	// Handlers
	void process_VFH();

};

} //: namespace VFH_PCL
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("VFH_PCL", Processors::VFH_PCL::VFH_PCL)

#endif /* VFH_PCL_HPP_ */
