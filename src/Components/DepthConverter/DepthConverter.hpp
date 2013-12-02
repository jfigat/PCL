/*!
 * \file
 * \brief 
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#ifndef DEPTHCONVERTER_HPP_
#define DEPTHCONVERTER_HPP_


#include "Component_Aux.hpp"
#include "Component.hpp"
//#include "Panel_Empty.hpp" ///
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"


#include <Types/CameraInfo.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace DepthConverter {

/*!
 * \class DepthConverter
 * \brief DepthConverter processor class.
 *
 * Conversion between depth map and pointcloud
 */
class DepthConverter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	DepthConverter(const std::string & name = "DepthConverter");

	/*!
	 * Destructor
	 */
	virtual ~DepthConverter();

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
	Base::DataStreamIn<cv::Mat> in_depth;
	Base::DataStreamIn<cv::Mat> in_color;
	Base::DataStreamIn<Types::CameraInfo> in_camera_info;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr > out_cloud_xyz;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;

	// Handlers
	Base::EventHandler2 h_process_all;
	Base::EventHandler2 h_process_depth;
	void process_depth();
	void process_all();

};

} //: namespace DepthConverter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DepthConverter", Processors::DepthConverter::DepthConverter)

#endif /* DEPTHCONVERTER_HPP_ */
