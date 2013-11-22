/*!
 * \file
 * \brief 
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#ifndef CLOUDVIEWER_HPP_
#define CLOUDVIEWER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
//#include "Panel_Empty.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"


#include <pcl/visualization/pcl_visualizer.h>

namespace Processors {
namespace CloudViewer {

/*!
 * \class CloudViewer
 * \brief CloudViewer processor class.
 *
 * Pointcloud viewer with normals visualization
 */
class CloudViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudViewer(const std::string & name = "CloudViewer");

	/*!
	 * Destructor
	 */
	virtual ~CloudViewer();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
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
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz;
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;
	Base::DataStreamIn< pcl::PointCloud<pcl::Normal>::Ptr > in_cloud_normals;

	// Handlers
	Base::EventHandler2 h_on_cloud_xyz;
	Base::EventHandler2 h_on_cloud_xyzrgb;
	Base::EventHandler2 h_on_cloud_normals;
	Base::EventHandler2 h_on_spin;

	
	// Handlers
	void on_cloud_xyz();
	void on_cloud_xyzrgb();
	void on_cloud_normals();
	void on_spin();


	pcl::visualization::PCLVisualizer * viewer;
};

} //: namespace CloudViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudViewer", Processors::CloudViewer::CloudViewer)

#endif /* CLOUDVIEWER_HPP_ */
