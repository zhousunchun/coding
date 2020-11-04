/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


namespace rtabmap_ros
{

class CameraModel
{
public:
	CameraModel();
	
	// minimal
	CameraModel(
		double fx,
		double fy,
		double cx,
		double cy)
        {
	        K_.at<double>(0,0) = fx;
	        K_.at<double>(1,1) = fy;
	        K_.at<double>(0,2) = cx;
	        K_.at<double>(1,2) = cy;
        }
	
	virtual ~CameraModel() {}
private:
	cv::Mat K_;
};

class PointCloudXYZ 
{
public:
	PointCloudXYZ(ros::NodeHandle& nh) :
		maxDepth_(0.0),
		minDepth_(0.0),
		voxelSize_(0.0),
		decimation_(1),
		noiseFilterRadius_(0.0),
		noiseFilterMinNeighbors_(5),
		normalK_(0),
		normalRadius_(0.0),
		filterNaNs_(false),
		approxSyncDepth_(0),
		approxSyncDisparity_(0),
		exactSyncDepth_(0),
		exactSyncDisparity_(0),
        nh_(nh),
        pnh_("~")
	{
        onInit();
    }

	virtual ~PointCloudXYZ()
	{
		if(approxSyncDepth_)
			delete approxSyncDepth_;
		if(approxSyncDisparity_)
			delete approxSyncDisparity_;
		if(exactSyncDepth_)
			delete exactSyncDepth_;
		if(exactSyncDisparity_)
			delete exactSyncDisparity_;
	}

private:
	virtual void onInit()
	{
		int queueSize = 10;
		bool approxSync = true;
		std::string roiStr;
		pnh_.param("approx_sync", approxSync, approxSync);
		pnh_.param("queue_size", queueSize, queueSize);
		pnh_.param("max_depth", maxDepth_, maxDepth_);
		pnh_.param("min_depth", minDepth_, minDepth_);
		pnh_.param("voxel_size", voxelSize_, voxelSize_);
		pnh_.param("decimation", decimation_, decimation_);
		pnh_.param("noise_filter_radius", noiseFilterRadius_, noiseFilterRadius_);
		pnh_.param("noise_filter_min_neighbors", noiseFilterMinNeighbors_, noiseFilterMinNeighbors_);
		pnh_.param("normal_k", normalK_, normalK_);
		pnh_.param("normal_radius", normalRadius_, normalRadius_);
		pnh_.param("filter_nans", filterNaNs_, filterNaNs_);
		pnh_.param("roi_ratios", roiStr, roiStr);

		// Deprecated
		if(pnh_.hasParam("cut_left"))
		{
			ROS_ERROR("\"cut_left\" parameter is replaced by \"roi_ratios\". It will be ignored.");
		}
		if(pnh_.hasParam("cut_right"))
		{
			ROS_ERROR("\"cut_right\" parameter is replaced by \"roi_ratios\". It will be ignored.");
		}
		if(pnh_.hasParam("special_filter_close_object"))
		{
			ROS_ERROR("\"special_filter_close_object\" parameter is removed. This kind of processing "
					  "should be done before or after this nodelet. See old implementation here: "
					  "https://github.com/introlab/rtabmap_ros/blob/f0026b071c7c54fbcc71df778dd7e17f52f78fc4/src/nodelets/point_cloud_xyz.cpp#L178-L201.");
		}

		//parse roi (region of interest)
		roiRatios_.resize(4, 0);
		if(!roiStr.empty())
		{
			std::list<std::string> strValues = uSplit(roiStr, ' ');
			if(strValues.size() != 4)
			{
				ROS_ERROR("The number of values must be 4 (\"roi_ratios\"=\"%s\")", roiStr.c_str());
			}
			else
			{
				std::vector<float> tmpValues(4);
				unsigned int i=0;
				for(std::list<std::string>::iterator jter = strValues.begin(); jter!=strValues.end(); ++jter)
				{
					tmpValues[i] = uStr2Float(*jter);
					++i;
				}

				if(tmpValues[0] >= 0 && tmpValues[0] < 1 && tmpValues[0] < 1.0f-tmpValues[1] &&
					tmpValues[1] >= 0 && tmpValues[1] < 1 && tmpValues[1] < 1.0f-tmpValues[0] &&
					tmpValues[2] >= 0 && tmpValues[2] < 1 && tmpValues[2] < 1.0f-tmpValues[3] &&
					tmpValues[3] >= 0 && tmpValues[3] < 1 && tmpValues[3] < 1.0f-tmpValues[2])
				{
					roiRatios_ = tmpValues;
				}
				else
				{
					ROS_ERROR("The roi ratios are not valid (\"roi_ratios\"=\"%s\")", roiStr.c_str());
				}
			}
		}

		ROS_INFO("Approximate time sync = %s", approxSync?"true":"false");

		if(approxSync)
		{
			approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageDepthSub_, cameraInfoSub_);
			approxSyncDepth_->registerCallback(boost::bind(&PointCloudXYZ::callback, this, _1, _2));
		}
		else
		{
			exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageDepthSub_, cameraInfoSub_);
			exactSyncDepth_->registerCallback(boost::bind(&PointCloudXYZ::callback, this, _1, _2));
		}

		ros::NodeHandle depth_nh(nh_, "depth");
		ros::NodeHandle depth_pnh(pnh_, "depth");
		image_transport::ImageTransport depth_it(depth_nh_);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(depth_nh, "camera_info", 1);

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
	}

	void callback(
			  const sensor_msgs::ImageConstPtr& depth,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)!=0 &&
		   depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0 &&
		   depth->encoding.compare(sensor_msgs::image_encodings::MONO16)!=0)
		{
			NODELET_ERROR("Input type depth=32FC1,16UC1,MONO16");
			return;
		}

		if(cloudPub_.getNumSubscribers())
		{
			ros::WallTime time = ros::WallTime::now();

			cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);
			cv::Rect roi = computeRoi(imageDepthPtr->image, roiRatios_);

			image_geometry::PinholeCameraModel model;
			model.fromCameraInfo(*cameraInfo);

			pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
			CameraModel m(
					model.fx(),
					model.fy(),
					model.cx()-roiRatios_[0]*double(imageDepthPtr->image.cols),
					model.cy()-roiRatios_[2]*double(imageDepthPtr->image.rows));

			pcl::IndicesPtr indices(new std::vector<int>);
			pclCloud = cloudFromDepth(
					cv::Mat(imageDepthPtr->image, roi),
					m,
					decimation_,
					maxDepth_,
					minDepth_,
					indices.get());
			processAndPublish(pclCloud, indices, depth->header);

			ROS_DEBUG("point_cloud_xyz from depth time = %f s", (ros::WallTime::now() - time).toSec());
		}
	}

	
	void processAndPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr & pclCloud, pcl::IndicesPtr & indices, const std_msgs::Header & header)
	{
		if(indices->size() && voxelSize_ > 0.0)
		{
			pclCloud = voxelize(pclCloud, indices, voxelSize_);
			pclCloud->is_dense = true;
		}

		// Do radius filtering after voxel filtering ( a lot faster)
		if(!pclCloud->empty() && (pclCloud->is_dense || !indices->empty()) && noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
		{
			if(pclCloud->is_dense)
			{
				indices = radiusFiltering(pclCloud, noiseFilterRadius_, noiseFilterMinNeighbors_);
			}
			else
			{
				indices = radiusFiltering(pclCloud, indices, noiseFilterRadius_, noiseFilterMinNeighbors_);
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*pclCloud, *indices, *tmp);
			pclCloud = tmp;
		}

		sensor_msgs::PointCloud2 rosCloud;
		if(!pclCloud->empty() && (pclCloud->is_dense || !indices->empty()) && (normalK_ > 0 || normalRadius_ > 0.0f))
		{
			//compute normals
			pcl::PointCloud<pcl::Normal>::Ptr normals = computeNormals(pclCloud, normalK_, normalRadius_);
			pcl::PointCloud<pcl::PointNormal>::Ptr pclCloudNormal(new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*pclCloud, *normals, *pclCloudNormal);
			if(filterNaNs_)
			{
				pclCloudNormal = removeNaNNormalsFromPointCloud(pclCloudNormal);
			}
			pcl::toROSMsg(*pclCloudNormal, rosCloud);
		}
		else
		{
			if(filterNaNs_ && !pclCloud->is_dense)
			{
				pclCloud = removeNaNFromPointCloud(pclCloud);
			}
			pcl::toROSMsg(*pclCloud, rosCloud);
		}
		rosCloud.header.stamp = header.stamp;
		rosCloud.header.frame_id = header.frame_id;

		//publish the message
		cloudPub_.publish(rosCloud);
	}

   cv::Rect computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios)
   {
	   return computeRoi(image.size(), roiRatios);
   }

   cv::Rect computeRoi(const cv::Size & imageSize, const std::vector<float> & roiRatios)
   {
	    if(imageSize.height!=0 && imageSize.width!= 0 && roiRatios.size() == 4)
	    {
		    float width = imageSize.width;
		    float height = imageSize.height;
		    cv::Rect roi(0, 0, width, height);
		    ROS_DEBUG("roi ratios = %f, %f, %f, %f", roiRatios[0],roiRatios[1],roiRatios[2],roiRatios[3]);
		    ROS_DEBUG("roi = %d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);

		    //left roi
		    if(roiRatios[0] > 0 && roiRatios[0] < 1.0f - roiRatios[1])
		    {
			    roi.x = width * roiRatios[0];
		    }

		    //right roi
		    if(roiRatios[1] > 0 && roiRatios[1] < 1.0f - roiRatios[0])
		    {
			    roi.width -= width * roiRatios[1];
		    }
		    roi.width -= roi.x;

		    //top roi
		    if(roiRatios[2] > 0 && roiRatios[2] < 1.0f - roiRatios[3])
		    {
			    roi.y = height * roiRatios[2];
		    }

		    //bottom roi
		    if(roiRatios[3] > 0 && roiRatios[3] < 1.0f - roiRatios[2])
		    {
			    roi.height -= height * roiRatios[3];
		    }
		    roi.height -= roi.y;
		    ROS_DEBUG("roi = %d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);

		    return roi;
	    }
	    else
	    {
		    ROS_ERROR("Image is null or _roiRatios(=%d) != 4", roiRatios.size());
		    return cv::Rect();
	    }
    }

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
    {
	    CameraModel model(fx, fy, cx, cy);
	    return cloudFromDepth(imageDepth, model, decimation, maxDepth, minDepth, validIndices);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
		const cv::Mat & imageDepthIn,
		const CameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
    {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    if(decimation == 0)
	    {
		    decimation = 1;
	    }
	    float rgbToDepthFactorX = 1.0f;
	    float rgbToDepthFactorY = 1.0f;

	    cv::Mat imageDepth = imageDepthIn;
		decimation = abs(decimation);
	

    	//cloud.header = cameraInfo.header;
	    cloud->height = imageDepth.rows/decimation;
	    cloud->width  = imageDepth.cols/decimation;
	    cloud->is_dense = false;
	    cloud->resize(cloud->height * cloud->width);
	    if(validIndices)
	    {
		    validIndices->resize(cloud->size());
	    }

	    float depthFx = model.fx() * rgbToDepthFactorX;
	    float depthFy = model.fy() * rgbToDepthFactorY;
	    float depthCx = model.cx() * rgbToDepthFactorX;
	    float depthCy = model.cy() * rgbToDepthFactorY;

	    int oi = 0;
	    for(int h = 0; h < imageDepth.rows && h/decimation < (int)cloud->height; h+=decimation)
	    {
		    for(int w = 0; w < imageDepth.cols && w/decimation < (int)cloud->width; w+=decimation)
		    {   
			    pcl::PointXYZ & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

			    pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, depthCx, depthCy, depthFx, depthFy, false, 0.02f);
			    if(isFinite(ptXYZ) && ptXYZ.z>=minDepth && (maxDepth<=0.0f || ptXYZ.z <= maxDepth))
			    {
				    pt.x = ptXYZ.x;
				    pt.y = ptXYZ.y;
				    pt.z = ptXYZ.z;
				    if(validIndices)
				    {
					    validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
				    }
			    }
			    else
			    {
				    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
			    }
		    }
	    }

	    if(validIndices)
	    {
		    validIndices->resize(oi);
	    }

	    return cloud;
    }

    pcl::PointXYZ projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float depthErrorRatio)
    {
	
	    pcl::PointXYZ pt;

	    float depth = getDepth(depthImage, x, y, smoothing, depthErrorRatio);
	    if(depth > 0.0f)
	    {
		    // Use correct principal point from calibration
		    cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
		    cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)

		    // Fill in XYZ
		    pt.x = (x - cx) * depth / fx;
		    pt.y = (y - cy) * depth / fy;
		    pt.z = depth;
	    }
	    else
	    {
		    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
	    }
	    return pt;
    }

    float getDepth(
		const cv::Mat & depthImage,
		float x, float y,
		bool smoothing,
		float depthErrorRatio)
    {
	
	    int u = int(x+0.5f);
	    int v = int(y+0.5f);
	    if(u == depthImage.cols && x<float(depthImage.cols))
	    {
		    u = depthImage.cols - 1;
	    }
	    if(v == depthImage.rows && y<float(depthImage.rows))
	    {
		    v = depthImage.rows - 1;
	    }

	    if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
	    {
		    return 0;
	    }

	    bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

	    // Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
	    // https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
	    // Window weights:
	    //  | 1 | 2 | 1 |
	    //  | 2 | 4 | 2 |
	    //  | 1 | 2 | 1 |
	    int u_start = std::max(u-1, 0);
	    int v_start = std::max(v-1, 0);
	    int u_end = std::min(u+1, depthImage.cols-1);
	    int v_end = std::min(v+1, depthImage.rows-1);

	    float depth = 0.0f;
	    if(isInMM)
	    {
		    if(depthImage.at<unsigned short>(v,u) > 0 &&
		    depthImage.at<unsigned short>(v,u) < std::numeric_limits<unsigned short>::max())
		    {
			    depth = float(depthImage.at<unsigned short>(v,u))*0.001f;
		    }
	    }
	    else
	    {
		    depth = depthImage.at<float>(v,u);
	    }

	    if(depth!=0.0f && std::isfinite(depth))
	    {
		    if(smoothing)
		    {
			    float sumWeights = 0.0f;
			    float sumDepths = 0.0f;
			    for(int uu = u_start; uu <= u_end; ++uu)
			    {
				    for(int vv = v_start; vv <= v_end; ++vv)
				    {
					    if(!(uu == u && vv == v))
					    {
						    float d = 0.0f;
						    if(isInMM)
						    {
							    if(depthImage.at<unsigned short>(vv,uu) > 0 &&
							    depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
							    {
								    d = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
							    }
						    }
						    else
						    {
							    d = depthImage.at<float>(vv,uu);
						    }

						    float depthError = depthErrorRatio * depth;

						    // ignore if not valid or depth difference is too high
						    if(d != 0.0f && std::isfinite(d) && fabs(d - depth) < depthError)
						    {
							    if(uu == u || vv == v)
							    {
							    	sumWeights+=2.0f;
								    d*=2.0f;
							    }
							    else
							    {
								    sumWeights+=1.0f;
							    }
							    sumDepths += d;
						    }
					    }
				    }
			    }
			    // set window weight to center point
			    depth *= 4.0f;
			    sumWeights += 4.0f;

			    // mean
			    depth = (depth+sumDepths)/sumWeights;
		    }
	    }
	    else
	    {
		    depth = 0;
	    }
	    return depth;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const pcl::IndicesPtr & indices, float voxelSize)
    {
	    return voxelizeImpl<pcl::PointXYZ>(cloud, indices, voxelSize);
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr voxelizeImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float voxelSize)
    {
	    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	    if((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()))
	    {
		    pcl::VoxelGrid<PointT> filter;
		    filter.setLeafSize(voxelSize, voxelSize, voxelSize);
		    filter.setInputCloud(cloud);
		    if(indices->size())
		    {
			    filter.setIndices(indices);
		    }
		    filter.filter(*output);
	    }
	    else if(cloud->size() && !cloud->is_dense && indices->size() == 0)
	    {
		    ROS_WARN("Cannot voxelize a not dense (organized) cloud with empty indices! (input=%d pts). Returning empty cloud!", (int)cloud->size());
	    }
	    return output;
    }

    pcl::IndicesPtr radiusFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float radiusSearch, int minNeighborsInRadius)
    {
	    pcl::IndicesPtr indices(new std::vector<int>);
	    return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
    }

    template<typename PointT>
    pcl::IndicesPtr radiusFilteringImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float radiusSearch,
		int minNeighborsInRadius)
    {
	    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>(false));

	    if(indices->size())
	    {
		    pcl::IndicesPtr output(new std::vector<int>(indices->size()));
		    int oi = 0; // output iterator
		    tree->setInputCloud(cloud, indices);
		    for(unsigned int i=0; i<indices->size(); ++i)
		    {
			    std::vector<int> kIndices;
			    std::vector<float> kDistances;
			    int k = tree->radiusSearch(cloud->at(indices->at(i)), radiusSearch, kIndices, kDistances);
			    if(k > minNeighborsInRadius)
			    {
				    output->at(oi++) = indices->at(i);
			    }
		    }
		    output->resize(oi);
		    return output;
	    }
	    else
	    {
		    pcl::IndicesPtr output(new std::vector<int>(cloud->size()));
		    int oi = 0; // output iterator
		    tree->setInputCloud(cloud);
		    for(unsigned int i=0; i<cloud->size(); ++i)
		    {
			    std::vector<int> kIndices;
			    std::vector<float> kDistances;
			    int k = tree->radiusSearch(cloud->at(i), radiusSearch, kIndices, kDistances);
			    if(k > minNeighborsInRadius)
			    {
				    output->at(oi++) = i;
			    }
		    }
		    output->resize(oi);
		    return output;
	    }
    }

    pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
    {
	    return computeNormalsImpl<pcl::PointXYZ>(cloud, indices, searchK, searchRadius, viewPoint);
    }

    pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
    {
	    pcl::IndicesPtr indices(new std::vector<int>);
	    return computeNormals(cloud, indices, searchK, searchRadius, viewPoint);
    }

    template<typename PointT>
    pcl::PointCloud<pcl::Normal>::Ptr computeNormalsImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK,
		float searchRadius,
		const Eigen::Vector3f & viewPoint)
    {
	    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	    if(indices->size())
	    {
		    tree->setInputCloud(cloud, indices);
	    }
	    else
	    {
		    tree->setInputCloud (cloud);
	    }

    	pcl::NormalEstimation<PointT, pcl::Normal> n;
	    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	    n.setInputCloud (cloud);
	    // Commented: Keep the output normals size the same as the input cloud
	    //if(indices->size())
	    //{
	    //	n.setIndices(indices);
	    //}
	    n.setSearchMethod (tree);
	    n.setKSearch (searchK);
	    n.setRadiusSearch (searchRadius);
	    n.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
	    n.compute (*normals);

	    return normals;
    }
 
    pcl::PointCloud<pcl::PointNormal>::Ptr removeNaNNormalsFromPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud)
    {
	    return removeNaNNormalsFromPointCloudImpl<pcl::PointNormal>(cloud);
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr removeNaNNormalsFromPointCloudImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud)
    {
	    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	    std::vector<int> indices;
	    pcl::removeNaNNormalsFromPointCloud(*cloud, *output, indices);
	    return output;
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr removeNaNFromPointCloudImpl(
		const typename pcl::PointCloud<PointT>::Ptr & cloud)
    {
	    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	    std::vector<int> indices;
	    pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	    return output;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeNaNFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
    {
	    return removeNaNFromPointCloudImpl<pcl::PointXYZ>(cloud);
    }

    bool isFinite(const cv::Point3f & pt)
    {
	    return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
    }
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
	double maxDepth_;
	double minDepth_;
	double voxelSize_;
	int decimation_;
	double noiseFilterRadius_;
	int noiseFilterMinNeighbors_;
	int normalK_;
	double normalRadius_;
	bool filterNaNs_;
	std::vector<float> roiRatios_;

	ros::Publisher cloudPub_;

	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;
};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_xyz");
    ros::NodeHandle nh("/point_cloud_xyz");
    rtabmap_ros::PointCloudXYZ pointCloudXYZ(nh);
    ros::spin();
    return 0;
}

