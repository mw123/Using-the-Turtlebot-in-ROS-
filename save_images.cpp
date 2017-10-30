/* This is a ROS Indigo node for saving Turtlebot images from the /camera/depth_registered/image_rect_raw
 * and /camera/rgb/color/image_rect_raw topics, for further processing outside of the program. 
 * Depth images are saved as 1 channel 16 bit unsigned int PNGs (CV_16UC1), and 
 * RGB images are saved as 3 channel 8 bit unsigned int PNGs (CV_8UC3).
 * A synchronized subscriber is used, for saving the pair of images that are most closer in time.
 * THE IMAGES ARE SAVED WHEREVER THE NODE IS RUN.
 * Created by Fabricio Emder and Pablo Aguado - 2016
 * Modified by Long Wang - 2017
 * Click "a" to capture a single rgb, depth, and masked rgb image set
 * Click "b" to automatically capture 300 images at 1.5 second intervals
 * */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>	// OpenCV
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>


 /* Exact sychronization policy IS NOT WORKING for the topics used, because the kinect
 * is not publishing them with the same timestamp. 
 * See more in http://wiki.ros.org/message_filters  */

//#define EXACT
#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif

 
 
using namespace std;
//using namespace sensor_msgs;
using namespace message_filters;


// Counter for filenames.
unsigned int cnt = 1;
char msgArrived = 0;



cv::Mat blendColorDepth(const cv::Mat& rgbImg, const cv::Mat& depthImg) {
    std::vector<cv::Mat> channels;
    cv::split(rgbImg, channels);

    // blend depth image into each channel
    channels[ 0 ] = (channels[ 0 ] * 0.5) +  ( depthImg * 0.5 );
    channels[ 1 ] = (channels[ 1 ] * 0.5) +  ( depthImg * 0.5 );
    channels[ 2 ] = (channels[ 2 ] * 0.5) +  ( depthImg * 0.5 );

    // merge to overlayed image
    cv::Mat blended;
    cv::merge( channels, blended ); 

    return blended;
}

// Handler / callback
void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{
	//ROS_INFO_STREAM("callback triggered\n");
	cv_bridge::CvImagePtr img_ptr_rgb;
	cv_bridge::CvImagePtr img_ptr_depth;

    msgArrived = 1;

        try{
            img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception:  %s", e.what());
            return;
        }
        try{
            img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
        }

        cv::Mat& mat_depth = img_ptr_depth->image;
		cv::Mat& mat_rgb = img_ptr_rgb->image;

        double min, max;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(mat_depth, &min, &max, &min_loc, &max_loc);

        //putText(mat_depth, format("maxLoc (%d,%d)", max_loc.x, max_loc.y), maxLoc,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 204), 1, CV_AA);
	    ROS_INFO("Max depth: %f at (%d %d) Min depth: %f at (%d %d)\n", max, max_loc.x, max_loc.y, min, min_loc.x, min_loc.y);

		char file_rgb[100];
		char file_depth[100];
		char file_rgbd[100];

		sprintf( file_rgb, "non_traversable%04d_rgb.png", cnt );
		sprintf( file_depth, "non_traversable%04d_depth.png", cnt );
		sprintf( file_rgbd, "non_traversable%04d_rgbd.png", cnt );
		
		vector<int> png_parameters;
		png_parameters.push_back( CV_IMWRITE_PNG_COMPRESSION );
		/* We save with highest compression.*/
		png_parameters.push_back( 9 ); 
		
		cv::imwrite( file_rgb , mat_rgb, png_parameters );
		//cv::imwrite( file_depth, mat_depth);
		cv::imwrite( file_depth, mat_depth, png_parameters );
		
		ROS_INFO_STREAM(cnt << "\n");
		ROS_INFO_STREAM("Images saved\n");
		
        mat_depth.convertTo(mat_depth, CV_8U);
        cv::Mat mat_rgbd=blendColorDepth(mat_rgb, mat_depth);
        //cv::Mat mat_rgbd;
        //mat_rgb.copyTo(mat_rgbd,mat_depth);
        cv::imwrite( file_rgbd, mat_rgbd, png_parameters );
        
		cnt++;
    
}



int main(int argc, char** argv)
{
	// Initialize the ROS system and become a node.
  ros::init(argc, argv, "save_images");
  ros::NodeHandle nh;
	
	
	message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/camera/depth_registered/hw_registered/image_rect_raw" , 1 );
	message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/camera/rgb/image_rect_color" , 1 );

	
#ifdef EXACT
	typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
#ifdef APPROXIMATE
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
	
	
  // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
  sync.registerCallback(boost::bind(&callback, _1, _2));

	
    char captureMode = 'i';
    while(ros::ok())
	{
        ROS_INFO_STREAM("\nEnter 'a' to save a pair of images or 'b' to automatically save 300 images\n");
	    cin.get(captureMode);
	    cin.ignore();
	    captureMode = tolower(captureMode);
	    ROS_INFO_STREAM("You entered " << captureMode << "\n");
        
        if( captureMode == 'a' )
		{
			/* We give control to the callback function.*/
            while(!msgArrived)  
			    ros::spinOnce();
            msgArrived = 0;
		}
		
		else if( captureMode == 'b' )
		{
			unsigned int cnt_init = cnt;
			while( cnt - cnt_init < 300 )
			{
				ros::spinOnce();
                ros::Duration(1.5).sleep();
			}
		}
		
		//else break;

	}
	ROS_INFO_STREAM("Closing node\n");

    return 0;
}
