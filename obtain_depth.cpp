/* Usage: rosrun save_images obtain_depth (rgb) -o PATH_TO_SAVE_IMAGES 
   If rgb specified, rgb images will be saved, otherwise, registered depth images are saved.
   Images save automatically at 1.5 second intervals
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <sstream>
#include <iomanip> //getfill
#include <dirent.h> //read directory
#include <sys/stat.h> //mkdir
#include <unistd.h> //get dir path

volatile int cnt = 0;
volatile int one_image_per_sec = 0;
volatile int imageType = 0;
std::string outFolder;
 
/*
void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
	if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
	  mono8_img = cv::Mat(float_img.size(), CV_8UC1);
	}
	cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}*/
void SaveImage(const cv::Mat& mat) {
    cv::Mat img2;
    
    if (!imageType) {
        double minVal, maxVal;
        //depthToCV8UC1(mat, img2);
        minMaxLoc(mat, &minVal, &maxVal,NULL,NULL); //find minimum and maximum intensities
        img2=mat;

    
        mat.convertTo(img2,CV_16UC1,65535.0/(maxVal - minVal), -minVal * 65535.0/(maxVal - minVal)); //2^16=65536
        cv::imshow("DepthImgScaled",img2);
        cv::waitKey(3);
    
        ROS_INFO_STREAM("Minimum depth captured:" << minVal);
        ROS_INFO_STREAM("Maximum depth captured:" << maxVal);

    }

    if (!one_image_per_sec) {
        std::string filename1;
        std::string filename2;
        std::ostringstream ss;
        char cwd[1024];
    	
        ss << std::setw(4) << std::setfill('0') << cnt/10;
        filename1 = "/frame" + ss.str() + ".jpg";
        if (!imageType) {
            filename2 = "/frame" + ss.str() + "_scaled.jpg";
        }

        ROS_INFO_STREAM("\nWriting to file" << filename1 << "\n");

        if (getcwd(cwd, sizeof(cwd)) != NULL) { //curr dir path
            std::string currDir(cwd, strlen(cwd)); //convert into c++ string
            if (outFolder != "none") {
                filename1 = currDir + "/" + outFolder + filename1;
                if (!imageType)
                    filename2 = currDir + "/" + outFolder + filename2;
            }
            else {
                filename1 = currDir + filename1;
                if (!imageType)
                    filename2 = currDir + filename2;
            }
        }
        else 
            ROS_INFO_STREAM("\ngetcwd() error\n");
/*
std::vector<int> p;
p[0] = CV_IMWRITE_PNG_COMPRESSION;
p[1] = 1; // compression factor*/	//LOS VECTORES NO SE TRABAJAN ASÍ. (sí se pueden acceder así).

        std::vector<int> pp;
        pp.push_back(CV_IMWRITE_JPEG_QUALITY);
        pp.push_back(100);


        imwrite(filename1,mat,pp);
        if (!imageType) {
            imwrite(filename2,img2,pp);
	        ROS_INFO_STREAM("Depth images saved\n");
        }
        else
	        ROS_INFO_STREAM("Color image saved\n");
    }

    cnt++;                               //each time an image comes increment cnt

    // Output modified video stream
    //ros::Publisher Pub = nh.publish(ddd->toImageMsg());

    one_image_per_sec++;
    if (one_image_per_sec == 10)
        one_image_per_sec = 0;
}

void ReturnColor(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr img_ptr;
    try{
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }
//ROS_INFO_STREAM("IM HERE\n");
    cv::Mat &mat = img_ptr->image;
    cv::imshow("ColorImg",mat);
    cv::waitKey(3);
    
    SaveImage(mat);
}

void ReturnDepth(const sensor_msgs::Image& msg) {
//ROS_INFO_STREAM("IM HERE\n");
    cv_bridge::CvImagePtr ddd;
    try{
        ddd = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    cv::Mat &mat = ddd->image;
    cv::imshow("DepthImgUnscaled", mat);

    SaveImage(mat);
}


int main(int argc, char **argv) {
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "obtain_depth");
    ros::NodeHandle nh;
    ros::Subscriber image_sub_;
    
    outFolder = "none";

    DIR *pdir = NULL;
    pdir = opendir (".");
    struct dirent *pent = NULL;
    int folderExists = 0;
//ROS_INFO_STREAM("\nargc: " << argc << "\n");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i],"rgb")==0) { 
            image_sub_ = nh.subscribe("/camera/rgb/image_color", 1, &ReturnColor);
            imageType = 1;

            ROS_INFO_STREAM("\nDisplaying rgb image\n");
        }
        else if (i + 1 != argc) {
            if (strcmp(argv[i],"-o")==0) { // specify folder name to save images
                outFolder = argv[i + 1];
                ROS_INFO_STREAM("\noutput folder name: " << argv[i+1] << "\n");

                // search for folder in curr dir. if doesn't exist, create new folder
                while (pent = readdir (pdir)) { //read all folders in current dir
                    if (pent == NULL) {
                        ROS_INFO_STREAM("\npent could not be initialized correctly WTF\n");
                        exit (1);
                    }
                    //ROS_INFO_STREAM("\ndirectory folder name: " << pent->d_name << "\n");
                    if (outFolder.compare(pent->d_name) == 0) {
                        folderExists = 1;
                    }
                }
                closedir (pdir);
                if (!folderExists) { // create folder if specified folder doesn't exist
                    const int dirErr = mkdir(argv[i + 1], S_IRWXU | S_IRWXG);
                    if (dirErr == -1) {
                        ROS_INFO_STREAM("\nError creating directory\n");
                        exit (2);
                    }
                }

                ROS_INFO_STREAM("Saving images to "<<outFolder<<"\n");
            }
        }
    }
    if (!imageType) { //by default displays depth image
        image_sub_ = nh.subscribe("/camera/depth_registered/image_raw", 1, ReturnDepth);
        ROS_INFO_STREAM("\nDisplaying depth image\n");
    }

    ros::Rate rate(10);

    if (!imageType) {
        cvNamedWindow("DepthImgUnscaled", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("DepthImgScaled", CV_WINDOW_AUTOSIZE);
    }
    else {
        cvNamedWindow("ColorImg", CV_WINDOW_AUTOSIZE);
    }

    cvStartWindowThread();
     
    while(ros::ok()) {

	    ros::spinOnce();

	//ROS_INFO("Hello, ROS!");

	    rate.sleep();

    }

    if (!imageType) {
        cvDestroyWindow("DepthImgUnscaled");
        cvDestroyWindow("DepthImgScaled");
    }
    else {
        cvDestroyWindow("ColorImg");
    }

    return 0;


}
