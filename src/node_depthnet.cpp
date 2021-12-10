//
// Created by Devon Super on 6/7/22.
//
#include "ros/ros.h"

#include <jetson-inference/depthNet.h>

#include "ros_compat.h"
#include "image_converter.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//globals
depthNet* net = NULL;

imageConverter* input_cvt = NULL;
imageConverter* visualization_cvt = NULL;
imageConverter* depth_cvt = NULL;

Publisher<sensor_msgs::Image> depth_pub = NULL;
Publisher<sensor_msgs::Image> visualization_pub = NULL;

int batchsize = 1;
bool visualize = false;
bool measure_time = true;

//measuring performance
int imagecount = 0;
//in milliseconds
double preprocesstime = 0.0;
double processtime = 0.0;
double publishtime = 0.0;

//from https://answers.ros.org/question/166286/measure-codenode-running-time/
//used by startTimer() and stopTimer()
ros::WallTime start_, end_;

//sets value of time. user stopTimer() to retrieve elapsed time.
void startTimer(){
    start_ = ros::WallTime::now();
}

int stopTimer(){
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;

    return execution_time;
}



//publish raw depth data
void publish_depths(){

    float* depths = net->GetDepthField();
    int height = net->GetDepthFieldHeight();
    int width = net->GetDepthFieldWidth();

    uint8_t *uint8_depth_pointer    = reinterpret_cast<uint8_t*>((float*)depths); //convert float data from model to ints

    //from https://gist.github.com/knorth55/007fab438d0c790b47c9de4ed76a9c29 and https://answers.ros.org/question/195979/creating-sensor_msgsimage-from-scratch/
    sensor_msgs::Image msg;
    msg.header.stamp     = ros::Time::now();
    msg.height           = height;
    msg.width            = width;
    msg.encoding         = "32FC1";
    msg.is_bigendian     = false;
    msg.step             = width * 4; // 32 bit float turns into 4 8 bit ints

    msg.data.clear();

    msg.data = std::vector<uint8_t>(uint8_depth_pointer, uint8_depth_pointer + (width * height * 4));

    depth_pub->publish(msg);
}



//     //from https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros
//     sensor_msgs::Image msg;

//     cv::Mat img = cv::Mat(height, width, CV_32F, &depths); //from https://stackoverflow.com/questions/22739320/how-can-i-initialize-a-cvmat-with-data-from-a-float-array
//     cv_bridge::CvImage img_bridge;

//     img_bridge = cv_bridge::CvImage(std_msgs::Header(), "32FC1", img);
//     img_bridge.toImageMsg(msg);

// 	// populate timestamp in header field
// 	msg.header.stamp = ROS_TIME_NOW();

// 	// publish the message
// 	depth_pub->publish(msg);
// }

// publish visualization
bool publish_visualization( uint32_t width, uint32_t height )
{
	// assure correct image size
	if( !visualization_cvt->Resize(width, height, imageConverter::ROSOutputFormat) )
		return false;

	// generate the overlay
	if( !net->Visualize(visualization_cvt->ImageGPU(), width, height) )
		return false;

	// populate the message
	sensor_msgs::Image msg;

	if( !visualization_cvt->Convert(msg, imageConverter::ROSOutputFormat) )
		return false;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message
	visualization_pub->publish(msg);
}

// input image subscriber callback
void img_callback( const sensor_msgs::ImageConstPtr input )
{
    imagecount++;

    startTimer();
    // convert the image to reside on GPU
    if( !input_cvt || !input_cvt->Convert(input) )
    {
        ROS_INFO("failed to convert %ux%u %s image", net->GetDepthFieldWidth(), net->GetDepthFieldHeight(), input->encoding.c_str());
        return;
    }
    preprocesstime += stopTimer();

    startTimer();

    ROS_INFO("Processing image %u", imagecount);

    // process the depth network
    if( !net->Process(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight()) )
    {
        ROS_ERROR("failed to process depth perception on %ux%u image", input->width, input->height);
        return;
    }
    processtime += stopTimer();

    if( ROS_NUM_SUBSCRIBERS(depth_pub) > 0 ){
        ROS_INFO("publishing image %u", imagecount);
        
        startTimer();
        publish_depths();
        publishtime += stopTimer();


        if(measure_time){
            float avgppt = (preprocesstime/imagecount);
            float avgpt = (processtime/imagecount);
            float avgput = (publishtime/imagecount);

            ROS_INFO("      average preprocessing time: %fms", avgppt);
            ROS_INFO("      average processing time: %fms", avgpt);
            ROS_INFO("      average publish time: %fms", avgput);
        }
    }

    if( ROS_NUM_SUBSCRIBERS(visualization_pub) > 0  && visualize){
        publish_visualization(input_cvt->GetWidth(), input_cvt->GetHeight());
    }

}


// node main loop
int main(int argc, char **argv)
{
    //create node instance
    ROS_CREATE_NODE("depthnet");

    ROS_INFO("initializing depthnet");

    //declare parameters
    std::string model_path;
    std::string precisionstring;

    std::string input_blob = "input";
    std::string output_blob = "output"; 


    ROS_DECLARE_PARAMETER("model_path", model_path);
    ROS_DECLARE_PARAMETER("input_blob", input_blob);
    ROS_DECLARE_PARAMETER("output_blob", output_blob);
    ROS_DECLARE_PARAMETER("precision", precisionstring);
    ROS_DECLARE_PARAMETER("visualize", visualize);
    ROS_DECLARE_PARAMETER("measure_time", measure_time);

    //retrieve parameters
    ROS_GET_PARAMETER("model_path", model_path);
    ROS_GET_PARAMETER("input_blob", input_blob);
    ROS_GET_PARAMETER("output_blob", output_blob);
    ROS_GET_PARAMETER("precision", precisionstring);
    ROS_GET_PARAMETER("visualize", visualize);
    ROS_GET_PARAMETER("measure_time", measure_time);


    if(model_path.size() > 0){

        precisionType precision = precisionTypeFromStr(precisionstring.c_str());

        net = depthNet::Create(model_path.c_str(),
                             input_blob.c_str(), output_blob.c_str(), 1, precision);
    }else{ //TODO create way to load model besides from path
        ROS_ERROR("No model path provided");
        return 0;
    }

    if( !net )
    {
        ROS_ERROR("failed to load monodepth2 model");
        return 0;
    }

    input_cvt              = new imageConverter();
    visualization_cvt      = new imageConverter();
    depth_cvt              = new imageConverter();

	if( !input_cvt || !visualization_cvt || !depth_cvt )
	{
		ROS_ERROR("failed to create imageConverter objects");
		return 0;
	}

    //announce publishing topics
    ROS_CREATE_PUBLISHER(sensor_msgs::Image, "depth", 2000, depth_pub); //TODO find a good queue size.
    ROS_CREATE_PUBLISHER(sensor_msgs::Image, "visualization", 30, visualization_pub);


    /*
      * subscribe to image topic
      */
    auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 120, img_callback);

    /*
	 * wait for messages
	 */
    ROS_INFO("depthnet node initialized, waiting for messages");
    ROS_SPIN();

    //free resources
    delete net;
    delete input_cvt;
    delete visualization_cvt;

    return 0;
}