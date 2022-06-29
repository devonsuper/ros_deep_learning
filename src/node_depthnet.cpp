//
// Created by Devon Super on 6/7/22.
//
#include "ros/ros.h"

#include <jetson-inference/depthNet.h>

#include "ros_compat.h"
#include "image_converter.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"


//globals
depthNet* net = NULL;

imageConverter* input_cvt = NULL;
imageConverter* visualization_cvt = NULL;

Publisher<std_msgs::Float32MultiArray> depth_pub = NULL;
Publisher<sensor_msgs::Image> visualization_pub = NULL;

int batchsize = 1;
bool visualize;


int imagecount = 0;

//publish raw depth data
void publish_depths(){

    float* depths = net->GetDepthField();
    int height = net->GetDepthFieldHeight();
    int width = net->GetDepthFieldWidth();

    std_msgs::Float32MultiArray msg;

    //from https://answers.ros.org/question/234028/how-to-publish-a-2-dimensional-array-of-known-values/
    //TODO unclear if this implementation works with ROS2

    //construct msg dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].label = "height";
    msg.layout.dim[1].label = "width";
    msg.layout.dim[0].size = height;
    msg.layout.dim[1].size = width;
    msg.layout.dim[0].stride = height*width;
    msg.layout.dim[1].stride = width;
    msg.layout.data_offset = 0;

    //load depths onto msg
    std::vector<float> vec( depths, depths + (width*height) );

    msg.data = vec;

    //TODO create wrapper for msg so that values like header can be added.
    //populate timestamp
    //msg.header.stamp = ROS_TIME_NOW();

    //publish
    depth_pub->publish(msg);

}

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
    int c = imagecount;

   // convert the image to reside on GPU
   if( !input_cvt || !input_cvt->Convert(input) )
   {
       ROS_INFO("failed to convert %ux%u %s image", net->GetDepthFieldWidth(), net->GetDepthFieldHeight(), input->encoding.c_str());
       return;
   }
   
    ROS_INFO("Processing image %u", imagecount);

   // process the depth network
   if( !net->Process(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight()) )
   {
       ROS_ERROR("failed to process depth perception on %ux%u image", input->width, input->height);
       return;
   }

    if( ROS_NUM_SUBSCRIBERS(depth_pub) > 0 ){
        ROS_INFO("publishing image %u", imagecount);

        publish_depths();
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

    //retrieve parameters
    ROS_GET_PARAMETER("model_path", model_path);
    ROS_GET_PARAMETER("input_blob", input_blob);
    ROS_GET_PARAMETER("output_blob", output_blob);
    ROS_GET_PARAMETER("precision", precisionstring);
    ROS_GET_PARAMETER("visualize", visualize);


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

	if( !input_cvt || !visualization_cvt  )
	{
		ROS_ERROR("failed to create imageConverter objects");
		return 0;
	}

    //announce publishing topics
    ROS_CREATE_PUBLISHER(std_msgs::Float32MultiArray, "depth", 2000, depth_pub); //TODO find a good queue size.
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