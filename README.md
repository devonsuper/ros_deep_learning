# Depthnet node for ros_deep_learning
This repo expands on the original ros_deep_learning repo by adding a monodepth ROS node. This node should be compatible with every configuration that the original ros_deep_learning can run, however it has only been tested with ROS1 melodic on the Jetson Xavier NX with Tensorrt 8.2.

**Compatibility:** One thing to note is that the node only currently supports Tensorrt 8.2 and above, because one of the monodepth2 model layers has reflection padding, which Tensorrt only supports in versions 8.2 and later.

**Models:** The node uses the monodepth DNN from the [`jetson-inference`](https://github.com/dusty-nv/jetson-inference) library, however it does not support any jetson-inference pretrained models. All models must be loaded from an ONNX file and must have a single input and a single output. To convert monodepth2 models to ONNX in a compatible fashion, refer to my [`monodepth2 fork`](https://github.com/devonsuper/monodepth2).

**ROS Topics:**
- depthnet/image_in - input, raw images for conversion
- depthnet/depth - output, depth images in 32FC1 format
- depthnet/visualization - output, visualization images

**Parameters:** 
- model_path - load model from this file. Launch files default to "$(find ros_deep_learning)/models/mono_1024x320.simplified.onnx."
- input_blob - name of input layer. Default is "input", which is also the correct name for any model exported by my monodepth2 repo
- output_blob - name of output layer. Default is "output", which is also the correct name for any model exported by my monodepth2 repo
- precision - which precision mode to load the network with. Supports "FP32", "FP16", "INT8" and "FASTEST".
- visualize - boolean for whether or not to publish visualization images. Default is false.
- measure_time - boolean for whether or not to publish average preprocess, process, and publish times to the log. Default is false.

**Launch Files:**
- depthnet.ros1.launch - launches depthnet along with ros_deep_learning input stream and output stream
- depthnetexperiment.ros1.launch - launches depthnet with a given power mode ("power_mode") and inputs a provided image folder ("image_folder"). Logs the outputs per second of the node to a file.
- depthnetonfile.ros1.launch - given a folder with an image file, performs inference on the file and saves the visualization to disk.
