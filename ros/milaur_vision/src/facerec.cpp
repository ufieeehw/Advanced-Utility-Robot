// Include ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

// Include ROS service
#include "milaur_vision/FindFace.h"
#include <std_msgs/Int16.h>

// OpenCV Headers
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

// Misc Headers
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>

using namespace std;
using namespace cv;

const string window_name = "Capture - Face detection";
const String face_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
const string fn_csv = "/home/baylis/Desktop/FaceRecognition/csv_file.txt";


static bool processing;

class FaceRecognition
{
    boost::mutex mtx_;

public:
    ros::NodeHandle nh_; //, n
    image_transport::ImageTransport it_;
    image_transport::Subscriber im_sub_;
    image_transport::Publisher im_pub_;
    ros::Publisher pub;
    ros::Subscriber sub;

    ros::ServiceServer service;

    /** Global variables */
    cv::Mat bgr_img_orig;
    // These vectors hold the images and corresponding labels:
    vector<Mat> images;
    vector<int> labels;

    int im_width;
    int im_height;

    Ptr<FaceRecognizer> model;

    CascadeClassifier face_cascade;


    FaceRecognition()
        :   it_(nh_)
    {
        // Subscribe to the raw image
        im_sub_ = it_.subscribe("milaur_cam/image_raw", 1, &FaceRecognition::imageCallback, this);

        sub = nh_.subscribe("milaur/state", 1, &FaceRecognition::startFaceRec, this);
        pub = nh_.advertise<std_msgs::Int16>("milaur/state", 1);

        // Advertise the service
        //service = nh_.advertiseService("milaur_vision/find_face", &FaceRecognition::find_face, this);

        // Read in the data (fails if no valid input filename is given, but you'll get an error message):
        try {
            read_csv(fn_csv, images, labels);
        } catch (cv::Exception& e) {
            ROS_ERROR("milaur_vision::facerec.cpp::OpenCV exception: %s", e.what());
            // nothing more we can do
            exit(1);
        }

        // Get the height from the first image. We'll need this
        // later in code to reshape the images to their original
        // size AND we need to reshape incoming faces to this size:
        im_width = images[0].cols;
        im_height = images[0].rows;

        // Create a FaceRecognizer and train it on the given images:
        model = createLBPHFaceRecognizer();
        model->train(images, labels);

         //-- 1. Load the cascades
         if( !face_cascade.load( face_cascade_name ) )
        { 
            ROS_ERROR("milaur_vision::facerec.cpp::Error loading face cascade");
            exit(1); 
        };
    }

    static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
        std::ifstream file(filename.c_str(), ifstream::in);
        if (!file) {
            string error_message = "No valid input file was given, please check the given filename.";
            CV_Error(CV_StsBadArg, error_message);
        }
        string line, path, classlabel;
        while (getline(file, line)) {
            stringstream liness(line);
            getline(liness, path, separator);
            getline(liness, classlabel);
            if(!path.empty() && !classlabel.empty()) {
                images.push_back(imread(path, 0));
                labels.push_back(atoi(classlabel.c_str()));
            }
        }
    }

    ~FaceRecognition()
    {
        cv::destroyWindow(window_name);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("milaur_vision_node::facerec.cpp::imageCallback::ENTERED");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            boost::mutex::scoped_lock scoped_lock(mtx_);
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        bgr_img_orig = cv_ptr->image;
    }

    void startFaceRec(const std_msgs::Int16& msg){
        if(msg.data == 1){
            ROS_INFO("milaur_vision_node::facerec.cpp::processing service::ENTERED");
            processing = true;
            boost::thread processthread;
            processthread = boost::thread(boost::bind(&FaceRecognition::imageProcessing, this));
        }
    }

    void imageProcessing()
    {
        /*
        Thread to process the most recent image recieved from ROS
        */
        ROS_INFO("milaur_vision_node::facerec.cpp::processing thread::ENTERED");
        //cv::namedWindow(window_name);

        int current_face = -1;
        int frame_count = 0;

        while(frame_count < 2){
            if(!(bgr_img_orig.empty())){
                cv::Mat current_img = bgr_img_orig;
                cv::Mat frame_gray;
                std::vector<Rect> faces;

                cv::cvtColor( current_img, frame_gray, CV_BGR2GRAY );
                cv::equalizeHist( frame_gray, frame_gray );

                face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

                for(int i = 0; i < faces.size(); i++) {
                    // Process face by face:
                    Rect face_i = faces[i];
                    // Crop the face from the image. So simple with OpenCV C++:
                    cv::Mat face = frame_gray(face_i);
                    // Resizing the face is necessary for Eigenfaces and Fisherfaces. You can easily
                    // verify this, by reading through the face recognition tutorial coming with OpenCV.
                    // Resizing IS NOT NEEDED for Local Binary Patterns Histograms, so preparing the
                    // input data really depends on the algorithm used.
                    //
                    // I strongly encourage you to play around with the algorithms. See which work best
                    // in your scenario, LBPH should always be a contender for robust face recognition.
                    //
                    // Since I am showing the Fisherfaces algorithm here, I also show how to resize the
                    // face you have just found:
                    cv::Mat face_resized;
                    cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
                    // Now perform the prediction, see how easy that is:
                    int predictedLabel = -1;
                    double confidence = 0.0;

                    model->predict(face_resized, predictedLabel, confidence);

                    if (confidence < 200)
                    {
                        ROS_INFO("milaur_vision_node::facerec.cpp::processing thread::PREDICTION: %d", predictedLabel);
                        ROS_INFO("milaur_vision_node::facerec.cpp::processing thread::COUNT: %d", frame_count);
                        if(current_face == predictedLabel && predictedLabel != -1){
                            frame_count += 1;
                        } else{
                            current_face = predictedLabel;
                            frame_count = 1;
                        }

                        // // First of all draw a green rectangle around the detected face:
                        // rectangle(current_img, face_i, CV_RGB(0, 255,0), 1);
                        // // Create the text we will annotate the box with:
                        // string prediction_text = format("Prediction = %d", predictedLabel);
                        // string confidence_text = format("Confidence = %f", confidence);
                        // // Calculate the position for annotated text (make sure we don't
                        // // put illegal values in there):
                        // int prediction_pos_x = std::max(face_i.tl().x - 10, 0);
                        // int prediction_pos_y = std::max(face_i.tl().y - 10, 0);
                        // int confidence_pos_x = prediction_pos_x;
                        // int confidence_pos_y = std::max(face_i.height + 25, 0) + prediction_pos_y;
                        // // And now put it into the image:
                        // putText(current_img, prediction_text, Point(prediction_pos_x, prediction_pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
                        // putText(current_img, confidence_text, Point(confidence_pos_x, confidence_pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
                        // TEST: Not sure if I need this break or not
                        break;
                    } else {
                        // If milaur doesn't see a face, reset count
                        frame_count = 0;
                    }
                }
                // cv::imshow(window_name, current_img);
                // cv::waitKey(1);
            }
        }

        std_msgs::Int16 new_msg;
        new_msg.data = 2;

        pub.publish(new_msg);

        ROS_INFO("milaur_vision_node::facerec.cpp::processing thread::EXITING");
    }
};

/** @function main */
    int main( int argc, char** argv ) {
    ros::init(argc, argv, "facial_recognition");
    FaceRecognition facerec;
    ros::spin();
    return 0;
}