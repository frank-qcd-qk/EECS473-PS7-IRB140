//get images from topic "simple_camera/image_raw"; remap, as desired;
//search for red pixels;
// convert (sufficiently) red pixels to white, all other pixels black
// compute centroid of red pixels and display as a blue square
// publish result of processed image on topic "/image_converter/output_video"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
//! Additional Message for calucating transform
#include <math.h>
#include <numeric>

//! Use the following to record camera location experiment:
//* The following are used for recording initial centralization data:
double e_i1 = 320.0;
double e_j1 = 240.0;
double e_x1 = 0.543547;
double e_y1 = 0.319172;
//* The following are used for recording second set of data:
double e_i2 = 471.65789;
double e_j2 = 209.47368;
double e_x2 = 1.0;
double e_y2 = 0.319172;
//* The following are used for controlled calculation
double e_i3 = 320.0;
double e_j3 = 332.0;
double e_x3 = 0.490582;
double e_y3 = 0.053320;

double e_i4 = 265.0;
double e_j4 = 240.0;
double e_x4 = 0.385604;
double e_y4 = 0.350798;

static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

int g_redratio; //threshold to decide if a pixel qualifies as dominantly "red"

const double BLOCK_HEIGHT=0.035; // hard-coded top surface of block relative to world frame

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher block_pose_publisher_; // = n.advertise<std_msgs::Float64>("topic1", 1);
    geometry_msgs::PoseStamped block_pose_;
    XformUtils xformUtils;

public:
    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("simple_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        block_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("block_pose", 1, true); 
        block_pose_.header.frame_id = "world"; //specify the  block pose in world coords
        block_pose_.pose.position.z = BLOCK_HEIGHT;
        block_pose_.pose.position.x = 0.5; //not true, but legal
        block_pose_.pose.position.y = 0.0; //not true, but legal
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(0); //not true, but legal
        
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    //image comes in as a ROS message, but gets converted to an OpenCV type
    void imageCb(const sensor_msgs::ImageConstPtr& msg); 
    
}; //end of class definition

//* Adapted From https://stackoverflow.com/questions/18939869/how-to-get-the-slope-of-a-linear-regression-line-using-c
double linearRegSlope(const vector<double>& x, const vector<double>& y){
    if(x.size() != y.size()){
        ROS_FATAL("Recorder value not match! Error!");
        return 0;
    }
    double n = x.size();

    double avgX = accumulate(x.begin(), x.end(), 0.0) / n;
    double avgY = accumulate(y.begin(), y.end(), 0.0) / n;

    double numerator = 0.0;
    double denominator = 0.0;

    for(int i=0; i<n; ++i){
        numerator += (x[i] - avgX) * (y[i] - avgY);
        denominator += (x[i] - avgX) * (x[i] - avgX);
    }

    if(denominator == 0){
        ROS_FATAL("Slope Denominator = 0! Error!");
        return 0;
    }

    return numerator / denominator;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr; //OpenCV data type
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::vector<double> xRecorder;
        std::vector<double> yRecorder;
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0; //count the red pixels
        int isum = 0; //accumulate the column values of red pixels
        int jsum = 0; //accumulate the row values of red pixels
        int redval, blueval, greenval, testval;
        cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel
        //comb through all pixels (j,i)= (row,col)
        for (int i = 0; i < cv_ptr->image.cols; i++) {
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j, i); //extract an RGB pixel
                //examine intensity of R, G and B components (0 to 255)
                redval = rgbpix[2] + 1; //add 1, to avoid divide by zero
                blueval = rgbpix[0] + 1;
                greenval = rgbpix[1] + 1;
                //look for red values that are large compared to blue+green
                testval = redval / (blueval + greenval);
                //if red (enough), paint this white:
                if (testval > g_redratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix++; //note that found another red pixel
                    isum += i; //accumulate row and col index vals
                    jsum += j;
                    //! Recorder here!!!
                    xRecorder.push_back(i-320);
                    yRecorder.push_back(j-240);
                } else { //else paint it black
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                }
            }
        }
        //cout << "npix: " << npix << endl;
        //paint in a blue square at the centroid:
        int half_box = 5; // choose size of box to paint
        int i_centroid, j_centroid;
        double x_centroid, y_centroid;
        if (npix > 0) {
            i_centroid = isum / npix; // average value of u component of red pixels
            j_centroid = jsum / npix; // avg v component
            x_centroid = ((double) isum)/((double) npix); //floating-pt version
            y_centroid = ((double) jsum)/((double) npix);
            ROS_INFO("u_avgu_avg: %f; v_avg: %f",x_centroid,y_centroid);
            //cout << "i_avg: " << i_centroid << endl; //i,j centroid of red pixels
            //cout << "j_avg: " << j_centroid << endl;
            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 255; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 0;
                    }
                }
            }

        }
        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3); //need waitKey call to update OpenCV image window

        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun image_view image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());
        ROS_INFO("[Find_Block.cpp]Hand Over to my code:...");

        //* We have x_centroid, y_centroid as input information, theta as 0
        // ! The following protion is the calculation of transformaed x,y coordinate
        // * Use experiment result define the world location for camera:
        double camera_center_x, camera_center_y;
        camera_center_x = 0.543547;        
        camera_center_y = 0.319172;
        ROS_INFO("[Math] Camera world positionis: [%f, %f]",camera_center_x,camera_center_y);

        //* Calculate Scale Factor:
        double scale_factor;
        scale_factor = sqrt((e_x1-e_x2)*(e_x1-e_x2)+(e_y1-e_y2)*(e_y1-e_y2))/sqrt((e_i1-e_i2)*(e_i1-e_i2)+(e_j1-e_j2)*(e_j1-e_j2)); //THis value should be around 0.003
        ROS_INFO("[Math] Camera scale factor is: %f",scale_factor);

        //* Calculate Corrected camera position:
        double corrected_camera_x, corrected_camera_y;
        corrected_camera_x = (x_centroid - 320)*scale_factor * 1.0;
        corrected_camera_y = (y_centroid - 240)*scale_factor * -1.0;
        ROS_INFO("[Math] Calculated Corrected Camera position is: [%f,%f]",corrected_camera_x,corrected_camera_y);

        //*Construct Matrix for transformations:
        Eigen::Matrix4d TF_object2camera; // Object to Camera Frame transformation
        Eigen::Matrix4d TF_camera2robot; // Camera to Robot Frame transformation
        Eigen::Matrix4d F_TF_object2robot; // Final Transformation result matrix

        //* Calculate rotation slope:
        //To calculate slope, I imagine the points as points need a linear fit, and by using linear regression algorithm fitting a line within a bunch of points.
        double slope;
        slope = linearRegSlope(xRecorder,yRecorder);// difference/norm
        ROS_INFO("[Math] Slop Calculated is: %f",slope);

        //* Calculate rotation object to camera:
        double rotationo2c;
        rotationo2c = atan(slope);
        ROS_INFO("[Math] Calculated rotationo2c is: %f.",rotationo2c*180/3.1415926);
        
        //* Populate object to camera matrix:
        TF_object2camera.row(0) << cos(rotationo2c) , -sin(rotationo2c), 0 , corrected_camera_x;
        TF_object2camera.row(1) << sin(rotationo2c) , cos(rotationo2c) , 0 , corrected_camera_y;
        TF_object2camera.row(2) <<          0       ,         0        , 1 ,        0;
        TF_object2camera.row(3) <<          0       ,         0        , 0 ,        1;
        ROS_INFO_STREAM("[Math_MAT] Calculated Object to Camera Transform matrix is: "<<endl<<TF_object2camera);
        ROS_INFO("[Math] Pass Object to Camera matrix population...");
        
        //* Populate camera to robot matrix:
        double camera_rotation = -0.2; //By experiement and measurement
        TF_camera2robot.row(0) << cos(camera_rotation),-sin(camera_rotation),0,camera_center_x;
        TF_camera2robot.row(1) << sin(camera_rotation), cos(camera_rotation),0,camera_center_y;
        TF_camera2robot.row(2) <<           0         ,         0           ,1,       0;
        TF_camera2robot.row(3) <<           0         ,         0           ,0,       1;
        ROS_INFO_STREAM("[Math_MAT] Calculated Camera to Robot Transform matrix is: "<<endl<<TF_camera2robot);
        ROS_INFO("[Math] Pass Camera to robot matrix population...");

        //* Final Transformation matrix = Object to Camera * Camera to Robot:
        F_TF_object2robot = TF_camera2robot*TF_object2camera;
        ROS_INFO_STREAM("[Math_MAT] Calculated Object to Robot Transform matrix is: "<<endl<<F_TF_object2robot);
        ROS_INFO("[Math] Pass Final Transform Matrix Calculation...");
        
        //* Obtain Useful Information from final matrix:
        double x_coordinate = F_TF_object2robot(0,3);
        double y_coordinate = F_TF_object2robot(1,3);
        double z_coordinate = 0;
        double orientation = -acos(F_TF_object2robot(0,0));
        ROS_INFO("[Math_Final] Caculated x_coordinate is: %f, y_coordinate is: %f, Orientation is: %f",x_coordinate,y_coordinate,orientation*180/3.14515);
        
        //* Give back to system:
        block_pose_.pose.position.x = x_coordinate; 
        block_pose_.pose.position.y = y_coordinate;
        block_pose_.pose.position.z = z_coordinate;
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(orientation); //not true, but legal
        block_pose_publisher_.publish(block_pose_);
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_pixel_finder");
    ros::NodeHandle n; //        
    ImageConverter ic(n); // instantiate object of class ImageConverter
    //cout << "enter red ratio threshold: (e.g. 10) ";
    //cin >> g_redratio;
    g_redratio= 10; //choose a threshold to define what is "red" enough
    ros::Duration timer(0.1);
    double x, y, z;
    ROS_INFO("[Find_Block.cpp]Called!");
    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
