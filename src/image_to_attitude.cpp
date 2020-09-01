#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <marine_msgs/NavEulerStamped.h>

class AttitudeExtractor
{
public:
    AttitudeExtractor()
    {
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        m_subscriber = it.subscribe("/pano_1/image_raw", 1, &AttitudeExtractor::imageCallback, this);
        m_angle_pub = nh.advertise<marine_msgs::NavEulerStamped>("image_to_attitude/angle", 1);
        cv::namedWindow("output");
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat input;
        cv::resize(cv_ptr->image, input, cv::Size(0,0), 0.5, 0.5);
        
        cv::Mat cropped;
        input(cv::Rect(0,100,input.size().width,input.size().height-325)).copyTo(cropped);
        
        cv::Mat grey;
        cv::cvtColor(cropped, grey, CV_BGR2GRAY);

        cv::Mat detected_edges;
        cv::Canny(grey, detected_edges, 20, 60, 3);
        
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(detected_edges, lines, 1, CV_PI/180.0, 80, 150, 10) ;
        std::cerr << lines.size() << " lines" << std::endl;
        
        cv::Mat grey_color;
        cv::cvtColor(grey, grey_color, CV_GRAY2BGR);
        
        double sum = 0.0;
        int count = 0;
        
        for(auto l: lines)
        {
            cv::line( grey_color, cv::Point(l[0], l[1]), cv::Point(l[2],l[3]), cv::Scalar(0,0,255), 2, cv::LINE_AA);
            sum += atan2(l[3]-l[1], l[2]-l[0]); 
            count++;
        }
        std::cerr << "avg: " << (180/M_PI)*sum/double(count) << std::endl;
        
        marine_msgs::NavEulerStamped nes;
        nes.header = msg->header;
        nes.orientation.roll = (180/M_PI)*sum/double(count);
        m_angle_pub.publish(nes);
        
        cv::Mat detected_edges_color;
        cv::cvtColor(detected_edges, detected_edges_color, CV_GRAY2BGR);
        
        cv::Mat output;
        cv::vconcat(detected_edges_color, grey_color , output);
        
        cv::Mat all;
        cv::vconcat(input,output,all);
        
        cv::imshow("output", all);
        cv::waitKey(3);
    }
    
    image_transport::Subscriber m_subscriber;
    ros::Publisher m_angle_pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_to_attitude");
    AttitudeExtractor ae;
    ros::spin();
    return 0;
}
