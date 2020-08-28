#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

class AttitudeExtractor
{
public:
    AttitudeExtractor()
    {
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        m_subscriber = it.subscribe("/pano_1/image_raw", 1, &AttitudeExtractor::imageCallback, this);
        cv::namedWindow("image_to_attitude");
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
        
        cv::Mat grey;
        cv::cvtColor(cv_ptr->image, grey, CV_BGR2GRAY);

        cv::Mat detected_edges;
        cv::blur(grey, detected_edges, cv::Size(30,30));
        cv::Canny(detected_edges, detected_edges, 0, 0, 3);
        
        std::vector<cv::Vec2f> lines;
        cv::HoughLines(detected_edges, lines, 1, CV_PI/180.0, 500);
        std::cerr << lines.size() << " lines" << std::endl;
        for( size_t i = 0; i < lines.size() && i < 75; i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 2500*(-b));
            pt1.y = cvRound(y0 + 2500*(a));
            pt2.x = cvRound(x0 - 2500*(-b));
            pt2.y = cvRound(y0 - 2500*(a));
            cv::line( cv_ptr->image, pt1, pt2, cv::Scalar(0,0,255), 2, cv::LINE_AA);
        }
        
        cv::imshow("image_to_attitude", cv_ptr->image);
        cv::waitKey(3);
    }
    
    image_transport::Subscriber m_subscriber;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_to_attitude");
    AttitudeExtractor ae;
    ros::spin();
    return 0;
}
