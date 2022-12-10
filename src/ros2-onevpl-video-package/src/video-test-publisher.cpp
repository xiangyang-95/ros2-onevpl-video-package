#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace cv;

class VideoReader : public rclcpp::Node
{
public:
    VideoReader(const string &video_file)
        : Node("video_reader"),
          video_file_(video_file),
          cap_(video_file),
          publisher_(this->create_publisher<sensor_msgs::msg::Image>("image", 1))
    {
        // Check if the video file was successfully opened
        if (!cap_.isOpened())
        {
            cerr << "Unable to open the video file" << endl;
            exit(1);
        }

        double fps = cap_.get(CAP_PROP_FPS);
        int frame_count = cap_.get(CAP_PROP_FRAME_COUNT);
        cout << "Video file information:" << endl;
        cout << "  FPS: " << fps << endl;
        cout << "  Frame count: " << frame_count << endl;

        // Start the video reader loop
        timer_ = this->create_wall_timer(
            chrono::milliseconds(1000 / 30),
            bind(&VideoReader::timer_callback, this));
    }

private:
    // Timer callback function
    void timer_callback()
    {
        // Read the next frame from the video file
        Mat frame;
        if (cap_.read(frame))
        {
            // Convert the frame to a ROS2 image message
            auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

            // Publish the image message
            publisher_->publish(*image_msg);
        }
    }

    // Video capture object
    VideoCapture cap_;

    // Video file name
    string video_file_;

    // Timer for publishing video frames
    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher for the image topic
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Check if a video file was specified on the command line
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <video_file>" << endl;
        return -1;
    }

    string video_file = argv[1];

    // Create and run the video reader node
    auto node = make_shared<VideoReader>(video_file);
    rclcpp::spin(node);

    return 0;
}