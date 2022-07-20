#include <iostream>
#include <chrono>
#include <memory> 
#include <cstdio>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;

class CamPublisher_: public rclcpp::Node
{
  public:
    CamPublisher_()
    : Node("camera")
    {
		setvbuf(stdout, NULL, _IONBF, BUFSIZ);
		initialize();
    }

	~CamPublisher_()
	{
		cap.release();
	}

  private:
	//functions
	void initialize()
	{
		configure();
		//topic_image = "/camera/image_raw";
		//topic_compressed = "/camera/compressed";
		auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
		pub_image = this->create_publisher<sensor_msgs::msg::Image>(
			topic_image, qos
		);
		// pub_compressed = this->create_publisher<sensor_msgs::msg::CompressedImage>(
		// 	topic_compressed, qos
		// );
		//pub_transport = it.advertise(topic_compressed, 10);

		cap.open(0);

		if (!cap.isOpened()) {
			RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
			throw std::runtime_error("Could not open video stream");
		}
		cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 640);

		timer_ = this->create_wall_timer(
				std::chrono::milliseconds(static_cast<int>(1000 / 60)), // 30 fps
				std::bind(&CamPublisher_::timerCallback, this));
	}

	void timerCallback()
	{
		cv::Mat frame;
		cap >> frame;
		if (frame.empty()) {
			return;
		}
		// For flip setting
		if(is_flip_v&&is_flip_h) {
			cv::flip(frame, frame, 0);
			cv::flip(frame, frame, 1);
		} else {
			if(is_flip_v) {
				cv::flip(frame, frame, 0);
			} else if(is_flip_h) {
				cv::flip(frame, frame, 1);
			}
		}
		convert_and_publish(frame);
	}

	std::string mat2encoding(int mat_type)
	{
		switch (mat_type) {
			case CV_8UC1:
				return "mono8";
			case CV_8UC3:
				return "bgr8";
			case CV_16SC1:
				return "mono16";
			case CV_8UC4:
				return "rgba8";
			default:
				std::runtime_error("Unsupported encoding type");
		}
		return 0;
	}

	void convert_and_publish(const cv::Mat& frame)
	{
		auto msg = sensor_msgs::msg::Image();
		msg.height = frame.rows;
		msg.width = frame.cols;
		msg.encoding = mat2encoding(std::move(frame.type()));
		msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);

		size_t size = frame.step * frame.rows;
		msg.data.resize(size);
		memcpy(&msg.data[0], frame.data, size);

		pub_image->publish(msg);
		//sensor_msgs::msg::CompressedImage img_compressed_msg;
	}

	void configure()
    {
		this->declare_parameter<std::string>("image.topic", "/camera/image_raw");
		this->declare_parameter<bool>		("image.flip_v", false);
		this->declare_parameter<bool>		("image.flip_h", false);
		this->get_parameter<std::string>    ("image.topic",       topic_image);
		this->get_parameter<bool>           ("image.flip_v",      is_flip_v);
		this->get_parameter<bool>           ("image.flip_h",      is_flip_h);
		RCLCPP_INFO(this->get_logger(),"Flip: %d", is_flip_v);
	}

	//vars
	cv::VideoCapture cap;

	std::string topic_image;
	//std::string topic_compressed;
	bool is_flip_v;
	bool is_flip_h;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
	//rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed;
	//image_transport::ImageTransport it;
	//image_transport::Publisher pub_transport;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamPublisher_>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
