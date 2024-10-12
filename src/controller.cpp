#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QKeyEvent>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <twistring/msg/twistring.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <map>

class RobotController : public QMainWindow {
    Q_OBJECT

public:
    RobotController(rclcpp::Node::SharedPtr node)
        : QMainWindow(), node_(node), scale_factor_(1.0) {
        setWindowTitle("Robot Controller");
        resize(640, 480);

        // Create a label to display the image
        image_label_ = new QLabel(this);
        setCentralWidget(image_label_);

        // Timer for publishing commands
        command_timer_ = new QTimer(this);
        connect(command_timer_, &QTimer::timeout, this, &RobotController::publishCommand);
        command_timer_->start(50);  // 20Hz

        // Subscriber for image
        image_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&RobotController::imageCallback, this, std::placeholders::_1));

        // Publisher for commands
        command_publisher_ = node_->create_publisher<twistring::msg::Twistring>("/cmd_vel", 10);

        // Initialize key states
        key_state_['w'] = false;
        key_state_['s'] = false;
        key_state_['a'] = false;
        key_state_['d'] = false;
    }

protected:
    void keyPressEvent(QKeyEvent *event) override {
        key_state_[event->key()] = true;
        RCLCPP_INFO(node_->get_logger(), "Key pressed: %d", event->key());
    }

    void keyReleaseEvent(QKeyEvent *event) override {
        key_state_[event->key()] = false;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV image
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image;

            // Debugging: Check image size and type
            RCLCPP_INFO(node_->get_logger(), "Received image: %dx%d, type: %d", current_image_.cols, current_image_.rows, current_image_.type());

            // Check if the image is empty
            if (current_image_.empty()) {
                RCLCPP_WARN(node_->get_logger(), "Received empty image!");
                return;
            }

            // Get the current window size
            int new_width = this->size().width();
            int new_height = this->size().height();

            // Resize the image
            cv::resize(current_image_, current_image_, cv::Size(new_width, new_height));

            // Convert to QImage
            QImage q_image(current_image_.data, current_image_.cols, current_image_.rows,
                        current_image_.step[0], QImage::Format_BGR888);
            
            // Update the label with the new image
            image_label_->setPixmap(QPixmap::fromImage(q_image));
            image_label_->update();  // Request update instead of repaint
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "Error processing image: %s", e.what());
        }
    }



    void publishCommand() {
        auto twistring_msg = std::make_shared<twistring::msg::Twistring>();
        twistring_msg->twist.linear.x = key_state_['w'] ? 1.0 : (key_state_['s'] ? -1.0 : 0.0);
        twistring_msg->twist.linear.y = key_state_['a'] ? 1.0 : (key_state_['d'] ? -1.0 : 0.0);
        
        // Publish the command
        command_publisher_->publish(*twistring_msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    QLabel *image_label_;
    QTimer *command_timer_;
    cv::Mat current_image_;
    std::map<int, bool> key_state_;
    double scale_factor_;
    rclcpp::Publisher<twistring::msg::Twistring>::SharedPtr command_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robot_controller");

    QApplication app(argc, argv);
    RobotController window(node);
    window.show();

    // ノードをスピンするスレッドを追加
    std::thread rclcpp_thread([&node]() { rclcpp::spin(node); });

    int result = app.exec();

    rclcpp::shutdown();
    rclcpp_thread.join();  // スレッドを待機

    return result;
}

#include "controller.moc"
