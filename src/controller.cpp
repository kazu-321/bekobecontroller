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

        setMouseTracking(true);

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
        key_state_['W'] = false;
        key_state_['A'] = false;
        key_state_['S'] = false;
        key_state_['D'] = false;

        old_mouse_x_=-1;

        std::signal(SIGINT, &RobotController::signalHandler);
    }
    ~RobotController() {
        // Cleanup if needed
    }

    static void signalHandler(int signum) {
        // Perform cleanup here if necessary
        RCLCPP_INFO(rclcpp::get_logger("robot_controller"), "Interrupt signal received, shutting down.");
        QApplication::quit(); // Quit the QApplication
    }

protected:
    void keyPressEvent(QKeyEvent *event) override {
        key_state_[event->key()] = true;
    }

    void keyReleaseEvent(QKeyEvent *event) override {
        key_state_[event->key()] = false;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV image
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image;

            // Check if the image is empty
            if (current_image_.empty()) {
                RCLCPP_WARN(node_->get_logger(), "Received empty image!");
                return;
            }

            // Store original image dimensions
            int original_width = current_image_.cols;
            int original_height = current_image_.rows;

            // Get the current window size
            int new_width = this->size().width();
            int new_height = this->size().height();

            // Calculate the new dimensions while maintaining the aspect ratio
            double aspect_ratio = static_cast<double>(original_width) / static_cast<double>(original_height);
            if (new_width / aspect_ratio <= new_height) {
                new_height = static_cast<int>(new_width / aspect_ratio);
            } else {
                new_width = static_cast<int>(new_height * aspect_ratio);
            }

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

    void mouseMoveEvent(QMouseEvent *event) override {
        // Update mouse position
        // RCLCPP_INFO(node_->get_logger(), "Mouse position: %d, %d", event->x(), event->y());
        if(old_mouse_x_==-1){
            old_mouse_x_=event->x();
        }
        last_mouse_x_ = event->x();
        last_mouse_y_ = event->y();
    }

   void mouseReleaseEvent(QMouseEvent *event) override {
        if(event->button() == Qt::LeftButton) {
            cmd = "click";
        }
    }

    void publishCommand() {
        auto twistring_msg = std::make_shared<twistring::msg::Twistring>();
        twistring_msg->twist.linear.x = key_state_['W'] ? 1.0 : (key_state_['S'] ? -1.0 : 0.0);
        twistring_msg->twist.linear.y = key_state_['A'] ? 1.0 : (key_state_['D'] ? -1.0 : 0.0);
        twistring_msg->twist.angular.z = 0.0;
        twistring_msg->twist.angular.x = old_mouse_x_ - last_mouse_x_;
        twistring_msg->twist.angular.y = last_mouse_y_;
        twistring_msg->cmd = cmd;
        cmd = "";
        old_mouse_x_ = last_mouse_x_;

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
    std::string cmd;
    int last_mouse_x_, last_mouse_y_,old_mouse_x_; 
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
