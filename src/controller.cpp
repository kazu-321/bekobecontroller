#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QKeyEvent>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QCoreApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <twistring/msg/twistring.hpp>
#include <krb2024_msgs/msg/robot_status.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <map>

std::string cmd;
krb2024_msgs::msg::RobotStatus robot_status;
bool paused_;

class ControlPanel : public QMainWindow {
public:
    ControlPanel(QWidget *parent = nullptr) : QMainWindow(parent) {
        QWidget *centralWidget = new QWidget(this);
        layout = new QVBoxLayout(centralWidget);
        add_control("PID 0", 3);
        add_control("PID 1", 3);
        add_control("PID 2", 3);
        add_control("PID 3", 3);
        add_control("init 0", 1);
        add_control("init 1", 1);
        add_control("init 2", 1);
        add_control("init 3", 1);
        QPushButton *button_load = new QPushButton("load", this);
        connect(button_load, &QPushButton::clicked, this,&ControlPanel::set_value);
        layout->addWidget(button_load);
        QPushButton *button_close = new QPushButton("close", this);
        connect(button_close, &QPushButton::clicked, [this](){this->close();paused_=false;});
        layout->addWidget(button_close);
        // set_value();
        centralWidget->setLayout(layout);
        setCentralWidget(centralWidget);
    }
protected:
    void keyPressEvent(QKeyEvent *event) override {
        if(event->key() == Qt::Key_Escape){
            this->close();
            paused_ = false;
        }
    }

private:
    void set_gain(krb2024_msgs::msg::Gain &gain,QList<QLineEdit*> lineEdits){
        lineEdits[0]->setText(QString::number(gain.p));
        lineEdits[1]->setText(QString::number(gain.i));
        lineEdits[2]->setText(QString::number(gain.d));
    }


    void set_value(){
        set_gain(robot_status.gain0,controls["PID 0"]);
        set_gain(robot_status.gain1,controls["PID 1"]);
        set_gain(robot_status.gain2,controls["PID 2"]);
        set_gain(robot_status.gain3,controls["PID 3"]);
        controls["init 0"][0]->setText(QString::number(robot_status.rotation_init.data0));
        controls["init 1"][0]->setText(QString::number(robot_status.rotation_init.data1));
        controls["init 2"][0]->setText(QString::number(robot_status.rotation_init.data2));
        controls["init 3"][0]->setText(QString::number(robot_status.rotation_init.data3));
    }
    void add_control(const QString &name, const int num = 1) {
        QHBoxLayout *rowLayout = new QHBoxLayout();
        QLabel *label = new QLabel(name, this);
        rowLayout->addWidget(label);

        // QLineEditを保存するリスト
        QList<QLineEdit*> lineEdits;

        // 複数のQLineEditを作成し、リストに追加
        for (int i = 0; i < num; i++) {
            QLineEdit *lineEdit = new QLineEdit(this);
            rowLayout->addWidget(lineEdit);
            lineEdits.append(lineEdit);
        }

        // QLabelの名前とQLineEditのリストを保存
        controls[name] = lineEdits;

        // ボタンを作成し、ボタンに対応するラベルの名前を設定
        QPushButton *button = new QPushButton("設定", this);
        button->setProperty("labelName", name); // ボタンにラベルの名前を保存
        connect(button, &QPushButton::clicked, this, &ControlPanel::onSettingButtonClicked);
        rowLayout->addWidget(button);

        layout->addLayout(rowLayout);
    }

    void onSettingButtonClicked() {
        QPushButton *button = qobject_cast<QPushButton *>(sender());
        if (!button) return; // 安全確認

        // ボタンに保存したラベル名を取得
        QString labelText = button->property("labelName").toString();
        
        // QLabelに対応するQLineEditのリストを取得
        QList<QLineEdit*> lineEdits = controls[labelText];

        // コマンドを作成
        std::string command = "set " + labelText.toStdString();
        for (QLineEdit *lineEdit : lineEdits) {
            command += " " + lineEdit->text().toStdString();
        }

        printf("command: %s\n", command.c_str());
        cmd = command;
    }

    QVBoxLayout *layout;

    // QLabelの名前と対応するQLineEditのリストを管理
    QMap<QString, QList<QLineEdit*>> controls;
};




class RobotController : public QMainWindow {
    Q_OBJECT

public:
    RobotController(rclcpp::Node::SharedPtr node)
        : QMainWindow(), node_(node) {
        setWindowTitle("ロボットコントローラー");
        resize(640, 480);

        // メインウィジェットを作成
        QWidget *mainWidget = new QWidget(this);
        QVBoxLayout *mainLayout = new QVBoxLayout(mainWidget);

        // 画像を表示するラベルを作成
        image_label_ = new QLabel(this);
        image_label_->setMinimumSize(1, 1);
        // image_label_->setScaledContents(true);
        this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        mainLayout->addWidget(image_label_);


        // メインウィジェットを設定
        setCentralWidget(mainWidget);

        // イベントフィルターをインストール
        QCoreApplication::instance()->installEventFilter(this);

        // コマンドを発行するためのタイマー
        command_timer_ = new QTimer(this);
        connect(command_timer_, &QTimer::timeout, this, &RobotController::publishCommand);
        command_timer_->start(50);  // 20Hz

        // 画像のサブスクライバー
        image_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&RobotController::imageCallback, this, std::placeholders::_1));
        status_subscriber_= node_->create_subscription<krb2024_msgs::msg::RobotStatus>(
            "/robot_status", 10, std::bind(&RobotController::set_status, this, std::placeholders::_1));

        // コマンドのパブリッシャー
        command_publisher_ = node_->create_publisher<twistring::msg::Twistring>("cmd_vel", 10);

        // キー状態の初期化
        key_state_['W'] = false;
        key_state_['A'] = false;
        key_state_['S'] = false;
        key_state_['D'] = false;
        key_state_[16777236] = false;  // 右矢印キー
        key_state_[16777234] = false;  // 左矢印キー

        old_mouse_x_ = -1;
        last_mouse_y_ = 0;

        paused_=false;

        std::signal(SIGINT, &RobotController::signalHandler);
    }
    ~RobotController() {
        // 必要に応じてクリーンアップ
    }

    static void signalHandler(int signum) {
        RCLCPP_INFO(rclcpp::get_logger("robot_controller"), "Interrupt signal received, shutting down.");
        QApplication::quit(); // QApplicationを終了
    }

protected:
    void set_status(const krb2024_msgs::msg::RobotStatus::SharedPtr msg){
        robot_status = *msg;
    }

    void keyPressEvent(QKeyEvent *event) override {
        key_state_[event->key()] = true;
        if(event->key() == 'C'){
            cmd="continue";
        }else if(event->key() == 'P'){
            cmd="pause";
        }else if(event->key() == 'R'){
            cmd="reset";
        }else if(event->key() == Qt::Key_Escape){
            // QApplication::quit();
            control_panel_ = new ControlPanel();
            control_panel_->setWindowTitle("Control Panel");
            control_panel_->resize(400, 300);
            control_panel_->show();
            paused_ = true;
        }
    }

    void keyReleaseEvent(QKeyEvent *event) override {
        key_state_[event->key()] = false;
    }

    bool eventFilter(QObject *obj, QEvent *event) override {
        if (event->type() == QEvent::MouseMove && obj == image_label_ && !paused_) {
            QMouseEvent *mouse_event = static_cast<QMouseEvent *>(event);
            if(old_mouse_x_ == -1) {
                old_mouse_x_ = mouse_event->x();
            }
            
            last_mouse_x_ = mouse_event->x();
            last_mouse_y_ = mouse_event->y();
            int window_width = width();
            if (last_mouse_x_ <= 30) {
                // 右端にラップアラウンド
                QCursor::setPos(mapToGlobal(QPoint(window_width - 70, last_mouse_y_)));
                last_mouse_x_ = window_width - 50;
                old_mouse_x_ = last_mouse_x_;
            } else if (last_mouse_x_ >= window_width - 50) {
                // 左端にラップアラウンド
                QCursor::setPos(mapToGlobal(QPoint(70, last_mouse_y_)));
                last_mouse_x_ = 70;
                old_mouse_x_ = last_mouse_x_;
            }
            return true; // イベントを処理済み
        }
        return QMainWindow::eventFilter(obj, event); // 他のイベントは基底クラスに渡す
    }

    void resizeEvent(QResizeEvent* event) override{
        if (!pixmap_.isNull()) {
            image_label_->setPixmap(pixmap_.scaled(width(),height(), Qt::KeepAspectRatio));
        }
        QMainWindow::resizeEvent(event);
    }


    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // ROSの画像メッセージをOpenCVの画像に変換
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image;

            // 画像が空かどうかをチェック
            if (current_image_.empty()) {
                RCLCPP_WARN(node_->get_logger(), "Received empty image!");
                return;
            }

            // 元の画像の寸法を保存
            int original_width = current_image_.cols;
            int original_height = current_image_.rows;

            // 現在のウィンドウサイズを取得

            // 画像の中心にクロスヘアを描画
            int center_x = current_image_.cols / 2;
            int center_y = current_image_.rows / 2;
            int crosshair_size = 20; // クロスヘアのサイズ
            // 水平線を描画
            cv::line(current_image_, cv::Point(center_x - crosshair_size, center_y), 
                     cv::Point(center_x + crosshair_size, center_y), cv::Scalar(0, 0, 255), 2);

            // 垂直線を描画
            cv::line(current_image_, cv::Point(center_x, center_y - crosshair_size), 
                     cv::Point(center_x, center_y + crosshair_size), cv::Scalar(0, 0, 255), 2);
            // QImageに変換
            QImage q_image(current_image_.data, current_image_.cols, current_image_.rows,
                        static_cast<int>(current_image_.step[0]), QImage::Format_BGR888);
            // 新しい画像でラベルを更新
            pixmap_=QPixmap::fromImage(q_image);
            image_label_->setPixmap(pixmap_.scaled(width(),height(), Qt::KeepAspectRatio));
            // image_label_->setPixmap(pixmap_);
            image_label_->update();  // 再描画を要求
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "Error processing image: %s", e.what());
        }
    }

    void mouseReleaseEvent(QMouseEvent *event) override {
        if(event->button() == Qt::LeftButton) {
            cmd = "click";
        }
    }

    void publishCommand() {
        auto twistring_msg = std::make_shared<twistring::msg::Twistring>();
        twistring_msg->twist.linear.x = 0.0;
        twistring_msg->twist.linear.y = 0.0;
        twistring_msg->twist.angular.z = 0.0;
        if(key_state_['W']) twistring_msg->twist.linear.x += 1.0;
        if(key_state_['S']) twistring_msg->twist.linear.x -= 1.0;
        if(key_state_['A']) twistring_msg->twist.linear.y += 1.0;
        if(key_state_['D']) twistring_msg->twist.linear.y -= 1.0;
        if(key_state_[16777236]) twistring_msg->twist.angular.z -= 1.0;
        if(key_state_[16777234]) twistring_msg->twist.angular.z += 1.0;

        twistring_msg->twist.angular.x = (old_mouse_x_ - last_mouse_x_)/100.0;
        twistring_msg->twist.angular.y = last_mouse_y_;
        twistring_msg->cmd = cmd;
        cmd = "";
        old_mouse_x_ = last_mouse_x_;

        // コマンドを発行
        command_publisher_->publish(*twistring_msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    QLabel *image_label_;
    QTimer *command_timer_;
    QPixmap pixmap_;
    cv::Mat current_image_;
    std::map<int, bool> key_state_;
    int last_mouse_x_, last_mouse_y_, old_mouse_x_; 
    rclcpp::Publisher<twistring::msg::Twistring>::SharedPtr command_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<krb2024_msgs::msg::RobotStatus>::SharedPtr status_subscriber_;
    ControlPanel *control_panel_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robot_controller");

    QApplication app(argc, argv);
    RobotController window(node);
    // window.showMaximized();
    window.show();

    // ノードをスピンするスレッドを追加
    std::thread rclcpp_thread([&node]() { rclcpp::spin(node); });

    int result = app.exec();

    rclcpp::shutdown();
    rclcpp_thread.join();  // スレッドを待機

    return result;
}

#include "controller.moc"
