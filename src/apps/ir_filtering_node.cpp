// ROS
#include <ros/ros.h>

// RAS
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_utils/basic_node.h>
#include <ras_srv_msgs/IRData.h>
#include <ras_utils/ras_names.h>
#include <ras_utils/kalman_filter.h>
#include <ras_utils/ras_sensor_utils.h>

#include <fstream>

// Kalman Filter parameters
#define QSHORT 0.2          // Sensor noise for short range [m]
#define QLONG  0.2          // Sensor noise for long range  [m]
#define R0     0.1           // Process noise [m]
#define SIGMA_0 100.0        // Initial uncertainty

class IR_Filtering : rob::BasicNode
{
public:
    IR_Filtering();
private:
    void adcCallback(const ras_arduino_msgs::ADConverterConstPtr &adc_msg);

    ros::Subscriber adc_sub_;
    ros::Publisher ir_pub_;
    ros::Publisher ir_no_filter_pub_;

    std::vector<KalmanFilter> kf_short_;
    std::vector<KalmanFilter> kf_long_;

    void initializeKF();

    std::ofstream fshort, flong;

};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ir_filtering");
    IR_Filtering ir_filter;
    ros::spin();

    return 0;
}

IR_Filtering::IR_Filtering()
{
    // ** Subscribers
    adc_sub_ = n.subscribe(TOPIC_ARDUINO_ADC, 1, &IR_Filtering::adcCallback, this);

    // ** Publishers
    ir_pub_           = n.advertise<ras_srv_msgs::IRData>(TOPIC_ARDUINO_ADC_FILTERED, 1);
    ir_no_filter_pub_ = n.advertise<ras_srv_msgs::IRData>(TOPIC_ARDUINO_ADC_NOT_FILTERED, 1);

    // ** Init KF
    initializeKF();

    fshort.open("/home/ras/ir_short.txt");
    flong.open("/home/ras/ir_long.txt");
}

void IR_Filtering::adcCallback(const ras_arduino_msgs::ADConverterConstPtr &adc_msg)
{
    // ** Get raw data
    Eigen::VectorXd z_front_right(1), z_back_right(1), z_front_left(1), z_back_left(1), z_front(1), z_back(1);

    z_front_left << RAS_Utils::sensors::shortSensorToDistanceInCM(adc_msg->ch1) * 0.01;
    z_back_left  << RAS_Utils::sensors::shortSensorToDistanceInCM(adc_msg->ch2) * 0.01;
    z_back_right   << RAS_Utils::sensors::shortSensorToDistanceInCM(adc_msg->ch3) * 0.01;
    z_front_right  << RAS_Utils::sensors::shortSensorToDistanceInCM(adc_msg->ch4) * 0.01;
    z_back       << RAS_Utils::sensors::longSensorToDistanceInCM(adc_msg->ch7) * 0.01;
    z_front        << RAS_Utils::sensors::longSensorToDistanceInCM(adc_msg->ch8) * 0.01;


    ras_srv_msgs::IRData msg_not_filtered;
    msg_not_filtered.front_right = z_front_right(0);
    msg_not_filtered.front_left  = z_front_left(0);
    msg_not_filtered.back_right  = z_back_right(0);
    msg_not_filtered.back_left   = z_back_left(0);
    msg_not_filtered.front       = z_front(0);
    msg_not_filtered.back        = z_back(0);

    Eigen::VectorXd f_front_right(1), f_back_right(1), f_front_left(1), f_back_left(1), f_front(1), f_back(1);

    // ** Filter with Kalman Filter
    kf_short_[0].filter(z_front_right, f_front_right);
    kf_short_[1].filter(z_back_right,  f_back_right);
    kf_short_[2].filter(z_front_left,  f_front_left);
    kf_short_[3].filter(z_back_left,   f_back_left);

    kf_long_[0].filter(z_front, f_front);
    kf_long_[1].filter(z_front, f_front);

    // ** Output filtered data
    ras_srv_msgs::IRData msg_filtered;
    msg_filtered.front_right = f_front_right(0);
    msg_filtered.front_left  = f_front_left(0);
    msg_filtered.back_right  = f_back_right(0);
    msg_filtered.back_left   = f_back_left(0);
    msg_filtered.front       = f_front(0);
    msg_filtered.back        = f_back(0);



//    flong << z_front(0)<< " " <<f_front(0)<<std::endl;
//    fshort << z_front_right(0)<< " " <<f_front_right(0)<<std::endl;

    // ** Publish msgs
    ir_pub_.publish(msg_filtered);
    ir_no_filter_pub_.publish(msg_not_filtered);
}


void IR_Filtering::initializeKF()
{
    Eigen::MatrixXd A(1,1);
    Eigen::MatrixXd B(1,1);
    Eigen::MatrixXd C(1,1);
    Eigen::MatrixXd R(1,1);
    Eigen::MatrixXd Q_short(1,1);
    Eigen::MatrixXd Q_long(1,1);
    Eigen::VectorXd mu0(1);
    Eigen::MatrixXd sigma0(1,1);

    A << 1;
    B << 0;
    C << 1;
    R << R0*R0;
    Q_short  << QSHORT*QSHORT;
    Q_long   << QLONG*QLONG;

    mu0 << 0;
    sigma0 << SIGMA_0*SIGMA_0;

    for(std::size_t i = 0; i < 4; ++i)
    {
        kf_short_.push_back(KalmanFilter(mu0, sigma0, A, B, C, R, Q_short));
    }

    for(std::size_t i = 0; i < 2; ++i)
    {
        kf_long_.push_back(KalmanFilter(mu0, sigma0, A, B, C, R, Q_long));
    }
}
