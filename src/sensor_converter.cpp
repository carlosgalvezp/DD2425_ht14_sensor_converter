#include "ros/ros.h"
#include "ras_arduino_msgs/ADConverter.h"

#include <iostream>
#include <fstream>

const std::string NODE_NAME = "sensor_converter";
const std::string ADC_SUB_NAME = "/arduino/adc";

const int DISTANCE = 4;

const int RATE = 20; // Hz
const int QUEUE_SIZE = 1000;
const int INIT_SLEEP_TIME_MS = 100; //Just to make sure we don't include strange inital readings.

const std::string ADC_FILE_DIR = "~/" ;
const std::string ADC_FILE_NAME = "adc_values";
const std::string ADC_FILE_FULL_PATH = ADC_FILE_DIR + ADC_FILE_NAME;

const std::string DIST_FILE_DIR = "~/" ;
const std::string DIST_FILE_NAME = "adc_distance";
const std::string DIST_FILE_FULL_PATH = DIST_FILE_DIR + DIST_FILE_NAME;

const std::string NEW_LINE = "\n";

// Note that this class use only sensor on channel one (ch1), which is the front-left on the robot.
class SensorConverter
{
public:
    SensorConverter(const ros::NodeHandle &n) : adc_value(0), n_(n) {
        // Start subscribing on ADC-Sensors.

        adc_sub_ = n_.subscribe(ADC_SUB_NAME, QUEUE_SIZE,  &SensorConverter::adcCallback, this);

    }

    // We do everything in the callback instead so we make sure that we don't read the same value twice.
    void run() {
        usleep(INIT_SLEEP_TIME_MS);

        ros::Rate loop_rate(RATE);

        // Open the files for output
        file_adc.open(ADC_FILE_FULL_PATH.c_str());
        file_dist.open(DIST_FILE_FULL_PATH.c_str());

        while(ros::ok()) {

            file_adc << adc_value << NEW_LINE;
            file_dist << DISTANCE << NEW_LINE;

            // ** Sleep
            ros::spinOnce();
            loop_rate.sleep();
        }

        file_adc.close();
        file_dist.close();
    }

private:

    ros::NodeHandle n_;
    ros::Subscriber adc_sub_;

    std::ofstream file_adc;
    std::ofstream file_dist;

    int adc_value;


    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg) {
        // Note that we only use the sensor on channel one (ch1), which is the front-left on the robot.
        adc_value = msg->ch1;
    }
};

int main (int argc, char* argv[]){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    SensorConverter sc(n);

    sc.run();

    return 0;
}
