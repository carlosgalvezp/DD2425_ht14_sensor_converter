#include "ros/ros.h"
#include "ras_arduino_msgs/ADConverter.h"

#include "ras_utils/basic_node.h"

#include <iostream>
#include <fstream>

const std::string NODE_NAME     = "sensor_converter";
const std::string ADC_SUB_NAME  = "/arduino/adc";// "/kobuki/adc";

const int LOOP_SIZE = 100;

const int RATE              = 10; // Hz
const int QUEUE_SIZE        = 1000;
const int SKIP_INIT_VALUES  = 10; //Just to make sure we don't include strange inital readings.

const std::string HOME_DIR              = getenv("HOME");

const std::string ADC_FILE_DIR          = "/" ;
const std::string ADC_FILE_NAME         = "adc_values";
const std::string ADC_FILE_FULL_PATH    = HOME_DIR + ADC_FILE_DIR + ADC_FILE_NAME;

const std::string DIST_FILE_DIR         = "/" ;
const std::string DIST_FILE_NAME        = "adc_distance";
const std::string DIST_FILE_FULL_PATH   = HOME_DIR + DIST_FILE_DIR + DIST_FILE_NAME;

const std::string NEW_LINE = "\n";

// Note that this class use only sensor on channel one (ch1), which is the front-left on the robot.
class SensorConverter : rob::BasicNode
{
public:
    SensorConverter() : adc_value(0) {

        add_param("sc_loop", loop_size, LOOP_SIZE);
        add_param("sc_distance", distance);
        print_params();

        // Start subscribing on ADC-Sensors.
        adc_sub_ = n.subscribe(ADC_SUB_NAME, QUEUE_SIZE,  &SensorConverter::adcCallback, this);
    }

    // We do everything in the callback instead so we make sure that we don't read the same value twice.
    void run() {

        ros::Rate loop_rate(RATE);

        int adc_accumilate_value = 0;

        long accumilate_counter = 0;
        int skip_counter = 0;

        while(ros::ok() && accumilate_counter < loop_size) {
            if(adc_value == 0) {
                //we have no input yet, do nothing
            }else if(skip_counter < SKIP_INIT_VALUES) {
                skip_counter++;
            } else {
                adc_accumilate_value += adc_value;
                accumilate_counter++;
            }
            // ** Sleep
            ros::spinOnce();
            loop_rate.sleep();
        }

        double adc_avarage = (double)adc_accumilate_value / accumilate_counter;



        ROS_INFO("distance: %d  adc_avarage: %lf  accumilated: %ld", distance, adc_avarage, accumilate_counter);

        // Open the files for output
        file_adc.open(ADC_FILE_FULL_PATH.c_str(), std::ios_base::app);
        file_dist.open(DIST_FILE_FULL_PATH.c_str(), std::ios_base::app);
        file_adc << adc_avarage << NEW_LINE;
        file_dist << distance << NEW_LINE;

        file_adc.close();
        file_dist.close();
    }

private:

    ros::Subscriber adc_sub_;

    std::ofstream file_adc;
    std::ofstream file_dist;

    int adc_value;
    int distance;
    int loop_size;

    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg) {
        // Note that we only use the sensor on channel one (ch1), which is the front-left on the robot.
        adc_value = msg->ch8;
    }
};

int main (int argc, char* argv[]){
    ros::init(argc, argv, NODE_NAME);

    SensorConverter sc;

    sc.run();

    return 0;
}
