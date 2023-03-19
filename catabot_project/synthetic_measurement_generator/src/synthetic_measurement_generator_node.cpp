#include "synthetic_measurement_generator/synthetic_measurement_generator.hpp"

int main(int argc, char* argv[] ) {
    ros::init( argc, argv,  "synthetic_measurement_generator_node" );
    ros::NodeHandle nh;
    // string filtertype = "Standard";//"adaptive";
    std::cout << "synthetic_measurement_generator_node started.." << std::endl;
    ros::AsyncSpinner spinner(0); // Use 4 threads
    spinner.start();
    std::unique_ptr < SyntheticMeasurementGenerator > SMGenerator (
                    new SyntheticMeasurementGenerator(nh));
    ros::waitForShutdown();

    return 0;
};