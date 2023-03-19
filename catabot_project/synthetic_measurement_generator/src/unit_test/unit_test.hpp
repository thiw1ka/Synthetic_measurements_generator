#include "synthetic_measurement_generator/blindspot_generator.hpp"

struct UnitTest : blindspot_generator {

    UnitTest();

    UnitTest(ros::NodeHandle n);

    bool TestTriangleAreaCalculation();

    void TestPointInsideFunc();

};