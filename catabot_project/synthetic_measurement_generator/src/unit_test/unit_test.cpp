#include "unit_test.hpp"

UnitTest::UnitTest(){};

UnitTest::UnitTest(ros::NodeHandle n) {
    std::cout << "unit test started " << std::endl;

    std::unique_ptr < blindspot_generator > bgTester (
                    new blindspot_generator(n, 30.0, 5.0, 0.0, 0.0));


}

bool UnitTest::TestTriangleAreaCalculation() {
    // triangle_point_0_ << (0.0, 0.0);
    // triangle_point_1_ << (0.0, 0.0);
    // triangle_point_2_ << (0.0, 0.0);
    std::printf("------- Testing triangle are calculation------\n");
    Eigen::Vector2d corn_1 {0.0,0.0};
    Eigen::Vector2d corn_2 {0.0,10.0};
    geometry_msgs::Point32 pt;
    pt.x = 10.0;
    pt.y = 0.0;
    double area = calculateTriangleArea(corn_1, corn_2, pt);
    std::printf("calculated area = %f \n", area);
    std::cout << "Test triangle area calc Pass  == " << bool(area == 50) << std::endl;
    return (area == 50);
}

void UnitTest::TestPointInsideFunc (){
    std::printf("------- Testing if a point is inside------\n");
    triangle_point_0_ << 0.0, 0.0;
    triangle_point_1_ << 0.0, 10.0;
    triangle_point_2_ << 10.0, 0.0;
    geometry_msgs::Point32 pt;
    pt.x = 0.0;
    pt.y = -10.0;
    bool result = isPointInsideBlindSpot(pt);
    std::cout << "triangle points -" <<std::endl
        << triangle_point_0_[0] <<"," << triangle_point_0_[1] <<std::endl
        << triangle_point_1_[0] <<"," << triangle_point_1_[1] <<std::endl
        << triangle_point_2_[0] <<"," << triangle_point_2_[1] <<std::endl;
    std::cout << "testing point - " << pt.x << "," << pt.y << std::endl;
    std::cout << "point is inside true or false == " << result << std::endl;

}

int main(int argc, char* argv[] ) {
    ros::init( argc, argv,  "unit_tester_synthetic_generator" );
    ros::NodeHandle nh;
    std::cout << "Unit_test_synthetic_measurement_generator_node started.." << std::endl;


    UnitTest Tester;
    bool output = Tester.TestTriangleAreaCalculation();

    Tester.TestPointInsideFunc ();
    // ros::AsyncSpinner spinner(0); // Use 4 threads
    // spinner.start();
    // UnitTest tester(nh);
    // ros::waitForShutdown();
    return 0;
};