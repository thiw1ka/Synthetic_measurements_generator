#include "synthetic_measurement_generator/blindspot_generator.hpp"

blindspot_generator::blindspot_generator() {};

blindspot_generator::blindspot_generator(ros::NodeHandle n, double fov_angle, 
                    double effective_distance, double originX, double originY, 
                    double rate, double ang_increment) :
                    n_(n),
                    fov_angle_(fov_angle),
                    ang_increment_(ang_increment),
                    effective_distance_(effective_distance) {

    pub_ = n_.advertise <visualization_msgs::Marker> ("blindspot_lines", 1, false);
    src_ = n_.advertiseService ("blindspots", &blindspot_generator::serviceCallback, this);
    std::cout << "blind spot class started.." << std::endl;

    n_.getParam("/fov_angle", fov_angle_);
    // std::printf("field of view angle from the yaml = %f",fov_angle_);
    fov_angle_ = fov_angle_* M_PI/180;
    n_.getParam("/effective_distance", effective_distance_);
    n_.getParam("/originX", originX);
    n_.getParam("/originY", originY);
    n_.getParam("/rate", rate_);
    n_.getParam("/ang_increment", ang_increment_);

    triangle_point_0_ << originX, originY; 
    triangle_point_1_ << 0.0, 0.0;
    triangle_point_2_ << 0.0, 0.0;
    createBlindspotTriangle();

    timer_for_publisher = n_.createTimer(ros::Duration(0.05), &blindspot_generator::publisher, this);
    timer_for_increment = n_.createTimer(ros::Duration(rate_), &blindspot_generator::incrementTriangle, this);
     
    // ros::Rate r(rate);
    // while(ros::ok()){
        

    //     r.sleep();
    // }

}

//static variable here

double blindspot_generator::azimuth = 0.0;


void blindspot_generator::incrementTriangle (const ros::TimerEvent& a){
    azimuth += ang_increment_;
    std::cout << "azimuth " << azimuth << std::endl;
    createBlindspotTriangle();
}


void blindspot_generator::createBlindspotTriangle (void) {
    /*
        consider triangle is in a circle where each side lengh is same as circle radius
        one corner is on the circle center and other two corners are on the circle's circumference
        point for center is given
        point 1 is calculated based on the azimuth angle which represents the angle from x axis (0 degree) to start of the triangle
        point 2 calculate by adding azimuth + fov.
        azimuth can be used to rotate the triangle
    */
    if(azimuth >= 360) azimuth = 0.0;
    double azimuth_rad = azimuth * M_PI / 180;
    double x_displacement = effective_distance_ * std::cos(azimuth_rad);
    double y_displacement = effective_distance_ * std::sin(azimuth_rad);
    triangle_point_1_ = triangle_point_0_ + Eigen::Vector2d(x_displacement, y_displacement);
    
    x_displacement = effective_distance_ * std::cos(azimuth_rad + fov_angle_);
    y_displacement = effective_distance_ * std::sin(azimuth_rad + fov_angle_);
    triangle_point_2_ = triangle_point_0_ + Eigen::Vector2d(x_displacement, y_displacement);

}

double blindspot_generator::calculateTriangleArea (const Eigen::Vector2d& corner1, const Eigen::Vector2d& corner2, const geometry_msgs::Point32& pt ){
    //area of an triangle using 3 coordinate points
    return 0.5*(abs(corner1[0]*(corner2[1]-pt.y)+corner2[0]*(pt.y-corner1[1])+pt.x*(corner1[1]-corner2[1])));

}

// template <typename T>
// double getBlindSpotArea ( const T& corner1, const T& corner2, const T& corner3){
//     double area =  0.5 * (abs(corner1[0]*(corner2[1]-corner3[1])+corner2[0]*(corner3[1]-corner1[1])+corner3[0]*(corner1[1]-corner2[1])));
//     return area;
// }

bool blindspot_generator::isPointInsideBlindSpot (const geometry_msgs::Point32& pt) {

    double blind_spot_area = getBlindSpotArea <Eigen::Vector2d> (triangle_point_0_, triangle_point_1_, triangle_point_2_);
    double area_to_match = calculateTriangleArea(triangle_point_0_, triangle_point_1_, pt);
    area_to_match +=  calculateTriangleArea(triangle_point_0_, triangle_point_2_, pt);
    area_to_match +=  calculateTriangleArea(triangle_point_1_, triangle_point_2_, pt);
    
    return ( trunc(blind_spot_area * 100) == trunc(area_to_match * 100)); // accurate for 1 cm
// double bloom_filter::phdfilter::get_total_area(gaussian_component& point){
// //calculating the area of four triangles
//     Eigen::Vector2d m(point.getMean());
//     double tot=area_triang(_imgcornerpts[0],_imgcornerpts[1],m(0),m(1));
//     tot+=area_triang(_imgcornerpts[0],_imgcornerpts[2],m(0),m(1));
//     tot+=area_triang(_imgcornerpts[1],_imgcornerpts[3],m(0),m(1));
//     tot+=area_triang(_imgcornerpts[2],_imgcornerpts[3],m(0),m(1));
//     return trunc(tot);
// }

}

void blindspot_generator::set_blindspot_triangle (const geometry_msgs::Point32& a, const geometry_msgs::Point32& b, const geometry_msgs::Point32& c){
    triangle_point_0_ << a.x, a.y; 
    triangle_point_1_ << b.x, b.y;
    triangle_point_2_ << c.x, c.y;
}

bool blindspot_generator::serviceCallback (synthetic_measurement_generator::blindspotServiceMsg::Request &req, synthetic_measurement_generator::blindspotServiceMsg::Response &res){
    if(req.request) {
        geometry_msgs::Point32 p;
        p.z = 0.0;
        p.x = triangle_point_0_[0]; p.y = triangle_point_0_[1];
        res.points.push_back(p);
        p.x = triangle_point_1_[0]; p.y = triangle_point_1_[1];
        res.points.push_back(p);
        p.x = triangle_point_2_[0]; p.y = triangle_point_2_[1];
        res.points.push_back(p);
        return true;
    }
    return false;

}

void blindspot_generator::publisher (const ros::TimerEvent& f) {
    // std::cout << "blind spot publishing.." << std::endl;
    visualization_msgs::Marker msg;
    msg.header.frame_id     = "experiment";
    msg.header.stamp        = ros::Time::now();
    msg.pose.orientation    = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    msg.color.b             = 1;
    msg.color.g             = 0;
    msg.color.a             = 1;
    msg.type                = visualization_msgs::Marker::LINE_STRIP;
    msg.lifetime            = ros::Duration(0.3);
    msg.ns                  = "line";
    msg.scale.x             = 0.05;

    geometry_msgs::Point p;
    p.z = 0.1;
    p.x = triangle_point_0_[0]; p.y = triangle_point_0_[1];
    msg.points.push_back(p);
    p.x = triangle_point_1_[0]; p.y = triangle_point_1_[1];
    msg.points.push_back(p);
    p.x = triangle_point_2_[0]; p.y = triangle_point_2_[1];
    msg.points.push_back(p);
    p.x = triangle_point_0_[0]; p.y = triangle_point_0_[1];
    msg.points.push_back(p);

    pub_.publish(msg);

}

//init static variables here

