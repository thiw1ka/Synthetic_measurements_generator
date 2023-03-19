#include <iostream>
#include <utility>
#include <Eigen/Eigen>
#include <cstdlib>
#include <tf/tf.h>

#include "geometry_msgs/Point32.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/PointCloud.h"
#include "synthetic_measurement_generator/blindspotServiceMsg.h"

#ifndef BLINDSPOT_GENERATOR_HPP
#define BLINDSPOT_GENERATOR_HPP

/**
 * @brief blind spot is created by rotating a cone from a given position.
 * 
 */
struct blindspot_generator{

    blindspot_generator();

    /**
     * @brief Construct a new blindspot generator object
     * 
     * @param n 
     * @param fov_angle angle in degrees
     * @param effective_distance 
     * @param originX 
     * @param originY 
     * @param rate rate for triangle rotations. default = 1.0 hz
     * @param ang_increment angle increment. default = 5.0 degrees
     */
    blindspot_generator(ros::NodeHandle n, double fov_angle = 15.0, double effective_distance = 5.0, 
                        double originX = 0.0, double originY = 0.0, double rate = 1.0, double ang_increment = 5.0);

    /**
     * @brief node handle for publisher 
     * 
    */
    ros::NodeHandle n_;

    /**
     * @brief the rate of triangle rotation;
     * 
     */
    double rate_;

    /**
     * @brief field of view angle
     * 
     */
    double fov_angle_;

    /**
     * @brief angle increment for every time step
     * 
     */
    double ang_increment_;

    /**
     * @brief blind spot effective distance
     * 
     */
    double effective_distance_;

    /**
     * @brief points of the traiangle that contain the blindspot
     * @param first X coordinate
     * @param second Y Coordinate
     * 
     */
    Eigen::Vector2d triangle_point_0_, triangle_point_1_, triangle_point_2_;

    /**
     * @brief Angle starting from 0 degrees
     * 
     */
    static double azimuth;

    /**
     * @brief compute coordinates for the triangle
     * 
     */
    void createBlindspotTriangle (void);

    /**
     * @brief check if a point is inside the blind spot.
     * 
     * @param pt Point32 format
     * @return true 
     * @return false 
     */
    bool isPointInsideBlindSpot (const geometry_msgs::Point32& pt);

    /**
     * @brief calculate triangle area
     * 
     * @param corner1 
     * @param corner2 
     * @param pt 
     * @return double 
     */
    double calculateTriangleArea (const Eigen::Vector2d& corner1, const Eigen::Vector2d& corner2, const geometry_msgs::Point32& pt );

    /**
     * @brief Get the Blind Spot Area. In this case it is a triangle
     * ###NOTE : Template func's interface(declerations) and definition(implementation in cpp) should
     * all be inside header file. cannot seperate. https://cplusplus.com/doc/oldtutorial/templates/
     * @tparam T 
     * @param corner1 
     * @param corner2 
     * @param corner3 
     * @return double 
     */
    template <typename T>
    inline double getBlindSpotArea ( const T& corner1, const T& corner2, const T& corner3){
        double area =  0.5 * (abs(corner1[0]*(corner2[1]-corner3[1])+corner2[0]*(corner3[1]-corner1[1])+corner3[0]*(corner1[1]-corner2[1])));
        return area;
    };
    
    /**
     * @brief rostopic publisher
     * 
     */
    ros::Publisher pub_;

    /**
     * @brief publisher
     * 
     */
    void publisher (const ros::TimerEvent& f);

    /**
     * @brief publish msgs in a duration
     * 
     */
    ros::Timer timer_for_publisher, timer_for_increment;

    /**
     * @brief rotate the triangle
     * 
     * @param a 
     */
    void incrementTriangle (const ros::TimerEvent& a);

    /**
     * @brief Set the blindspot triangle object
     * 
     * @param a corner 1
     * @param b corner 2
     * @param c corner 3
     */
    void set_blindspot_triangle (const geometry_msgs::Point32& a, const geometry_msgs::Point32& b, const geometry_msgs::Point32& c);

    ros::ServiceClient srvclient_;

    ros::ServiceServer src_;

    bool serviceCallback (synthetic_measurement_generator::blindspotServiceMsg::Request &req, synthetic_measurement_generator::blindspotServiceMsg::Response &res);





};


#endif