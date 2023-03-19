#ifndef SYNTHETIC_MEASUREMENT_GENERATOR_HPP
#define SYNTHETIC_MEASUREMENT_GENERATOR_HPP

#include "blindspot_generator.hpp"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/ChannelFloat32.h"

#include "ros/ros.h"
#include <deque>
#include <random>

#include "tf/tf.h"

/**
 * This class republish vrpn topic msgs in a given probabiliy. Created to test Probability of detection 
 * of a adapative PHD filter.
 * 
 * author - Thivanka_perera thiva@uri.edu
 * 
 */

class SyntheticMeasurementGenerator {

    /**
     * rosnode handle for internal use
    */
    ros::NodeHandle n_;

    /**
     * @brief topic that publish congregated messages from each vrpn topic
     * 
     */
    ros::Publisher smg_pub;
    
    /**
     * @brief synthetic measurement msg that publishes in the topic
     * 
     */
    sensor_msgs::PointCloud smg_pub_msg;

    /**
     * @brief publish odom empty odom msgs for time update
     * this publish to keep time update intact.
     * 
     */
    ros::Publisher publisher_for_odom;

    /**
     * @brief synthetic odom msg. it is a empty msg.
     * avoid changing the time update.
     * 
     */
    nav_msgs::Odometry odom_msg;

    /**
     * publish vrpn topic msgs that being used as synthetic msgs.
     * this for error calculation because smg publisher doesnt have all robots position.
     * 
    */
    ros::Publisher republisher_vrpn;

    /**
     * @brief msg for republisher_vrpn
     * 
     */
    sensor_msgs::PointCloud rvrpn_pub_msg;

    /**
     * cov value for white noise generator.
    */
    double white_noise_cov;

    /**
     * amount to deduc from the existing pd
    */
    double pd_decay_amount;

    /**
     * duration of time steps for pd decay to occur
    */
    int decay_time_duration;


    /**
     * @brief strruct to hold each topic information along with its subcribers
     * data containers and publishers as required.
     * 
     */
    struct rosCommHolder {

        /**
         * @brief Construct a new ros Comm Holder object
         * 
         * @param n nodehandle to passed inside
         * @param topic topic name for subcribe
         */
        rosCommHolder( ros::NodeHandle n, std::string topic, double pd, double white_noise_cov);
    
        /**
         * @brief holds the msgs from vrpn topic.
         * the reason to use deque is the efficiency when often remove elements
         * from the front compared to vectors. unlike vectors, deque scatters data
         * across the memory and do not relocate container when grows
         * 
         */
        std::unique_ptr < std::deque <geometry_msgs::PoseStamped> > incoming_msg_container_;

        /**
         * @brief nodehandle for communications
         * 
         */
        ros::NodeHandle comm_nh_;

        /**
         * @brief subcriber for the topic
         * 
         */
        ros::Subscriber sub_;

        /**
         * @brief publisher if required
         * 
         */
        ros::Publisher pub_;

        /**
         * @brief callback function for each subcriber.
         * push each msg to the data container above.
         * 
         * @param msg 
         */
        void vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        /**
         * @brief retreived first msg received from list
        */
        geometry_msgs::Point32 getMsg();

        /**
         * read from the container and delete.
        */
        geometry_msgs::Point32 readMsg();

        /**
         * check if a point can be retrieved by checking following conditions
         * is container not empty
         * is counter reached to pd or counter is over pd
         * 
        */
        bool isMsgAvailable();

        /**
         * @brief custom pd for this topic
        */
        double pd_;

        /**
         * @brief rate of PD reduction over time steps
         * @param decay_rate rate to change
        */
        void decay_pd (double decay_amount);

        /**
         * @brief counter for publisher. used to determined the pd
        */
        int counter_;

        /**
         * introduce noise to the point
        */
        void add_noise(geometry_msgs::Point32& msg);

        /**
         * @brief probability for publishing measurements.
         * used bournulli distribution.
         * 
         */
        std::random_device rd_;
        std::mt19937* generator_;
        std::unique_ptr<std::bernoulli_distribution> bernoulli_dist_; //pd generator
        std::unique_ptr<std::normal_distribution<double> > gaussian_dist_; //white noise generator

        ~rosCommHolder();
    };

    /**
     * @brief holds ros communication class for each vrpn topic.
     * This allows to dynamically change the number of topics without changing the code.
     * map key - topic name
     * data type - pointer to roscommHolder class object
     * 
     */
    std::unique_ptr<std::map <std::string, rosCommHolder* > > topics_container;


    bool enable_blind_spot_;
    std::unique_ptr <blindspot_generator> blindspot_ptr;

    public:

        /**
         * Constructor for the class
         *  @param n nodle handle to be passed inside
        */
        SyntheticMeasurementGenerator(ros::NodeHandle n);

        ~SyntheticMeasurementGenerator();

};



#endif