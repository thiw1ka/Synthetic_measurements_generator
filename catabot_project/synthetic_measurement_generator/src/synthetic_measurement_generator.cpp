#include "synthetic_measurement_generator/synthetic_measurement_generator.hpp"

SyntheticMeasurementGenerator::rosCommHolder::rosCommHolder(//constructor
                        ros::NodeHandle n, 
                        std::string topic, 
                        double pd,
                        double white_noise_cov = 0.01) : comm_nh_(n), pd_(pd) {
    incoming_msg_container_.reset(new std::deque < geometry_msgs::PoseStamped > );
    sub_ = comm_nh_.subscribe(topic, 10, &rosCommHolder::vrpnCallback, this);
    std::size_t found = topic.rfind("ugv");
    if (found == std::string::npos)
        std::cout << "[SMG] Error extracting UGV name" << std::endl;
    generator_ = new std::mt19937(rd_());
    bernoulli_dist_.reset(new std::bernoulli_distribution(pd_)); //create distribution with probability = pd of true state
    gaussian_dist_.reset(new std::normal_distribution<double> (0, white_noise_cov) ); //white noise generator
    std::cout << "[SMG] subcribed to the topic -  "<< topic.substr(found, 5) 
                << "PD - " << pd_ << std::endl;
}

SyntheticMeasurementGenerator::SyntheticMeasurementGenerator(ros::NodeHandle n) : n_(n) {//constructor
    if (!ros::master::check()) { //check if rosmaster is up
            std::cout << "[SMG] ros master is not up. exiting..." << std::endl;
            exit(0);
    }    
    std::string state = "init";
    ros::Rate r(10); //5hz
    std::vector<double> copy_of_pds;
    int counter_for_decay = 0;
    while (ros::ok()) {
        if (state == "init") {
            std::vector<double> pd_for_each_target;
            n_.getParam("/pd_for_each_target", pd_for_each_target);
            n_.getParam("/white_noise_cov", white_noise_cov);
            n_.param <double> ("/decay_amount", pd_decay_amount, 0.0);
            n_.param <int> ("/decay_time_duration", decay_time_duration, 0);
            n_.param <bool> ("/enable_blind_spot", enable_blind_spot_, false);
            std::copy(pd_for_each_target.begin(), pd_for_each_target.end(), std::back_inserter(copy_of_pds));
            if (pd_for_each_target.size() == 0) {
                state == "exit";
                std::cout << "[SMG] pd list is empty. check smg.launch file..." << std::endl;
                continue;
            }

            // n_.param <double[]> ("/pd_list", pd_list, {5, 5, 5, 5});
            std::cout << "[SMG] waiting 2 seconds till rosbag up and running..." << std::endl;
            ros::Duration(2.0).sleep(); //wait till rosbag plays
            ros::master::V_TopicInfo topic_list;
            bool successfully_read_topics = ros::master::getTopics(topic_list);
            if (!successfully_read_topics) {
                std::cout << "[SMG] Error reading the rostopics from the ros master. Exiting..." 
                            << std::endl;
                state = "exit";
                continue;
            }
            topics_container.reset(new std::map <std::string, rosCommHolder* > );
            // double pd_for_each_target [5] = {0.3, 0.4, 0.9, 0.8}; // list of pd. add number between 0 to 10. 10 IS NOT VALID
            //create seperate object for each topic.
            for (auto topic : topic_list) {
                // std::cout << topic.na?me << std::endl;
                // std::cout << topic.datatype << std::endl;
                std::string::size_type found_vrpn_topic = topic.name.find("vrpn");
                if (found_vrpn_topic != std::string::npos) {
                    std::size_t found = topic.name.rfind("ugv");
                    std::string name = topic.name.substr(found, 5);
                    static int pd_index = 0;
                    topics_container->emplace(name,new rosCommHolder( n_, topic.name, pd_for_each_target[pd_index], white_noise_cov));
                    ++pd_index;
                    // subCBPointer name = new SyntheticMeasurementGenerator::vrpnCallback;
                    // subcribers_for_topics[name] = subCBPointer ( &SyntheticMeasurementGenerator::vrpnCallback);
                    // std::cout << "Address of the pointer = " <<  subcribers_for_topics.at(name) << std::endl;
                }
            }
            if (topics_container->empty() == true) {
                std::cout << "[SMG] no vrpn topics found. waiting..."    << std::endl;
                continue;
            }

            /*blind spot generator*/
            if (enable_blind_spot_) blindspot_ptr.reset(new blindspot_generator(n_));

            smg_pub = n_.advertise < sensor_msgs::PointCloud > ("synthetic_measurements", 10);
            republisher_vrpn = n_.advertise < sensor_msgs::PointCloud > ("synthetic_measurements_vrpn", 10);
            publisher_for_odom = n_.advertise < nav_msgs::Odometry > ("synthetic_odom", 10);
            
            odom_msg.header.frame_id = "experiment";
            odom_msg.pose.pose.position.x = 0;
            odom_msg.pose.pose.position.y = 0;
            odom_msg.pose.pose.position.z = 0;
            odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
            smg_pub_msg.header.frame_id = "experiment";
            rvrpn_pub_msg = smg_pub_msg;
            std::cout << "[SMG] topics successfully aquired " << std::endl;
            state = "aquire_data";
        }
        else if (state == "aquire_data") {
            publisher_for_odom.publish(odom_msg);
            smg_pub_msg.points.clear();
            rvrpn_pub_msg.points.clear();
            for (auto robot_topic : *topics_container.get()) {
                geometry_msgs::Point32 retrived_pt = robot_topic.second->getMsg();
                double is_point_valid = retrived_pt.x + retrived_pt.y + retrived_pt.z;
                if (is_point_valid != 0) {//continue;
                    rvrpn_pub_msg.points.push_back(retrived_pt);

                    //adding noise only for the measurements
                    robot_topic.second->add_noise(retrived_pt);
                    smg_pub_msg.points.push_back(retrived_pt);
                    //blind spot checker
                    if(enable_blind_spot_) {
                        bool ispointinside = blindspot_ptr -> isPointInsideBlindSpot(retrived_pt);
                        if (ispointinside) smg_pub_msg.points.pop_back(); //remove last point if inside
                
                    }
                }
                else if(!robot_topic.second->incoming_msg_container_ -> empty()) {
                    rvrpn_pub_msg.points.push_back(robot_topic.second ->readMsg()); //making sure location is published if pd doent allow
                }

            }
            if(enable_blind_spot_) {
                sensor_msgs::ChannelFloat32 x,y;
                x.name = "x";
                y.name = "y";
                for (auto pt : smg_pub_msg.points) {
                    
                }

            }
            smg_pub.publish(smg_pub_msg);
            republisher_vrpn.publish(rvrpn_pub_msg);

            // PD decay done here
            if (pd_decay_amount > 0){
                counter_for_decay++;
                if( counter_for_decay == decay_time_duration){
                    counter_for_decay = 0;
                    for (auto robot_topic : *topics_container.get()) {
                        robot_topic.second -> decay_pd(pd_decay_amount);
                        if (robot_topic.second ->pd_ <= 0){ // if pd is invalid
                            std::printf("[SMG] PD of one robot reached to zero or below. exiting..");
                            state == "exit";
                            return;
                        }
                    }
                }
            }


        }
        else if (state == "exit") {
            std::cout << "[SMG] Error. exiting.."    << std::endl;
            exit(0);
        }
        bool is_rate_acheived = r.sleep();
        // std::cout << "rate is acheived - " << is_rate_acheived << std::endl;
    }
}

void SyntheticMeasurementGenerator::rosCommHolder::decay_pd (double decay_amount){
    pd_ = pd_ - decay_amount;
    if (pd_ < 0) {
        std::printf("[rosCommHolder] PD is less than zero, reset zero");
        pd_ = 0;
    }
    bernoulli_dist_.reset(new std::bernoulli_distribution(pd_)); //create distribution with probability = pd of true state

}

void SyntheticMeasurementGenerator::rosCommHolder::add_noise(geometry_msgs::Point32& msg){
    msg.x =  msg.x + gaussian_dist_->operator()(*generator_);
    msg.y =  msg.y + gaussian_dist_->operator()(*generator_);//opti-track tranformation applied here
    msg.z =  msg.z;
}

bool SyntheticMeasurementGenerator::rosCommHolder::isMsgAvailable(){

    // ++counter_;
    bool pd_output (bernoulli_dist_->operator()(*generator_)); //check probability output
    bool is_msg_container_empty = incoming_msg_container_->empty();
    // std::cout << "[SMG] pd output = "<<pd_output <<", pd is "<< pd_ << std::endl;
    
    if (pd_output && !is_msg_container_empty) {
        //probabilty allows to send msg && msges are present in the list
        // counter_ = 0;
        // std::cout << "[SMG] checking msg availablility " << std::endl;
        return true;
    }
    // else if (!is_msg_container_empty) {
    //     //probabilty down't allows to send msg. remove the msg from the list
    //     incoming_msg_container_->pop_front(); //remove the front if msgs are in the list
    // }
    return false;
}

geometry_msgs::Point32 SyntheticMeasurementGenerator::rosCommHolder::readMsg() {
    geometry_msgs::Point32 msg;
    geometry_msgs::PoseStamped poped_msg = incoming_msg_container_->operator[](0);
    msg.x =  poped_msg.pose.position.x; //+ gaussian_dist_->operator()(*generator_);
    msg.y =  poped_msg.pose.position.z; //+ gaussian_dist_->operator()(*generator_);//opti-track tranformation applied here
    msg.z =  -poped_msg.pose.position.y;
    // std::printf("\n x = %f, y = %f, z = %f ",msg.x, msg.y, msg.z);
    // std::printf("x = %f, y = %f, z = %f", gaussian_dist_->operator()(*generator_), gaussian_dist_->operator()(*generator_), gaussian_dist_->operator()(*generator_));
    incoming_msg_container_ -> pop_front(); //remove the front
    return msg;
}

geometry_msgs::Point32 SyntheticMeasurementGenerator::rosCommHolder::getMsg() {
    geometry_msgs::Point32 msg;
    msg.x =  0;
    msg.y =  0;
    msg.z =  0;
    bool is_msg_availble = isMsgAvailable();
    if (!is_msg_availble) return msg; //returning an empty message.
    return readMsg();
}

void SyntheticMeasurementGenerator::rosCommHolder::vrpnCallback(
                            const geometry_msgs::PoseStamped::ConstPtr& msg) {
    incoming_msg_container_ -> push_back(*msg);
    if (incoming_msg_container_->size() > 20) incoming_msg_container_->pop_front();
}


SyntheticMeasurementGenerator::rosCommHolder::~rosCommHolder() {
    delete generator_;
    incoming_msg_container_->~deque();
    incoming_msg_container_.~unique_ptr();
}

SyntheticMeasurementGenerator::~SyntheticMeasurementGenerator() {
    // topics_container.~unique_ptr();
    std::cout << "[SMG] Synthetic Measurement Generator destructor called..." << std::endl;
}