#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <thread>
#include <mutex>
#include <condition_variable>

// Global variable to store the Vector3D message

gz::msgs::Time sim_time_;
gz::msgs::Time real_time_;
gz::msgs::Double rtf_;
bool newData = false;
std::condition_variable cv;
std::mutex mutex;


void cb(const gz::msgs::WorldStatistics &_msg)
{
    // Obtain the velocity data of DVLVelocityTracking
    // Update the message
    {
        std::lock_guard<std::mutex> lock(mutex);
        sim_time_ = _msg.sim_time();
        real_time_ = _msg.real_time();
        rtf_.set_data(_msg.real_time_factor());
        newData = true;
    }
    cv.notify_one();
}

int main(int argc, char **argv)
{
    gz::transport::Node node;
    std::string topic_x1 = "/world/aiapaec_underwater/stats";
    std::string topic_x2 = "/aiapaec/gazebo/sim_time_bridge";
    std::string topic_x3 = "/aiapaec/gazebo/real_time_bridge";
    std::string topic_x4 = "/aiapaec/gazebo/rtf_bridge";

    auto pub_sim_time = node.Advertise<gz::msgs::Time>(topic_x2);
    auto pub_real_time = node.Advertise<gz::msgs::Time>(topic_x3);
    auto pub_rtf = node.Advertise<gz::msgs::Double>(topic_x4);

    // Subscribe to the DVLVelocityTracking topic 
    // by registering a callback.
    if (!node.Subscribe(topic_x1, cb))
    {
        std::cerr << "Error suscribing to the topic [" << topic_x1 << "]" << std::endl;
        return -1;
    }
    // Main Loop
    while (true)
    {
        // Waiting new data
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, []{ return newData; });
            newData = false;
        }
        // Publish the message
        pub_sim_time.Publish(sim_time_);
        pub_real_time.Publish(real_time_);
        pub_rtf.Publish(rtf_);
    }
    return 0;
}
