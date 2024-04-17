#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <thread>
#include <mutex>
#include <condition_variable>

// Global variable to store the Vector3D message
gz::msgs::Vector3d velocity;
std::mutex mutex;
std::condition_variable cv;
bool newData = false;

// Function called each time a topic update is received.
void cb(const gz::msgs::DVLVelocityTracking &_msg)
{
    // Obtain the velocity data of DVLVelocityTracking
    // Update the message
    {
        std::lock_guard<std::mutex> lock(mutex);
        velocity = _msg.velocity().mean();
        newData = true;
    }
    cv.notify_one();
}

int main(int argc, char **argv)
{
    gz::transport::Node node;
    std::string topic = "/aiapaec/gazebo/dvl";
    std::string topic1 = "/aiapaec/gazebo/dvl_bridge";

    auto pub = node.Advertise<gz::msgs::Vector3d>(topic1);
    // Subscribe to the DVLVelocityTracking topic 
    // by registering a callback.
    if (!node.Subscribe(topic, cb))
    {
        std::cerr << "Error suscribing to the topic [" << topic << "]" << std::endl;
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
        pub.Publish(velocity);
    }
    return 0;
}
