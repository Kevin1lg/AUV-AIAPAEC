#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <thread>
#include <mutex>
#include <condition_variable>

// Variable global para almacenar el mensaje Vector3d
gz::msgs::Vector3d pubMsg;
std::mutex mutex;
std::condition_variable cv;
bool newData = false;

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const gz::msgs::DVLVelocityTracking &_msg)
{
    // Obtener los datos de velocidad del mensaje DVLVelocityTracking
    gz::msgs::DVLKinematicEstimate velocity_1 = _msg.velocity();
    gz::msgs::Vector3d velocity = velocity_1.mean();
  
    // Actualizar el mensaje Vector3d
    {
        std::lock_guard<std::mutex> lock(mutex);
        pubMsg = velocity;
        newData = true;
    }
    cv.notify_one();
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
    gz::transport::Node node;
    std::string topic = "/aiapaec/dvl";
    std::string topic1 = "/aiapaec/dvl";
    auto pub = node.Advertise<gz::msgs::Vector3d>(topic1);

    // Subscribe to the DVLVelocityTracking topic by registering a callback.
    if (!node.Subscribe(topic, cb))
    {
        std::cerr << "Error suscribiendo al topic [" << topic << "]" << std::endl;
        return -1;
    }

    // Bucle principal
    while (true)
    {
        // Esperar a que lleguen nuevos datos
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, []{ return newData; });
            newData = false;
        }

        // Publicar el mensaje
        pub.Publish(pubMsg);
    }

    return 0;
}
