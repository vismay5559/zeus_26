#ifndef ZEUS_CAN_INTERFACE_HPP
#define ZEUS_CAN_INTERFACE_HPP

#include <string>
#include <functional>
#include <linux/can.h>

namespace zeus_hardware {

// A custom callback type. Notice we added a string parameter!
// When a message arrives, this tells the middle layer exactly WHICH highway it came from.
using CanFrameCallback = std::function<void(const std::string& interface_name, const can_frame& frame)>;

class ZeusCanInterface {
public:
    ZeusCanInterface();
    ~ZeusCanInterface();

    // Initializes both the muscle (ODrive) and sense (ESP32) networks
    bool init(const std::string& muscle_iface, 
              const std::string& sense_iface, 
              CanFrameCallback callback);
              
    void deinit();

    // Separated send functions so we never accidentally send ODrive commands to the ESP32s
    bool send_muscle_frame(const can_frame& frame); 
    bool send_sense_frame(const can_frame& frame);  

    // The core 1000Hz polling function. It will rapidly check both inboxes.
    void read_all_nonblocking();

private:
    // Helper functions for the internal OS networking
    int init_socket(const std::string& iface_name);
    bool read_socket_nonblocking(int socket_fd, const std::string& iface_name);

    // Linux File Descriptors for our two raw sockets
    int muscle_socket_id_; // Typically mapped to can0
    int sense_socket_id_;  // Typically mapped to can1
    
    std::string muscle_iface_name_;
    std::string sense_iface_name_;

    // The function we call to pass data UP to the zeus_control_interface layer
    CanFrameCallback frame_callback_;
};

} // namespace zeus_hardware

#endif // ZEUS_CAN_INTERFACE_HPP