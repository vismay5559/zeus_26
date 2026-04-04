#include "zeus_can_interface/zeus_can_interface.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cerrno>

namespace zeus_hardware {

ZeusCanInterface::ZeusCanInterface() 
    : muscle_socket_id_(-1), sense_socket_id_(-1) {}

ZeusCanInterface::~ZeusCanInterface() {
    deinit();
}

bool ZeusCanInterface::init(const std::string& muscle_iface, 
                            const std::string& sense_iface, 
                            CanFrameCallback callback) {
    muscle_iface_name_ = muscle_iface;
    sense_iface_name_ = sense_iface;
    frame_callback_ = std::move(callback);

    // Open the ODrive highway
    muscle_socket_id_ = init_socket(muscle_iface_name_);
    if (muscle_socket_id_ < 0) {
        std::cerr << "[ZeusCanInterface] Failed to initialize muscle CAN: " << muscle_iface_name_ << std::endl;
        return false;
    }

    // Open the ESP32 highway
    sense_socket_id_ = init_socket(sense_iface_name_);
    if (sense_socket_id_ < 0) {
        std::cerr << "[ZeusCanInterface] Failed to initialize sense CAN: " << sense_iface_name_ << std::endl;
        return false;
    }

    std::cout << "[ZeusCanInterface] Successfully bound to " 
              << muscle_iface_name_ << " and " << sense_iface_name_ << "!" << std::endl;
    return true;
}

int ZeusCanInterface::init_socket(const std::string& iface_name) {
    // 1. Ask Linux for a Raw CAN socket. 
    // SOCK_NONBLOCK is crucial here so our 1000Hz loop never freezes waiting for data.
    int socket_fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_fd < 0) {
        return -1;
    }

    // 2. Translate the string name (e.g., "can0") into the OS's internal hardware index
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, iface_name.c_str());
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd);
        return -1;
    }

    // 3. Bind our software socket to that physical hardware index
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(socket_fd);
        return -1;
    }

    return socket_fd;
}

void ZeusCanInterface::deinit() {
    if (muscle_socket_id_ >= 0) {
        close(muscle_socket_id_);
        muscle_socket_id_ = -1;
    }
    if (sense_socket_id_ >= 0) {
        close(sense_socket_id_);
        sense_socket_id_ = -1;
    }
}

bool ZeusCanInterface::send_muscle_frame(const can_frame& frame) {
    if (muscle_socket_id_ < 0) return false;
    
    // Push the bytes directly to the Linux Kernel
    ssize_t nbytes = write(muscle_socket_id_, &frame, sizeof(can_frame));
    return (nbytes == sizeof(can_frame));
}

bool ZeusCanInterface::send_sense_frame(const can_frame& frame) {
    if (sense_socket_id_ < 0) return false;
    
    ssize_t nbytes = write(sense_socket_id_, &frame, sizeof(can_frame));
    return (nbytes == sizeof(can_frame));
}

void ZeusCanInterface::read_all_nonblocking() {
    // Keep emptying the inboxes until both are completely dry
    bool muscle_has_data = true;
    bool sense_has_data = true;

    while (muscle_has_data || sense_has_data) {
        muscle_has_data = read_socket_nonblocking(muscle_socket_id_, muscle_iface_name_);
        sense_has_data = read_socket_nonblocking(sense_socket_id_, sense_iface_name_);
    }
}

bool ZeusCanInterface::read_socket_nonblocking(int socket_fd, const std::string& iface_name) {
    if (socket_fd < 0) return false;

    struct can_frame frame;
    ssize_t nbytes = read(socket_fd, &frame, sizeof(can_frame));

    if (nbytes < 0) {
        // EAGAIN or EWOULDBLOCK just means the inbox is currently empty.
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;
        }
        std::cerr << "[ZeusCanInterface] Error reading from " << iface_name << std::endl;
        return false;
    }

    if (nbytes == sizeof(can_frame)) {
        // We successfully pulled a frame! Send it UP to the control layer.
        if (frame_callback_) {
            frame_callback_(iface_name, frame);
        }
        return true; // Return true to indicate we found data, so the while-loop checks again
    }

    return false;
}

} // namespace zeus_hardware