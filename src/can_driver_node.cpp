#include <linux/can.h>
#include <linux/can/raw.h>

#include <endian.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ros/ros.h"
#include <bitten/canopen_msg.h>
#include <global_node_definitions.h>
#include <can_driver_node.h>

#define PROGNAME  "socketcan"
#define VERSION  "0.1.0"

namespace
{

std::sig_atomic_t signalValue;

void onSignal(int value)
{
    signalValue = static_cast<decltype(signalValue)>(value);
}

void usage()
{
    std::cout << "Usage: " PROGNAME " [-h] [-V] [-f] interface" << std::endl
              << "Options:" << std::endl
              << "  -h  Display this information" << std::endl
              << "  -V  Display version information" << std::endl
              << "  -f  Run in the foreground" << std::endl
              << std::endl;
}

void version()
{
    std::cout << PROGNAME " version " VERSION << std::endl
              << "Compiled on " __DATE__ ", " __TIME__ << std::endl
              << std::endl;
}

void processFrame(const struct canfd_frame& frame)
{
    switch (frame.can_id)
    {
    case 0x8CFDD633:
    case 0x8CFDD634:
    case 0x8CFDD733:
    case 0x8CFDD734:
    {
        std::cout << "CAN-ID: 0x" << std::hex << std::uppercase
                  << std::setw(3) << std::setfill('0')
                  << frame.can_id << std::endl;
        std::cout << "Length: " << (int)frame.len << std::endl; //payload is 8 bytes in length
        std::cout << "Data: ";
        for(int i = 0; i < (int)frame.len; i++)
        {
            std::cout << (int)frame.data[i];
        }
        std::cout << std::endl;
    }
        break;

    default:
        std::cerr << "Unexpected CAN ID: 0x"
                  << std::hex << std::uppercase
                  << std::setw(3) << std::setfill('0')
                  << frame.can_id << std::endl;
        std::cerr.copyfmt(std::ios(nullptr));
        break;
    }
}

} // namespace

int main(int argc, char** argv)
{   
    using namespace std::chrono_literals;

    ROS_INFO("Initiating system...");
    ros::init(argc, argv, "can_driver_node");
    ros::NodeHandle n;

    ROS_INFO("Publishing to can_topic");
    ros::Publisher canPub = n.advertise<bitten::canopen_msg>("can_topic", 1000);

    ros::Rate loop_rate(LOOP_RATE_INT);

    // Options
    const char* interface;
    bool foreground = false;

    // Service variables
    struct sigaction sa;
    int rc;

    // CAN connection variables
    struct sockaddr_can addr;
    struct ifreq ifr;
    int sockfd;

    // Parse command line arguments
    {
        int opt;

        // Parse option flags
        while ((opt = ::getopt(argc, argv, "Vfh")) != -1)
        {
            switch (opt)
            {
            case 'V':
                version();
                return EXIT_SUCCESS;
            case 'f':
                foreground = true;
                break;
            case 'h':
                usage();
                return EXIT_SUCCESS;
            default:
                usage();
                return EXIT_FAILURE;
            }
        }

        // Check for the one positional argument
        if (optind != (argc - 1))
        {
            std::cerr << "Missing network interface option!" << std::endl;
            usage();
            return EXIT_FAILURE;
        }

        // Set the network interface to use
        interface = argv[optind];
    }

    // Check if the service should be run as a daemon
    if (!foreground)
    {
        if (::daemon(0, 1) == -1)
        {
            std::perror("daemon");
            return EXIT_FAILURE;
        }
    }

    // Register signal handlers
    sa.sa_handler = onSignal;
    ::sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    ::sigaction(SIGINT, &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);
    ::sigaction(SIGQUIT, &sa, nullptr);
    ::sigaction(SIGHUP, &sa, nullptr);

    // Initialize the signal value to zero
    signalValue = 0;

    // Open the CAN network interface
    sockfd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (-1 == sockfd)
    {
        std::perror("socket");
        return errno;
    }

    // Set a receive filter so we only receive select CAN IDs
    {
        struct can_filter filter[4];
        filter[0].can_id   = 0xCFDD633;
        filter[0].can_mask = CAN_SFF_MASK;
        filter[1].can_id   = 0xCFDD634;
        filter[1].can_mask = CAN_SFF_MASK;
        filter[2].can_id   = 0xCFDD733;
        filter[2].can_mask = CAN_SFF_MASK;
        filter[3].can_id   = 0xCFDD734;
        filter[3].can_mask = CAN_SFF_MASK;

        rc = ::setsockopt(
            sockfd,
            SOL_CAN_RAW,
            CAN_RAW_FILTER,
            &filter,
            sizeof(filter)
        );
        if (-1 == rc)
        {
            std::perror("setsockopt filter");
            ::close(sockfd);
            return errno;
        }
    }

    // Enable reception of CAN FD frames
    {
        int enable = 1;

        rc = ::setsockopt(
            sockfd,
            SOL_CAN_RAW,
            CAN_RAW_FD_FRAMES,
            &enable,
            sizeof(enable)
        );
        if (-1 == rc)
        {
            std::perror("setsockopt CAN FD");
            ::close(sockfd);
            return errno;
        }
    }

    // Get the index of the network interface
    std::strncpy(ifr.ifr_name, interface, IFNAMSIZ);
    if (::ioctl(sockfd, SIOCGIFINDEX, &ifr) == -1)
    {
        std::perror("ioctl");
        ::close(sockfd);
        return errno;
    }

    // Bind the socket to the network interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    rc = ::bind(
        sockfd,
        reinterpret_cast<struct sockaddr*>(&addr),
        sizeof(addr)
    );
    if (-1 == rc)
    {
        std::perror("bind");
        ::close(sockfd);
        return errno;
    }

    // Log that the service is up and running
    std::cout << "Started" << std::endl;

    // Main loop
    while ( ( 0 == signalValue ) && ( ros::ok() ) )
    {
        struct canfd_frame frame;

        // Read in a CAN frame
        auto numBytes = ::read(sockfd, &frame, CANFD_MTU);
        switch (numBytes)
        {
        case CAN_MTU:
            processFrame(frame);
            break;
        case CANFD_MTU:
            break;
        case -1:
            // Check the signal value on interrupt
            if (EINTR == errno)
                continue;

            // Delay before continuing
            std::perror("read");
            std::this_thread::sleep_for(100ms);
        default:
            continue;
        }
    }

    // Cleanup
    if (::close(sockfd) == -1)
    {
        std::perror("close");
        return errno;
    }

    std::cout << std::endl << "Bye!" << std::endl;
    return EXIT_SUCCESS;
}
