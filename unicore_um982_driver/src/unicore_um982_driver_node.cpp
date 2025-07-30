#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <unicore_um982_driver/pvtsln_data.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

using namespace drivers::serial_driver;
using namespace drivers::common;

class UnicoreDriverNode : public rclcpp::Node
{
public:
    UnicoreDriverNode() : Node("unicore_um982_driver"), io_context_(1)
    {
        RCLCPP_INFO(this->get_logger(), "Unicore UM982 Driver Node started");
        
        // Create ROS 2 publishers
        gps_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        gps_imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("gps/imu", 10);
        
        RCLCPP_INFO(this->get_logger(), "Created ROS 2 publishers for /gps/fix and /gps/imu");
        
        // Initialize serial communication
        initializeSerial();
        
        // Create timer for reading serial data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz as per requirements
            std::bind(&UnicoreDriverNode::readSerialData, this));
    }

    ~UnicoreDriverNode()
    {
        if (serial_driver_ && serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
    }

private:
    void initializeSerial()
    {
        try {
            // Hardcoded values for initial testing as per implementation plan
            std::string port = "/dev/ttyUSB0";
            uint32_t baudrate = 230400;
            
            RCLCPP_INFO(this->get_logger(), "Initializing serial port %s at %d baud", 
                        port.c_str(), baudrate);
            
            // Create serial port configuration
            SerialPortConfig config(
                baudrate,
                FlowControl::NONE,
                Parity::NONE,
                StopBits::ONE
            );
            
            // Create and initialize serial driver
            serial_driver_ = std::make_unique<SerialDriver>(io_context_);
            serial_driver_->init_port(port, config);
            serial_driver_->port()->open();
            
            if (serial_driver_->port()->is_open()) {
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
                
                // Configure the receiver with automatic settings
                configureReceiver();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial initialization failed: %s", e.what());
            throw;
        }
    }
    
    void readSerialData()
    {
        if (!serial_driver_ || !serial_driver_->port()->is_open()) {
            return;
        }
        
        try {
            std::vector<uint8_t> buffer(1024);
            size_t bytes_read = serial_driver_->port()->receive(buffer);
            
            if (bytes_read > 0) {
                // Convert to string and process line by line
                std::string data(buffer.begin(), buffer.begin() + bytes_read);
                processSerialData(data);
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading serial data: %s", e.what());
        }
    }
    
    void processSerialData(const std::string& data)
    {
        static std::string line_buffer;
        line_buffer += data;
        
        // Process complete lines
        size_t pos = 0;
        while ((pos = line_buffer.find('\n')) != std::string::npos) {
            std::string line = line_buffer.substr(0, pos);
            line_buffer.erase(0, pos + 1);
            
            // Remove carriage return if present
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            
            // Check if this is a PVTSLN message
            if (line.find("PVTSLN") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "Raw PVTSLN: %s", line.c_str());
                
                // Parse the PVTSLN message
                unicore_um982_driver::PVTSLNData parsed_data;
                if (unicore_um982_driver::parsePVTSLN(line, parsed_data)) {
                    RCLCPP_INFO(this->get_logger(), 
                        "Parsed PVTSLN - Status: %s, Lat: %.8f, Lon: %.8f, Alt: %.3f, Heading: %.2f, Sats: %d", 
                        parsed_data.position_status.c_str(),
                        parsed_data.latitude, 
                        parsed_data.longitude, 
                        parsed_data.altitude, 
                        parsed_data.heading,
                        parsed_data.num_satellites_tracked);
                    
                    // Publish ROS 2 messages
                    publishNavSatFix(parsed_data);
                    publishImu(parsed_data);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse PVTSLN message");
                }
            }
        }
    }
    
    void publishNavSatFix(const unicore_um982_driver::PVTSLNData& data)
    {
        auto msg = sensor_msgs::msg::NavSatFix();
        
        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = "gps";
        
        // Set position
        msg.latitude = data.latitude;
        msg.longitude = data.longitude;
        msg.altitude = data.altitude;
        
        // Set status based on position status
        if (data.position_status == "SINGLE") {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        } else if (data.position_status == "RTK_FIXED") {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        } else if (data.position_status == "RTK_FLOAT") {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        } else {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }
        
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        
        // Set covariance based on standard deviations from PVTSLN message
        // Position covariance is a 3x3 matrix (ENU - East, North, Up)
        // Convert standard deviations to variances (square them)
        double east_variance = data.sigma_longitude * data.sigma_longitude;
        double north_variance = data.sigma_latitude * data.sigma_latitude;
        double up_variance = data.sigma_altitude * data.sigma_altitude;
        
        // Use default values if sigma values are invalid or zero
        if (east_variance <= 0.0) east_variance = 1.0;
        if (north_variance <= 0.0) north_variance = 1.0;
        if (up_variance <= 0.0) up_variance = 1.0;
        
        msg.position_covariance[0] = east_variance;  // East
        msg.position_covariance[4] = north_variance; // North  
        msg.position_covariance[8] = up_variance;    // Up
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        
        gps_fix_publisher_->publish(msg);
    }
    
    void publishImu(const unicore_um982_driver::PVTSLNData& data)
    {
        auto msg = sensor_msgs::msg::Imu();
        
        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = "gps";
        
        // Convert heading to quaternion (heading is in degrees, convert to radians)
        double heading_rad = data.heading * M_PI / 180.0;
        geometry_msgs::msg::Quaternion quat = headingToQuaternion(heading_rad);
        msg.orientation = quat;
        
        // Set orientation covariance
        // For heading-only data, we have uncertainty only around Z-axis
        double heading_variance = 0.1; // ~5.7 degrees standard deviation
        msg.orientation_covariance[8] = heading_variance; // Z-axis rotation variance
        msg.orientation_covariance[0] = -1; // Mark X and Y as unknown
        msg.orientation_covariance[4] = -1;
        
        // Angular velocity and linear acceleration are not available from PVTSLN
        // Set all to zero and mark covariances as unknown
        msg.angular_velocity.x = 0.0;
        msg.angular_velocity.y = 0.0;
        msg.angular_velocity.z = 0.0;
        msg.angular_velocity_covariance[0] = -1;
        
        msg.linear_acceleration.x = 0.0;
        msg.linear_acceleration.y = 0.0;
        msg.linear_acceleration.z = 0.0;
        msg.linear_acceleration_covariance[0] = -1;
        
        gps_imu_publisher_->publish(msg);
    }
    
    geometry_msgs::msg::Quaternion headingToQuaternion(double heading_rad)
    {
        // Convert heading (yaw) to quaternion
        // Heading is rotation around Z-axis
        geometry_msgs::msg::Quaternion quat;
        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = std::sin(heading_rad / 2.0);
        quat.w = std::cos(heading_rad / 2.0);
        return quat;
    }
    
    void configureReceiver()
    {
        if (!serial_driver_ || !serial_driver_->port()->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot configure receiver: serial port not open");
            return;
        }
        
        try {
            RCLCPP_INFO(this->get_logger(), "Configuring UM982 receiver");
            
            // Complete configuration sequence from UM982-CONFIG.txt
            std::vector<std::string> config_commands = {
                "CONFIG COM3 230400",           // Set COM3 to 230400 baud
                "MODE ROVER SURVEY",            // Set receiver mode
                "unmask GPS",                   // Enable GPS constellation
                "unmask bds",                   // Enable BeiDou constellation  
                "unmask GLO",                   // Enable GLONASS constellation
                "unmask GAL",                   // Enable Galileo constellation
                "unmask QZSS",                  // Enable QZSS constellation
                "UNLOG",                        // Clear all existing logs
                "CONFIG SMOOTH RTKHEIGHT 2",    // Configure RTK height smoothing
                "CONFIG SMOOTH HEADING 2",      // Configure heading smoothing
                "CONFIG SMOOTH PSRVEL ENABLE",  // Enable pseudorange velocity smoothing
                "CONFIG RTK TIMEOUT 300",       // Set RTK timeout to 300 seconds
                "CONFIG RTK RELIABILITY 4 2",  // Set RTK reliability parameters
                "CONFIG HEADING FIXLENGTH",     // Enable fixed-length heading
                "CONFIG HEADING LENGTH 43 1",   // Set heading baseline length (43cm with 1cm tolerance)
                "CONFIG HEADING RELIABILITY 3", // Set heading reliability level
                "PVTSLNA COM3 0.05",           // Enable PVTSLN output at 20Hz (0.05s interval)
                "SAVECONFIG"                    // Save all configuration to non-volatile memory
            };
            
            RCLCPP_INFO(this->get_logger(), "Sending %zu configuration commands to UM982", config_commands.size());
            
            // Send each configuration command with proper timing
            for (size_t i = 0; i < config_commands.size(); ++i) {
                const auto& command = config_commands[i];
                std::string full_command = command + "\r\n";
                std::vector<uint8_t> command_data(full_command.begin(), full_command.end());
                
                serial_driver_->port()->send(command_data);
                RCLCPP_INFO(this->get_logger(), "Sent command %zu/%zu: %s", i+1, config_commands.size(), command.c_str());
                
                // Wait between commands to allow processing
                if (command == "SAVECONFIG") {
                    // Give extra time for save operation
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "UM982 receiver configuration complete");
            RCLCPP_INFO(this->get_logger(), "Configuration includes: Multi-constellation GNSS, RTK settings, heading configuration, and 20Hz PVTSLN output");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure receiver: %s", e.what());
        }
    }

    // Member variables
    IoContext io_context_;
    std::unique_ptr<SerialDriver> serial_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gps_imu_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnicoreDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
