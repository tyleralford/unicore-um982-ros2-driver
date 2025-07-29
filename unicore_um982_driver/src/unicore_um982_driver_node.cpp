#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <memory>
#include <string>
#include <vector>

using namespace drivers::serial_driver;
using namespace drivers::common;

class UnicoreDriverNode : public rclcpp::Node
{
public:
    UnicoreDriverNode() : Node("unicore_um982_driver"), io_context_(1)
    {
        RCLCPP_INFO(this->get_logger(), "Unicore UM982 Driver Node started");
        
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
                RCLCPP_INFO(this->get_logger(), "PVTSLN: %s", line.c_str());
            }
        }
    }

    // Member variables
    IoContext io_context_;
    std::unique_ptr<SerialDriver> serial_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnicoreDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
