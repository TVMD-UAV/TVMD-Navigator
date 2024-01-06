#include <cstdio>
#include <unistd.h>
#include <cstdlib>
#include <cstring>

#include <i2c/i2c.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;


class IMU_Transporter : public rclcpp::Node {
public:
    IMU_Transporter() : Node("IMU_Transpoter"), count_(0), _imu_msg()
    {
        
        while (i2c_init() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Fail to open device");
            rclcpp::sleep_for(500ms);
            RCLCPP_INFO(this->get_logger(), "Retrying...");
        }
        // Queue size = 10
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        timer_ = this->create_wall_timer(
            40ms, std::bind(&IMU_Transporter::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Initiated!");
    }

    ~IMU_Transporter() {
        i2c_close(_i2c_bus);
        RCLCPP_INFO(this->get_logger(), "Closing I2C");
    }
    struct Quaternion {
        float q0;   // q0: w
        float q1;   // q1: x
        float q2;   // q2: y
        float q3;   // q3: z
    };

    struct Vector3f {
        float x;
        float y;
        float z;
    };

    static const size_t _buf_size{256};
    union SensorData {
        struct Data{
            Quaternion orientation;
            Vector3f acc;
            Vector3f gyro;
            Vector3f compass;

            float temperature;
            float pressure;
            float altitude;
        } data;
        uint8_t raw[_buf_size];
    };

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    size_t count_;

    /* I2C */
    static const unsigned int _imu_addr{0x66};  // Address of the imu data provider

    int _i2c_bus{-1};
    I2CDevice _device;
    I2C_READ_HANDLE i2c_read_handle = i2c_ioctl_read;

    SensorData _raw_data;

    sensor_msgs::msg::Imu _imu_msg;

    int i2c_init() {
        /* Open i2c bus */
        // I2C Bus 1 SDA is on Pin 3
        // I2C Bus 1 SCL is on Pin 5
        const char bus_name[] = "/dev/i2c-1";

        if ((_i2c_bus = i2c_open(bus_name)) == -1) {
            RCLCPP_FATAL(this->get_logger(), "Open i2c bus:%s error!\n", bus_name);
            return -1;
        }

        memset(&_device, 0, sizeof(_device));
        i2c_init_device(&_device);

        _device.bus = _i2c_bus;
        _device.addr = _imu_addr & 0x3ff;
        _device.page_bytes = 0;         // No writing is required
        _device.iaddr_bytes = 0;        // Having no internal address
        return 0;
    }

    void timer_callback()
    {
        if (_read_from_i2c() > 0) {
            publisher_->publish(_imu_msg);
        }
    }

    /* Store the readings and return the number of bytes received. 
     */
    int _read_from_i2c() {
        ssize_t ret = 0;

        ret = i2c_read_handle(&_device, 0x0, _raw_data.raw, sizeof(SensorData::Data));
        if (ret == -1) {
            RCLCPP_FATAL(this->get_logger(), "Read i2c error!\n");
        }
        else if ((size_t)ret != sizeof(SensorData::Data)) {
            RCLCPP_FATAL(this->get_logger(), "Data length not equal!\n");
        }
        else {
            // Successfully read sensor data
            _imu_msg.orientation.w = _raw_data.data.orientation.q0;
            _imu_msg.orientation.x = _raw_data.data.orientation.q1;
            _imu_msg.orientation.y = _raw_data.data.orientation.q2;
            _imu_msg.orientation.z = _raw_data.data.orientation.q3;
            _imu_msg.linear_acceleration.x = _raw_data.data.acc.x;
            _imu_msg.linear_acceleration.y = _raw_data.data.acc.y;
            _imu_msg.linear_acceleration.z = _raw_data.data.acc.z;
            _imu_msg.angular_velocity.x = _raw_data.data.gyro.x;
            _imu_msg.angular_velocity.y = _raw_data.data.gyro.y;
            _imu_msg.angular_velocity.z = _raw_data.data.gyro.z;
        }
        return ret;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMU_Transporter>());
    rclcpp::shutdown();
    
    return 0;
}


// void print_i2c_data(const unsigned char *data, size_t len)
// {
//     size_t i = 0;

//     for (i = 0; i < len; i++) {

//         if (i % 16 == 0) {

//             fprintf(stdout, "\n");
//         }

//         fprintf(stdout, "%02x ", data[i]);
//     }

//     fprintf(stdout, "\n");
// }
