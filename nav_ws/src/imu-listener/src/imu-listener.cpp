#include <cstdio>
#include <unistd.h>
#include <cstdlib>
#include <cstring>

#include "i2c/i2c.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

using namespace std::chrono_literals;


class IMU_Transporter {
public:
    IMU_Transporter(ros::NodeHandle* nodehandle) : nh_(*nodehandle) {
        while (i2c_init() != 0) {
            ROS_FATAL("Fail to open device");
            ros::Duration(0.5).sleep();
            ROS_INFO("Retrying...");
        }
        // Queue size = 10
        publisher_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
        timer_ = nh_.createTimer(ros::Duration(0.01), &IMU_Transporter::timer_callback, this);
        ros::Duration(0.5).sleep();
        ROS_INFO("Initiated!");
    }

    ~IMU_Transporter() {
        i2c_close(_i2c_bus);
        ROS_INFO("Closing I2C");
    }
    struct Quaternion {
        float q0;   // q0: w
        float q1;   // q1: x
        float q2;   // q2: y
        float q3;   // q3: z

        bool is_legal() {
            return (std::fabs(q0) < 1.0f) && (std::fabs(q1) < 1.0f) && (std::fabs(q2) < 1.0f) && (std::fabs(q3) < 1.0f);
        }
    };

    struct Vector3f {
        float x;
        float y;
        float z;

        bool is_finite() {
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        }

        bool is_valid() {
            return std::fabs(x) < 50.0f && std::fabs(y) < 50.0f && std::fabs(z) < 50.0f;
        }
    };

    static const size_t _buf_size{256};
    static size_t _data_count;
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
    ros::NodeHandle nh_;
    ros::Publisher  publisher_;
    ros::Timer timer_;

    /* I2C */
    static const unsigned int _imu_addr{0x66};  // Address of the imu data provider

    int _i2c_bus;
    I2CDevice _device;
    I2C_READ_HANDLE i2c_read_handle = i2c_ioctl_read;

    SensorData _raw_data;

    sensor_msgs::Imu _imu_msg;

    int i2c_init() {
        /* Open i2c bus */
        // I2C Bus 1 SDA is on Pin 3
        // I2C Bus 1 SCL is on Pin 5
        const char bus_name[] = "/dev/i2c-1";

        if ((_i2c_bus = i2c_open(bus_name)) == -1) {
            ROS_FATAL("Open i2c bus:%s error!\n", bus_name);
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

    void timer_callback(const ros::TimerEvent& e)
    {
        static auto last_update_time = ros::Time::now();
        static int count = 0;
        static int sent = 0;
        if (_read_from_i2c() > 0) {
            publisher_.publish(_imu_msg);
            sent += 1;
        }
        if (count >= 100) {
            auto duration = ros::Time::now() - last_update_time;
            last_update_time = ros::Time::now();
            ROS_INFO("Polling FPS: %f, ratio: %d / 100", 100 / duration.toSec(), sent);
            count = 0;
            sent = 0;
        }
        count += 1;
    }

    /* Store the readings and return the number of bytes received. 
     */
    int _read_from_i2c() {
        ssize_t ret = 0;

        ret = i2c_read_handle(&_device, 0x0, _raw_data.raw, sizeof(SensorData::Data));
        if (ret == -1) {
            ROS_FATAL("Read i2c error!\n");
        }
        else if ((size_t)ret != sizeof(SensorData::Data)) {
            ROS_FATAL("Data length not equal!\n");
        }
        else {
            // Successfully read sensor data
            if (!_raw_data.data.orientation.is_legal() || !_raw_data.data.acc.is_finite() || !_raw_data.data.gyro.is_finite()) {
                return -1; 
            }

            if (!_raw_data.data.acc.is_valid() || !_raw_data.data.gyro.is_valid()) {
                return -1; 
            }
            
            _imu_msg.header.stamp = ros::Time::now();
            _imu_msg.header.seq = _data_count;
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
            _data_count++;
            return ret;
        }
        return -1;
    }
};

size_t IMU_Transporter::_data_count{0};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu_listener");
    ros::NodeHandle nh;
    IMU_Transporter transporter(&nh);
    
    ros::spin();
    ros::shutdown();
    
    return 0;
}
