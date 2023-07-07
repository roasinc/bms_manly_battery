#include "rclcpp/rclcpp.hpp"

#include <csignal>
#include <fcntl.h>
#include <libserial/SerialPort.h>
#include <sensor_msgs/msg/battery_state.hpp>


class BMSManlyBatteryNode: public rclcpp::Node
{
    public:
        BMSManlyBatteryNode(): Node("bms_manly_battery_node")
        {
            this->declare_parameter<std::string>("port_name", "/dev/ttyS1");
            this->declare_parameter<int>("baudrate", 9600);
            this->declare_parameter<double>("rate", 5.0);

            auto port_name = this->get_parameter("port_name").get_parameter_value().get<std::string>();
            auto baudrate = this->get_parameter("baudrate").get_parameter_value().get<uint32_t>();

            RCLCPP_INFO(this->get_logger(), "Port: [%s] and baudrate %ld", port_name.c_str(), baudrate);

            try
            {
                ser_.Open(port_name);
            }
            catch(LibSerial::OpenFailed &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[0;91m%s\033[0m", e.what());
                RCLCPP_ERROR(this->get_logger(), "\033[0;91mFailed to open port\033[0m [\033[0;92m%s\033[0m]...", port_name.c_str());
                exit(-1);
            }

            auto ser_baudrate = LibSerial::BaudRate::BAUD_9600;
            switch(baudrate)
            {
                case 9600:   ser_baudrate = LibSerial::BaudRate::BAUD_9600;   break;
                case 19200:  ser_baudrate = LibSerial::BaudRate::BAUD_19200;  break;
                case 38400:  ser_baudrate = LibSerial::BaudRate::BAUD_38400;  break;
                case 57600:  ser_baudrate = LibSerial::BaudRate::BAUD_57600;  break;
                case 115200: ser_baudrate = LibSerial::BaudRate::BAUD_115200; break;
            }
            ser_.SetBaudRate(ser_baudrate);
            ser_.FlushIOBuffers();
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            // Get SerialNumber of Battery
            std::vector<uint8_t> send_packet {0xdd, 0xa5, 0x05, 0x00, 0xff, 0xfb, 0x77};
            ser_.Write(send_packet);
            ser_.DrainWriteBuffer();

            std::vector<uint8_t> recv_header(4, 0);
            int data_len = 0;
            try
            {
                ser_.Read(recv_header, 4, 200);
                if(recv_header[0] == 0xdd && recv_header[1] == 0x05)
                {
                    if(recv_header[2] == 0x0)
                    {
                        data_len = recv_header[3];
                        RCLCPP_DEBUG(this->get_logger(), "Receive data length: %d", data_len);
                    }
                    else
                    {
                        ser_.FlushIOBuffers();
                        return;
                    }
                }
            }
            catch(LibSerial::ReadTimeout &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[91m%s\033[0m", e.what());
                ser_.FlushIOBuffers();
                return;
            }

            std::vector<uint8_t> recv_data(data_len + 3, 0);
            try
            {
                ser_.Read(recv_data, data_len + 3, 100);
                if(recv_data[data_len + 3 - 1] == 0x77)
                {
                    for(int i = 0; i < data_len; i++)
                    {
                        batt_serial_number_ += (char)recv_data[i];
                    }
                }
            }
            catch(LibSerial::ReadTimeout &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[91m%s\033[0m", e.what());
                ser_.FlushIOBuffers();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Battery [%s] is ready...", batt_serial_number_.c_str());


            pub_batt_state_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SystemDefaultsQoS());

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&BMSManlyBatteryNode::timer_callback, this));
        }

        ~BMSManlyBatteryNode() {}

    private:
        void timer_callback()
        {
            auto batt_msg = sensor_msgs::msg::BatteryState();

            batt_msg.location = "main";
            batt_msg.serial_number = batt_serial_number_;
            batt_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            batt_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
            batt_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;


            // Battery Status: 0x03
            std::vector<uint8_t> send_packet1 {0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77};
            ser_.Write(send_packet1);
            ser_.DrainWriteBuffer();

            std::vector<uint8_t> recv_header1(4, 0);
            int data_len = 0;
            try
            {
                ser_.Read(recv_header1, 4, 200);
                if(recv_header1[0] == 0xdd && recv_header1[1] == 0x03)
                {
                    if(recv_header1[2] == 0x0)
                    {
                        data_len = recv_header1[3];
                        RCLCPP_DEBUG(this->get_logger(), "Receive data length: %d", data_len);
                    }
                    else
                    {
                        ser_.FlushIOBuffers();
                        return;
                    }
                }
            }
            catch(LibSerial::ReadTimeout &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[91m%s\033[0m", e.what());
                ser_.FlushIOBuffers();
                return;
            }

            std::vector<uint8_t> recv_data1(data_len + 3, 0);
            try
            {
                ser_.Read(recv_data1, data_len + 3, 100);
                if(recv_data1[data_len + 3 - 1] == 0x77)
                {
                    // double batt_volt = ((int16_t)(recv_data1[0] << 8) + recv_data1[1]) / 100.0;
                    // double batt_current = ((int16_t)((recv_data1[2] << 8) + recv_data1[3])) / 100.0;
                    // double remained_cap = ((int16_t)((recv_data1[4] << 8) + recv_data1[5])) / 100.0;
                    // double design_cap = ((int16_t)((recv_data1[6] << 8) + recv_data1[7])) / 100.0;
                    // int16_t charging_count = (int16_t)((recv_data1[8] << 8) + recv_data1[9]);
                    // int16_t manufactured_date = (int16_t)((recv_data1[10] << 8) + recv_data1[11]);
                    // int16_t offset_status_low = (int16_t)((recv_data1[12] << 8) + recv_data1[13]);
                    // int16_t offset_status_high = (int16_t)((recv_data1[14] << 8) + recv_data1[15]);
                    // int16_t safe_status = (int16_t)((recv_data1[16] << 8) + recv_data1[17]);
                    // int8_t software_version = recv_data1[18];
                    // int8_t rsoc = recv_data1[19];
                    // int8_t fet_status = recv_data1[20];
                    // int8_t batt_cell_count = recv_data1[21];
                    // int8_t ntc_number = recv_data1[22];
                    // uint16_t temp1 = (uint16_t)((recv_data1[23] << 8) + recv_data1[24]);
                    // uint16_t temp2 = (uint16_t)((recv_data1[25] << 8) + recv_data1[26]);

                    batt_msg.voltage = ((int16_t)(recv_data1[0] << 8) + recv_data1[1]) / 100.0;
                    batt_msg.temperature = ((uint16_t)((recv_data1[23] << 8) + recv_data1[24]) - 2731) / 10.0;
                    batt_msg.current = ((int16_t)((recv_data1[2] << 8) + recv_data1[3])) / 100.0;

                    batt_msg.power_supply_status = (batt_msg.current > 0.0)
                        ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

                    batt_msg.charge = (batt_msg.current > 0.0) ? batt_msg.current : 0.0;
                    batt_msg.capacity = ((int16_t)((recv_data1[4] << 8) + recv_data1[5])) / 100.0;
                    batt_msg.design_capacity = ((int16_t)((recv_data1[6] << 8) + recv_data1[7])) / 100.0;
                    batt_msg.percentage = recv_data1[19] * 1.0;
                    batt_msg.cell_voltage.resize(recv_data1[21], 0.0);
                    batt_msg.cell_temperature.resize(recv_data1[21], 0.0);
                }
            }
            catch(LibSerial::ReadTimeout &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[91m%s\033[0m", e.what());
                ser_.FlushIOBuffers();
                return;
            }

            // Cell Voltage: 0x04
            std::vector<uint8_t> send_packet2 {0xdd, 0xa5, 0x04, 0x00, 0xff, 0xfc, 0x77};
            ser_.Write(send_packet2);
            ser_.DrainWriteBuffer();

            std::vector<uint8_t> recv_header2(4, 0);
            data_len = 0;
            try
            {
                ser_.Read(recv_header2, 4, 200);
                if(recv_header2[0] == 0xdd && recv_header2[1] == 0x04)
                {
                    if(recv_header2[2] == 0x0)
                    {
                        data_len = recv_header2[3];
                        RCLCPP_DEBUG(this->get_logger(), "Receive data length: %d", data_len);
                    }
                    else
                    {
                        ser_.FlushIOBuffers();
                        return;
                    }
                }
            }
            catch(LibSerial::ReadTimeout &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[91m%s\033[0m", e.what());
                ser_.FlushIOBuffers();
                return;
            }

            std::vector<uint8_t> recv_data2(data_len + 3, 0);
            try
            {
                ser_.Read(recv_data2, data_len + 3, 100);
                if(recv_data2[data_len + 3 - 1] == 0x77)
                {
                    for(size_t i = 0; i < batt_msg.cell_voltage.size(); i++)
                    {
                        batt_msg.cell_voltage[i] = ((int16_t)(recv_data2[i*2] << 8) + recv_data2[i*2+1]) / 1000.0;
                    }
                }
            }
            catch(LibSerial::ReadTimeout &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[91m%s\033[0m", e.what());
                ser_.FlushIOBuffers();
                return;
            }

            batt_msg.header.stamp = this->now();
            pub_batt_state_->publish(batt_msg);
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        LibSerial::SerialPort ser_;
        std::string batt_serial_number_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> pub_batt_state_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BMSManlyBatteryNode>());
    rclcpp::shutdown();
    return 0;
}