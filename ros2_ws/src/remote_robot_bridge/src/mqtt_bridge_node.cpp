#include <mosquitto.h>
#include <string.h>
#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>        //Jason parsing as required
#include <mutex>          //prevent premature read by ros2 timer
#include <algorithm>
#include <atomic>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;  //write 20Hz directly
using json = nlohmann::json;  //ease

class MqttBridgeNode : public rclcpp::Node
{
public: 
    MqttBridgeNode() : Node("mqtt_bridge")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); //create publisher - publish msg "/cmd_vel" with a depth 10
        timer_ = this ->create_wall_timer(
            50ms,
            std::bind(&MqttBridgeNode::publishTwist, this));  //create timer - call publisher every 50 ms ie 20times/sec
        state_timer_ = this->create_wall_timer(
            500ms, std::bind(&MqttBridgeNode::publishState, this));   // 2 Hz readable time
        RCLCPP_INFO(this->get_logger(), "MQTT Bridge Node Started");
        
        setupMqtt();
    }

    ~MqttBridgeNode() override
    {
        if (mosq_) {
        mosquitto_loop_stop(mosq_, true);   // stop the network thread
        mosquitto_destroy(mosq_);
        mosquitto_lib_cleanup();
        }
    }
//ROS2 thread
private:
    void publishTwist()
    {
        double lin = 0.0, ang = 0.0;
        bool fallback = true;
        std::string reason = "ok";              // <-- HERE, outside the block
        const auto now = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::milliseconds(1000);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            auto cmd_age_ = now - last_cmd_time_;
            auto heartbeat_age_ = now - last_heartbeat_time_;

            if (!connected_) {
                reason = "mqtt_disconnected";
            } else if (last_cmd_time_.time_since_epoch().count() == 0) {
                reason = "no_command_yet";
            } else if (cmd_age_ > timeout) {
                reason = "command_old";
            } else if (heartbeat_age_ > timeout) {
                reason = "heartbeat_old";
            } else if (!valid_) {
                reason = "invalid_command";
            } else if (!deadman_) {
                reason = "deadman_is_false";
            } else {
                fallback = false;
                lin = linear_;
                ang = angular_;
            }
        }   // lock released here
     // clamp because - five conditions satisfies but error values must be filtered 
        lin = std::clamp(lin, -0.5, 0.5);
        ang = std::clamp(ang, -1.0, 1.0);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "fallback=%s reason=%s lin=%.2f ang=%.2f",
            fallback ? "true" : "false", reason.c_str(), lin, ang);

        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x  = fallback ? 0.0 : lin;
        msg.angular.z = fallback ? 0.0 : ang;
        pub_->publish(msg);
    }
// robot state publish
    void publishState()
    {
        if (!connected_) return;

        bool fallback = true;
        std::string reason = "ok";
        int64_t cmd_age_ms = -1;
        const auto now = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::milliseconds(1000);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            auto cmd_age_ = now - last_cmd_time_;
            auto heartbeat_age_ = now - last_heartbeat_time_;

            if (last_cmd_time_.time_since_epoch().count() == 0) {
                reason = "no_command_yet";
            } else {
                cmd_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cmd_age_).count();
                if (cmd_age_ > timeout)              reason = "command_old";
                else if (heartbeat_age_ > timeout)   reason = "heartbeat_old";
                else if (!valid_)                    reason = "invalid_command";
                else if (!deadman_)                  reason = "deadman_is_false";
                else                                 fallback = false;
            }
        }

        json s;
        s["mode"]            = fallback ? "FALLBACK" : "REMOTE_CONTROL";
        s["fallback_active"] = fallback;
        s["last_cmd_age_ms"] = cmd_age_ms;
        s["reason"]          = reason;
        std::string payload = s.dump();  //jsoon reverse (object to text)

        mosquitto_publish(mosq_, nullptr, "remote_robot/robotthinkit/state",
                          static_cast<int>(payload.size()), payload.data(), 0, false);
    }
// MQTT thread
    static void onMessageStatic(struct mosquitto * /*mosq*/, void *obj, const struct mosquitto_message *msg)
    {
        static_cast<MqttBridgeNode *>(obj)->onMessage(msg);     //Libmosquitto - c, no object -  need static for pointer *obj = this
    }

    void onMessage(const struct mosquitto_message *msg)  //above function links to object ->this and here the obj can access this (message, its length)
    {
        std::string payload(static_cast<char *>(msg->payload), msg->payloadlen);

        std::string topic(msg->topic);

        //since message is in Json fortmat - parsing here - because, the MQTT writes to the member variable lin, ang from here
        json j = json::parse(payload, nullptr, false);  //parse text to JSON object, no exception, instead special discarded value
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if(topic == "remote_robot/robotthinkit/cmd") {
            bool ok = !j.is_discarded() 
                    && j.contains("linear_x") && j["linear_x"].is_number()
                    && j.contains("angular_z") && j["angular_z"].is_number()
                    && j.contains("deadman") && j["deadman"].is_boolean();

            if (ok) {
                linear_ = j["linear_x"];
                angular_ = j["angular_z"];
                deadman_ = j["deadman"];
                valid_ = true;
            } else {
                valid_ = false;
                RCLCPP_WARN(get_logger(), "invalid command");
            }
            last_cmd_time_ = std::chrono::steady_clock::now();
        }
        else if (topic == "remote_robot/robotthinkit/esp32_heartbeat") {
            last_heartbeat_time_ =  std::chrono::steady_clock::now();
        }
    }

    ///robot state back - subscribe in here because we need the robot state back and then continue
    // whether or not connection/reconnection has happened
    static void onConnectStatic(struct mosquitto * /*mosq*/, void *obj, int rc)
    {
        static_cast<MqttBridgeNode *>(obj)->onConnect(rc);
    }

    void onConnect(int rc)
    {
        if (rc != 0) {
            RCLCPP_WARN(get_logger(), "MQTT connect failed, rc=%d", rc);
            return;
        }
        connected_ = true;
        
        mosquitto_subscribe(mosq_, nullptr, "remote_robot/robotthinkit/cmd", 0);
        mosquitto_subscribe(mosq_, nullptr, "remote_robot/robotthinkit/esp32_heartbeat", 0);
        RCLCPP_INFO(get_logger(), "MQTT connected and subscribed");
    }

    static void onDisconnectStatic(struct mosquitto * /*mosq*/, void *obj, int rc)
    {
        static_cast<MqttBridgeNode *>(obj)->onDisconnect(rc);
    }

    void onDisconnect(int rc)
    {
        connected_ = false;
        RCLCPP_WARN(get_logger(), "MQTT disconnected, rc=%d", rc);
    }

    void setupMqtt()
    {
        mosquitto_lib_init();
//need unique id per process
        std::string client_id = "ros2-bridge-" + std::to_string(getpid());
        mosq_ = mosquitto_new(client_id.c_str(), true, this);
        
        ///include connect &   disconnect static to link subscription
        mosquitto_connect_callback_set(mosq_, &MqttBridgeNode::onConnectStatic);
        mosquitto_disconnect_callback_set(mosq_, &MqttBridgeNode::onDisconnectStatic);
        mosquitto_message_callback_set(mosq_, &MqttBridgeNode::onMessageStatic);

    
        mosquitto_connect(mosq_, "broker.hivemq.com", 1883, 60);

       
        /*mosquitto_subscribe(mosq_, nullptr, "remote_robot/robotthinkit/cmd", 0);
        mosquitto_subscribe(mosq_, nullptr, "remote_robot/robotthinkit/esp32_heartbeat", 0);*/

        // loop MQTT
        mosquitto_loop_start(mosq_);

        //RCLCPP_INFO(get_logger(), "MQTT connected and subscribed");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    struct mosquitto *mosq_{nullptr};
    std::atomic<bool> connected_{false};

    std::mutex data_mutex_;
    double linear_{0.0};
    double angular_{0.0};   //latest messages

    //deadmans switch - remote operation safety check
    bool deadman_{false};
    bool valid_{false};

    std::chrono::steady_clock::time_point last_cmd_time_{};  //last command   steady clock for reliable rate (monotonic) 
    std::chrono::steady_clock::time_point last_heartbeat_time_{}; //heartbeat

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                  
    auto node = std::make_shared<MqttBridgeNode>();                                     
    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}