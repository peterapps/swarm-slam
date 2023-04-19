#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using tf2_msgs::msg::TFMessage;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::NavSatFix;
using geometry_msgs::msg::TwistStamped;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using std::string;
using std::bind;

#define MAKE_PUB(TYPE, TOPIC, QOS) create_publisher<TYPE>(TOPIC, QOS);

#define MAKE_SUB(TYPE, TOPIC, CALLBACK, QOS) \
    create_subscription<TYPE>(TOPIC, QOS, \
            bind(&KittiRemapTime::CALLBACK, this, _1));

#define MAKE_CALLBACK(NAME, PUB, TYPE) void NAME(const TYPE::SharedPtr msg){\
        TYPE msgOut = *msg; \
        msgOut.header.stamp = offset + msgOut.header.stamp; \
        PUB->publish(msgOut); \
    }

#define MAKE_TF_CALLBACK(NAME, PUB) void NAME(const TFMessage::SharedPtr msg){\
        TFMessage msgOut = *msg; \
        for (auto &transform : msgOut.transforms){ \
            transform.header.stamp = offset + transform.header.stamp; \
        } \
        PUB->publish(msgOut); \
    }

/*#define MAKE_PUB_SUB(T, SUB, PUB, SUB_TOPIC, PUB_TOPIC, QOS) { \
        this->PUB = create_publisher<T>(PUB_TOPIC, QOS); \
        this->SUB = create_subscription<T>(SUB_TOPIC, QOS, \
            [this](const auto msg){ \
                T msgOut = *msg; \
                msg.header.stamp += this->offset; \
                this->PUB->publish(msgOut); \
            }); \
    }*/

/*#define MAKE_PUB_SUB(TYPE, SUB, PUB, SUB_TOPIC, PUB_TOPIC, QOS) { \
        PUB = create_publisher<TYPE>(PUB_TOPIC, QOS); \
        SUB = create_subscription<TYPE>(SUB_TOPIC, QOS, \
            bind(&KittiRemapTime::headerCallback<rclcpp::Publisher<TYPE>::SharedPtr, TYPE>, PUB, _1)); \
    }*/

class KittiRemapTime : public rclcpp::Node {
private:
    rclcpp::Duration offset;

    rclcpp::Publisher<TFMessage>::SharedPtr tfPub;
    rclcpp::Publisher<TFMessage>::SharedPtr tfStaticPub;
    rclcpp::Publisher<Imu>::SharedPtr imuPub;
    rclcpp::Publisher<NavSatFix>::SharedPtr gpsPub;
    rclcpp::Publisher<TwistStamped>::SharedPtr velPub;
    rclcpp::Publisher<PointCloud2>::SharedPtr lidarPub;

    rclcpp::Subscription<TFMessage>::SharedPtr tfSub;
    rclcpp::Subscription<TFMessage>::SharedPtr tfStaticSub;
    rclcpp::Subscription<Imu>::SharedPtr imuSub;
    rclcpp::Subscription<NavSatFix>::SharedPtr gpsSub;
    rclcpp::Subscription<TwistStamped>::SharedPtr velSub;
    rclcpp::Subscription<PointCloud2>::SharedPtr lidarSub;

    MAKE_TF_CALLBACK(tfCallback, tfPub)
    MAKE_TF_CALLBACK(tfStaticCallback, tfStaticPub)
    MAKE_CALLBACK(imuCallback, imuPub, Imu)
    MAKE_CALLBACK(gpsCallback, gpsPub, NavSatFix)
    MAKE_CALLBACK(velCallback, velPub, TwistStamped)
    MAKE_CALLBACK(lidarCallback, lidarPub, PointCloud2)
public:
    KittiRemapTime() : Node("kitti_remap_time"), offset(0, 0) {
        // Read parameters
        string kOffset("offset_sec");
        declare_parameter(kOffset);
        double offsetSec = get_parameter(kOffset).as_double();
        offset = rclcpp::Duration::from_seconds(offsetSec);

        // Publishers and subscribers
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        tfPub = MAKE_PUB(TFMessage, "/tf_out", qos);
        tfStaticPub = MAKE_PUB(TFMessage, "/tf_static_out", qos);
        imuPub = MAKE_PUB(Imu, "/imu_out", qos);
        gpsPub = MAKE_PUB(NavSatFix, "/gps_out", qos);
        velPub = MAKE_PUB(TwistStamped, "/vel_out", qos);
        lidarPub = MAKE_PUB(PointCloud2, "/lidar_out", qos);

        tfSub = MAKE_SUB(TFMessage, "/tf_in", tfCallback, qos);
        tfStaticSub = MAKE_SUB(TFMessage, "/tf_static_in", tfCallback, qos);
        imuSub = MAKE_SUB(Imu, "/imu_in", imuCallback, qos);
        gpsSub = MAKE_SUB(NavSatFix, "/gps_in", gpsCallback, qos);
        velSub = MAKE_SUB(TwistStamped, "/vel_in", velCallback, qos);
        lidarSub = MAKE_SUB(PointCloud2, "/lidar_in", lidarCallback, qos);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KittiRemapTime>());
    rclcpp::shutdown();
}
