#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Vector3.hh>
#include <thread>

namespace gazebo
{
    class ConstantVelocityPlugin : public ModelPlugin
    {
    public:
        ConstantVelocityPlugin() : ModelPlugin()
        {
            // Constructor: ROS2 node initialisation will be handled in Load()
        }

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
        {
            this->model = _model;
            this->world = _model->GetWorld();

            // Read the topic name from the URDF parameters
            if (_sdf->HasElement("topicName"))
            {
                this->topicName = _sdf->Get<std::string>("topicName");
            }
            else
            {
                this->topicName = "/cmd_vel"; // default topic name
            }

            // Initialise ROS2 node
            if (!rclcpp::ok())
            {
                rclcpp::init(0, nullptr);
            }
            this->rosNode = rclcpp::Node::make_shared("constant_velocity_plugin");
            this->velocitySubscriber = this->rosNode->create_subscription<geometry_msgs::msg::Twist>(
                this->topicName, 10, std::bind(&ConstantVelocityPlugin::OnVelocityMsg, this, std::placeholders::_1));

            // Start a separate thread for the ROS2 spinner
            this->rosThread = std::thread([this]() { rclcpp::spin(this->rosNode); });

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ConstantVelocityPlugin::OnUpdate, this));

            std::cout << "ConstantVelocityPlugin loaded with topic: " << this->topicName << std::endl;
        }

        ~ConstantVelocityPlugin()
        {
            if (this->rosNode)
            {
                rclcpp::shutdown();
                if (this->rosThread.joinable())
                {
                    this->rosThread.join();
                }
            }
        }

    private:
        void OnVelocityMsg(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->linearVelocity = ignition::math::Vector3d(msg->linear.x, msg->linear.y, msg->linear.z);
            this->angularVelocity = ignition::math::Vector3d(msg->angular.x, msg->angular.y, msg->angular.z);
        }

        void OnUpdate()
        {
            // Apply linear velocity
            this->model->SetLinearVel(this->linearVelocity);

            // Apply angular velocity
            this->model->SetAngularVel(this->angularVelocity);
        }

        physics::ModelPtr model;
        physics::WorldPtr world;
        event::ConnectionPtr updateConnection;

        std::shared_ptr<rclcpp::Node> rosNode;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocitySubscriber;
        std::string topicName;

        ignition::math::Vector3d linearVelocity;
        ignition::math::Vector3d angularVelocity;

        std::thread rosThread;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ConstantVelocityPlugin)
}

