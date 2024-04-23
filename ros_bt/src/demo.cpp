/*
This is a demo for BehaviorTree.CPP BehaviorTree.ROS2 
first you must ros2 run the fibonacci and add_two_ints servers 
the run demo and it will go through the behaviour tree
*/
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "ros_interfaces/action/fibonacci.hpp"
#include "ros_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


// define some namespace shortcuts
using Fibonacci = ros_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
using AddTwoInts = ros_interfaces::srv::AddTwoInts;

namespace chr = std::chrono;
using namespace BT;
// dummy data type for pretend MoceBase action
struct Pose2D
{
    double x, y, theta;
};

namespace BT {
    template <> inline Pose2D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Pose2D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            output.theta     = convertFromString<double>(parts[2]);
            return output;
        }
    }
}

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// SyncActionNode (synchronous action) with an input port.
/*
   a SyncActionNode can only return SUCCESS or FAILURE
   it cannot return RUNNING. so it is only good for simple, fast operations
 */
// SyncActionNode (synchronous action) with an input port.
class SaySomething : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        SaySomething(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config)
        { }

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts()
        {
            // This action has a single input port called "message"
            return { InputPort<std::string>("message") };
        }

        // Override the virtual function tick()
        NodeStatus tick() override
        {
            Expected<std::string> msg = getInput<std::string>("message");
            // Check if expected is valid. If not, throw its error
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [message]: ", 
                        msg.error() );
            }
            // use the method value() to extract the valid message.
            std::cout << "Robot says: " << msg.value() << std::endl;
            return NodeStatus::SUCCESS;
        }
};

/*
   a StatefulActionNode can return RUNNING but it cannot block
   so it must rely on some other thread/process/node to do work 
   and it just reports the status to the tree
   This example just demonstrates a simple RUNNING state that 
   does nothing
 */
class MoveBase : public BT::StatefulActionNode
{
    public:
        // Any TreeNode with ports must have a constructor with this signature
        MoveBase(const std::string& name, const BT::NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {}

        // It is mandatory to define this static method.
        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<Pose2D>("goal") };
        }

        // this function is invoked once at the beginning.
        BT::NodeStatus onStart() override;

        // If onStart() returned RUNNING, we will keep calling
        // this method until it return something different from RUNNING
        BT::NodeStatus onRunning() override;

        // callback to execute if the action was aborted by another node
        void onHalted() override;

    private:
        Pose2D _goal;
        chr::system_clock::time_point _completion_time;
};

//-------------------------

BT::NodeStatus MoveBase::onStart()
{
    if ( !getInput<Pose2D>("goal", _goal))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }
    printf("[ MoveBase: SEND REQUEST ]. goal: x=%f y=%f theta=%f\n",
            _goal.x, _goal.y, _goal.theta);

    // We use this counter to simulate an action that takes a certain
    // amount of time to be completed (200 ms)
    _completion_time = chr::system_clock::now() + chr::milliseconds(220);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBase::onRunning()
{
    // Pretend that we are checking if the reply has been received
    // you don't want to block inside this function too much time.
    std::this_thread::sleep_for(chr::milliseconds(10));

    // Pretend that, after a certain amount of time,
    // we have completed the operation
    if(chr::system_clock::now() >= _completion_time)
    {
        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}
void MoveBase::onHalted()
{
    printf("[ MoveBase: ABORTED ]");
}
/* 
   this one calls a ROS action (the Fibonnaci action tutorial)
   It should call the action and return RUNNING until it finishes.
 */
class FibonacciAction: public RosActionNode<Fibonacci>
{
    public:
        FibonacciAction(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosActionNode<Fibonacci>(name, conf, params)
        {}

        // The specific ports of this Derived class
        // should be merged with the ports of the base class,
        // using RosActionNode::providedBasicPorts()
        static PortsList providedPorts()
        {
            return providedBasicPorts({InputPort<unsigned>("order")});
        }

        // This is called when the TreeNode is ticked and it should
        // send the request to the action server
        bool setGoal(RosActionNode::Goal& goal) override 
        {
            // get "order" from the Input port
            getInput("order", goal.order);
            // return true, if we were able to set the goal correctly.
            return true;
        }

        // Callback executed when the reply is received.
        // Based on the reply you may decide to return SUCCESS or FAILURE.
        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            std::stringstream ss;
            ss << "Result received: ";
            for (auto number : wr.result->sequence) {
                ss << number << " ";
            }
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return NodeStatus::SUCCESS;
        }

        // Callback invoked when there was an error at the level
        // of the communication between client and server.
        // This will set the status of the TreeNode to either SUCCESS or FAILURE,
        // based on the return value.
        // If not overridden, it will return FAILURE by default.
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
            return NodeStatus::FAILURE;
        }

        // we also support a callback for the feedback, as in
        // the original tutorial.
        // Usually, this callback should return RUNNING, but you
        // might decide, based on the value of the feedback, to abort
        // the action, and consider the TreeNode completed.
        // In that case, return SUCCESS or FAILURE.
        // The Cancel request will be send automatically to the server.
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Next number in sequence received: ";
            for (auto number : feedback->partial_sequence) {
                ss << number << " ";
            }
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return NodeStatus::RUNNING;
        }
};

class AddTwoIntsNode: public RosServiceNode<AddTwoInts>
{
    public:

        AddTwoIntsNode(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosServiceNode<AddTwoInts>(name, conf, params)
        {}

        // The specific ports of this Derived class
        // should be merged with the ports of the base class,
        // using RosServiceNode::providedBasicPorts()
        static PortsList providedPorts()
        {
            return providedBasicPorts({
                    InputPort<unsigned>("A"),
                    InputPort<unsigned>("B")});
        }

        // This is called when the TreeNode is ticked and it should
        // send the request to the service provider
        bool setRequest(Request::SharedPtr& request) override
        {
            // use input ports to set A and B
            getInput("A", request->a);
            getInput("B", request->b);
            // must return true if we are ready to send the request
            return true;
        }

        // Callback invoked when the answer is received.
        // It must return SUCCESS or FAILURE
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override
        {
            RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
            return NodeStatus::SUCCESS;
        }

        // Callback invoked when there was an error at the level
        // of the communication between client and server.
        // This will set the status of the TreeNode to either SUCCESS or FAILURE,
        // based on the return value.
        // If not overridden, it will return FAILURE by default.
        virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
            return NodeStatus::FAILURE;
        }
};
static const char* xml_text = R"(
<root BTCPP_format="4" >
     <BehaviorTree ID="MainTree">
        <Sequence>
            <BatteryOK/>
            <SaySomething   message="mission started..." />
            <MoveBase           goal="1;2;3"/>
            <SaySomething   message="mission completed!" />
            <Fibonacci order="3"/>
            <AddTwoInts A="3" B="4"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    // these are the simple BT nodes
    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<MoveBase>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

    // this is the ROS2 action client node so it needs some extra parameters
    auto node = std::make_shared<rclcpp::Node>("fibonacci_action_client");
    // provide the ROS node and the name of the action service
    RosNodeParams params; 
    params.nh = node;
    params.default_port_value = "fibonacci";
    factory.registerNodeType<FibonacciAction>("Fibonacci", params);

    auto service_node = std::make_shared<rclcpp::Node>("add_two_ints_client");
    // provide the ROS node and the name of the  service
    RosNodeParams service_params; 
    service_params.nh = service_node;
    service_params.default_port_value = "add_two_ints";
    factory.registerNodeType<AddTwoIntsNode>("AddTwoInts", service_params);

    auto tree = factory.createTreeFromText(xml_text);

    // Here, instead of tree.tickWhileRunning(),
    // we prefer our own loop.
    std::cout << "--- ticking\n";
    NodeStatus status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";

    while(status == NodeStatus::RUNNING) 
    {
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.
        tree.sleep(std::chrono::milliseconds(100));

        std::cout << "--- ticking\n";
        status = tree.tickOnce();
        std::cout << "--- status: " << toStr(status) << "\n\n";
    }
    return 0;
}


