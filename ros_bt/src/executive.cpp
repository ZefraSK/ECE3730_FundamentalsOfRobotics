#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ros_interfaces/srv/get_pose.hpp"


namespace chr = std::chrono;
using namespace BT;
using navToPos = nav2_msgs::action::NavigateToPose;
using getMap = nav_msgs::srv::GetMap;
using PoseMsg = geometry_msgs::msg::Pose;
using O_Grid = nav_msgs::msg::OccupancyGrid;
using Spin = nav2_msgs::action::Spin;
using GetPose = ros_interfaces::srv::GetPose;

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return NodeStatus::SUCCESS;
}

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
                throw RuntimeError("missing required input [message]: ", 
                        msg.error() );
            }
            // use the method value() to extract the valid message.
            std::cout << "Robot says: " << msg.value() << std::endl;
            return NodeStatus::SUCCESS;
        }
};

 /* Hardcoded to wait 30 seconds upon bootup */
 class WaitForBoot : public BT::StatefulActionNode
{
    public:
        // Any TreeNode with ports must have a constructor with this signature
        WaitForBoot(const std::string& name, const BT::NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {}

        // It is mandatory to define this static method.
        static PortsList providedPorts()
        {
             return {InputPort<std::string>("seconds")};
        }

        // this function is invoked once at the beginning.
        BT::NodeStatus onStart() override;

        // If onStart() returned RUNNING, we will keep calling
        // this method until it return something different from RUNNING
        BT::NodeStatus onRunning() override;

        // callback to execute if the action was aborted by another node
        void onHalted() override;

    private:
        chr::system_clock::time_point _completion_time;
};


/* Waits given amount of seconds */
 class WaitSeconds: public BT::StatefulActionNode
{
    public:
        // Any TreeNode with ports must have a constructor with this signature
        WaitSeconds(const std::string& name, const BT::NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {}

        // It is mandatory to define this static method.
        static PortsList providedPorts()
        {
             return {InputPort<std::string>("seconds")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
    	unsigned _seconds;
        chr::system_clock::time_point _completion_time;
};
/* Function that waits */
BT::NodeStatus WaitSeconds::onStart()
{
    if (!getInput<unsigned>("seconds", _seconds))
    {
    	throw RuntimeError("missing required input [seconds]");
    }
    printf("[ WaitSeconds: ] seconds = %d\n", _seconds);

    // We use this counter to simulate an action that takes a certain
    // amount of time to be completed (200 ms)
    _completion_time = chr::system_clock::now() + chr::milliseconds(_seconds * 1000);

    return NodeStatus::RUNNING;
}

BT::NodeStatus WaitSeconds::onRunning()
{
    std::this_thread::sleep_for(chr::milliseconds(1000));
    if(chr::system_clock::now() >= _completion_time)
    {
        std::cout << "[ WaitSeconds: FINISHED ]" << std::endl;
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}
void WaitSeconds::onHalted()
{
    printf("[ WaitSeconds: ABORTED ]");
}



/* Function takes in a target_yaw value and will spin based on that */
class SpinAction: public RosActionNode<Spin>
{
    public:
        SpinAction(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosActionNode<Spin>(name, conf, params)
        {}

        static PortsList providedPorts()
        {
            return providedBasicPorts({InputPort<unsigned>("target_yaw")});
        }

        bool setGoal(RosActionNode::Goal& goal) override 
        {
            getInput("target_yaw", goal.target_yaw);
            return true;
        }

        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            std::stringstream ss;
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
            return NodeStatus::FAILURE;
        }
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Angular Distance Traveled: "<< feedback->angular_distance_traveled << " ";
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return NodeStatus::RUNNING;
        }
};



/* Upon receiving the map, will output the map as map */
class MapGrabber: public RosServiceNode<getMap>
{
    public:
        MapGrabber(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosServiceNode<getMap>(name, conf, params)
        {}

        static PortsList providedPorts()
        {
            return providedBasicPorts({OutputPort<O_Grid>("map")});
        }

        bool setRequest(Request::SharedPtr& request) override
        {
            return true;
        }

        NodeStatus onResponseReceived(const Response::SharedPtr& response) override
        {
            RCLCPP_INFO(node_->get_logger(), "Map Received!");	// Logs 
            
            O_Grid map;			// Creates occup_grid called map
            map = response->map;	// Give map variable the map detected
            setOutput("map", map);	// Returns the new map gotten
            
            return NodeStatus::SUCCESS;
        }

        virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
            return NodeStatus::FAILURE;
        }
};



/* Given pose x,y values it will go there */
class MovToPoseAction: public RosActionNode<navToPos>
{
    public:
        MovToPoseAction(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosActionNode<navToPos>(name, conf, params)
        {}

        static PortsList providedPorts()
        {
            return providedBasicPorts({InputPort<PoseMsg>("pose")});
        }

        bool setGoal(RosActionNode::Goal& goal) override 
        {
            getInput("pose", goal.pose.pose);
            goal.pose.header.frame_id="map";
            
            return true;
        }

        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            std::stringstream ss;
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
            return NodeStatus::SUCCESS;
            //return NodeStatus::FAILURE;	// Should really return false but we use a getto method instead
        }
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
            //std::stringstream ss;
            //ss <<"Estimated Time Remaining to Reach Pos: " << feedback->estimated_time_remaining << " ";
            //RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return NodeStatus::RUNNING;
        }
};

/* Finds the frontier, calculates new location to go to, output goal_pose*/
class FindFrontier : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        FindFrontier(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config)
        { }
        
        static PortsList providedPorts()
        {
            // Inputs & Outputs
            return { OutputPort<PoseMsg>("goal_pose"),
            	     InputPort<O_Grid>("map"),
            	     InputPort<PoseMsg>("current_pose")
            	 };
        }
        
        // Things to consider
        // data inside O_Grid = -1 indicates unknown area
        
        //Creates a function that returns the goal position
        PoseMsg _current_pose;
        PoseMsg _find_frontier_pose(O_Grid map, PoseMsg pose)
        {
	     unsigned pos_x = (pose.position.x - map.info.origin.position.x) / map.info.resolution;		// Current X-Position [Pixels] realitive to the origin
	     unsigned pos_y = (pose.position.y - map.info.origin.position.y) / map.info.resolution;		// Current Y-Position [Pixels] realitive to the origin 
	     bool tried_once = 0;
	     printf("[PERSONAL LOGS]: MAP INFO: %f", map.info.resolution);
	     
	     float distance; 
	     float distance_goal = 0;						// Distance [Pixels] to map pose from current pos
	     float distance_max = 75;							// Max distance [Pixels] the goal can be set
	     float distance_min = 30;
             unsigned margin = 30;							// Small area near walls
             unsigned max_x = map.info.width;
             unsigned max_y = map.info.height;
             
             unsigned goal_x = max_x - margin;
             unsigned goal_y = max_y - margin;
             
             unsigned temp_x;
             unsigned temp_y;
        	
        	// For loops that goes to each pixel point of the map
        	for(int x = 0; x < max_x; x++)
        	{
        	     for(int y = 0; y < max_y; y++)
        	     {
        	          int map_data = map.data[x+y*max_x];
        	          if((map_data == -1) && (x > margin) && (x < max_x-margin) && (y > margin) && (y < max_y-margin))
        	          {
        	               printf("[PERSONAL LOGS]: VALID -1 DETECTED at x: %d     y: %d\n",x,y);
        	               distance = sqrt(pow(((float)x - (float)pos_x),2) + pow(((float)y - (float)pos_y),2));	// Calculated distance to map
        	               printf("[PERSONAL LOGS]: Distance Calculated: %f\n", distance);
        	               temp_x = x;
        	               temp_y = y;
        	               if((distance > distance_goal) && (distance < distance_max)) 	// (distance > distance_goal) && 
        	               {
        	                    distance_goal = distance;
        	                    goal_x = x;
        	                    goal_y = y;
        	                    printf("[PERSONAL LOGS]: NEW DISTANCE: %f\n", distance);
        	                    printf("[PERSONAL LOGS]: NEW X: %d     pos_X: %d\n", x,pos_x);
        	                    printf("[PERSONAL LOGS]: NEW Y: %d     pos_Y: %d\n", y,pos_y);
        	                    tried_once = 1;
        	               }
        	               //else if (tried_once == 0)
        	               //{
        	               //goal_x = x;
        	               //goal_y = y;
        	               //}
        	          }
        	     }
        	}
        	
        	if((goal_x == max_x-margin) || (goal_y == max_y-margin) || (distance > 5000))
        	{
        	     printf("[PERSONAL LOGS]: GOAL X AND Y NEVER CHANGED\n");
        	     goal_x = temp_x;
        	     goal_y = temp_y;
        	}
        	
	        printf("[PERSONAL LOGS]: NEW DISTANCE: %f\n", distance);
	        printf("[PERSONAL LOGS]: Goal X: %d     pos_X: %d\n", goal_x,pos_x);
	        printf("[PERSONAL LOGS]: Goal Y: %d     pos_Y: %d\n", goal_y,pos_y);
        	PoseMsg goal;
        	goal.position.x = goal_x * map.info.resolution + map.info.origin.position.x;
        	goal.position.y = goal_y * map.info.resolution + map.info.origin.position.y;
        	
        	return goal;
         	
	}
	
	NodeStatus tick() override
        {
            Expected<O_Grid> msg = getInput<O_Grid>("map");
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [map]: ", 
                        msg.error() );
            }
            

	    getInput("current_pose", _current_pose);
	    O_Grid map = msg.value();
	    PoseMsg goal = _find_frontier_pose(map, _current_pose);
	    setOutput("goal_pose", goal);
	    return NodeStatus::SUCCESS;

        }	
};

        
/* Checks if map is mostly finished, returns true if so */
class MapFinished : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        MapFinished(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config)
        { }
        
        static PortsList providedPorts()
        {
            // Inputs & Outputs
            return { InputPort<O_Grid>("map")};
        }
        
        NodeStatus tick() override
        {
            Expected<O_Grid> msg = getInput<O_Grid>("map");
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [map]: ", 
                        msg.error() );
            }
            
            O_Grid map = msg.value();
            unsigned map_size = map.info.width * map.info.height;
            unsigned unknownCount = 0;
            
            for(int i = 0; i < map_size; i++)
            {
            	if(map.data[i] == -1)
            	{
            	     unknownCount++;
            	}
            }
            
            float percent_done = float(unknownCount)/float(map_size);
            if(percent_done > .95)
            {
            	std::cout << "MAP is at least 95% completed";
            	return NodeStatus::SUCCESS;
            }
            else
            {
            	return NodeStatus::FAILURE;
            }

        }
};


class PoseService: public RosServiceNode<GetPose>
{
    public:
        PoseService(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosServiceNode<GetPose>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({OutputPort<PoseMsg>("pose")});
        }
        bool setRequest(Request::SharedPtr& request) override
        {
            return true;
        }
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override
        {
            RCLCPP_INFO(node_->get_logger(), "Got a pose!!!!!");
            setOutput("pose",response->pose);
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
            RCLCPP_INFO(node_->get_logger(), "WHERE THE POSE AT?!?");
            return NodeStatus::FAILURE;
        }
};


/* Function that waits */
BT::NodeStatus WaitForBoot::onStart()
{
    std::cout << "[ WaitForBoot: STARTED ]" << std::endl;

    // We use this counter to simulate an action that takes a certain
    // amount of time to be completed (200 ms)
    _completion_time = chr::system_clock::now() + chr::milliseconds(30000);

    return NodeStatus::RUNNING;
}

BT::NodeStatus WaitForBoot::onRunning()
{
    // Pretend that we are checking if the reply has been received
    // you don't want to block inside this function too much time.
    std::this_thread::sleep_for(chr::milliseconds(10));

    // Pretend that, after a certain amount of time,
    // we have completed the operation
    if(chr::system_clock::now() >= _completion_time)
    {
        std::cout << "[ WaitForBoot: FINISHED ]" << std::endl;
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}
void WaitForBoot::onHalted()
{
    printf("[ WaitForBoot: ABORTED ]");
}


//             		<GetCurrentPose pose = "{robot_pose}"/>
static const char* xml_text = R"(
<root BTCPP_format="4" >
     <BehaviorTree ID="MainTree">
        <Sequence>
            <BatteryOK/>
            <WaitSeconds seconds = "30"/>
            <SaySomething   message="Spin Started..." />
            <SpinAction target_yaw="7"/>
            <RetryUntilSuccessful num_attempts="250">
            	<Sequence>
            		<MapGrabber map = "{map}" />
            		<PoseService pose = "{robot_pose}" />
            		<FindFrontier goal_pose = "{target_pose}"  current_pose = "{robot_pose}"  map = "{map}" />
            		<MovToPoseAction pose = "{target_pose}" />
            		<WaitSeconds seconds ="20"/>
            		<MapFinished map = "{map}"/>
            	</Sequence>
            </RetryUntilSuccessful>
            <SaySomething   message="MISSION COMPLETED!!!" />
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
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MapFinished>("MapFinished");
    factory.registerNodeType<FindFrontier>("FindFrontier");
    
    
    //WaitForBoot BT Setup
    factory.registerNodeType<WaitForBoot>("WaitForBoot");
    factory.registerNodeType<WaitSeconds>("WaitSeconds");


    //Spin BT Setup
    auto spin_node = std::make_shared<rclcpp::Node>("spin_client");
    RosNodeParams spin_params; 
    spin_params.nh = spin_node;
    spin_params.default_port_value = "spin";
    factory.registerNodeType<SpinAction>("SpinAction", spin_params);


    //MapGrabber BT Setup
    auto map_service_node = std::make_shared<rclcpp::Node>("MapGrabber_client");
    RosNodeParams map_service_params; 
    map_service_params.nh = map_service_node;
    map_service_params.default_port_value = "slam_toolbox/dynamic_map";
    factory.registerNodeType<MapGrabber>("MapGrabber", map_service_params);
    
    
    //GetCurrentPose BT Setup
    auto service_node = std::make_shared<rclcpp::Node>("GetCurrentPose_client");
    RosNodeParams service_params; 
    service_params.nh = service_node;
    service_params.default_port_value = "get_pose";
    factory.registerNodeType<PoseService>("PoseService", service_params);
    
    
    //NavToPose BT Setup
    auto navToPose_node = std::make_shared<rclcpp::Node>("NavToPose_client");
    RosNodeParams navToPose_params; 
    navToPose_params.nh = navToPose_node;
    navToPose_params.default_port_value = "navigate_to_pose";
    factory.registerNodeType<MovToPoseAction>("MovToPoseAction", navToPose_params);



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


