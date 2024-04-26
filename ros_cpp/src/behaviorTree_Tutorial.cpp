#include <iostream>
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

// Define action execution functions
void executeFollowPath() {
    std::cout << "Executing FollowPath action..." << std::endl;
    // Add logic to control the simulated robot to follow a path in Gazebo
}

// Parse and interpret the XML file
void parseXMLFile(const char* filename) {
    rapidxml::file<> xmlFile(filename); // Load XML file
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data()); // Parse XML data

    // Traverse the XML tree and interpret each action
    for (auto node = doc.first_node(); node; node = node->next_sibling()) {
        if (std::string(node->name()) == "ActionNode") {
            std::string actionName = node->first_attribute("name")->value();
            if (actionName == "FollowPath") {
                executeFollowPath();
            } else {
                std::cerr << "Unknown action: " << actionName << std::endl;
            }
        }
    }
}

// Gazebo plugin initialization function
void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Load XML file and interpret behavior tree structure
    std::string filePath = "../behavioralTree_tutorial.xml";
    parseXMLFile(filePath.c_str());
}

// Register Gazebo plugin
GZ_REGISTER_MODEL_PLUGIN(YourGazeboPlugin)

