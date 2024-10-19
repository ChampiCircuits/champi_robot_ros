#include <iostream>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp/loggers/groot2_publisher.h"


using namespace std::chrono_literals;

class ApproachObject : public BT::SyncActionNode
{
public:
    explicit ApproachObject(const std::string &name) : BT::SyncActionNode(name, {})
    {}

    BT::NodeStatus tick() override
    {
        std::cout << "Approach Object: " << this->name() << std::endl;

        std::this_thread::sleep_for(5s);
        return BT::NodeStatus::SUCCESS;
    }
};

BT::NodeStatus CheckBattery()
{
    std::cout << "Battery check" << std::endl;
    std::this_thread::sleep_for(1s);
    return BT::NodeStatus::SUCCESS;
}

class GripperInterface
{
public:
    GripperInterface() : _open(true)
    {}

    BT::NodeStatus open()
    {
        std::cout << "Gripper Open" << std::endl;
        std::this_thread::sleep_for(1s);
        _open = true;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus close()
    {
        std::cout << "Gripper Close" << std::endl;
        std::this_thread::sleep_for(1s);
        _open = false;
        return BT::NodeStatus::SUCCESS;
    }
private:
    bool _open;
};

int main()
{

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<ApproachObject>("ApproachObject");
    
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    GripperInterface gripper;
    
    factory.registerSimpleAction(
        "OpenGripper",
        std::bind(&GripperInterface::open, &gripper)
        );

    factory.registerSimpleAction(
        "CloseGripper",
        std::bind(&GripperInterface::close, &gripper)
        );


    // Get the path of the package to access the installed (via cmakelists.txt) "xml" folder
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("champi_brain");

    auto tree = factory.createTreeFromFile(package_share_directory + "/xml/bt_tree.xml");
    
    // Publish the tree for real-time visualization on Groot2.
    BT::Groot2Publisher publisher(tree);

    tree.tickOnce();
    std::this_thread::sleep_for(10s);
    
    return 0;
}