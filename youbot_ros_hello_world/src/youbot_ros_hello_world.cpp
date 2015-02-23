//
// Simple demo program that calls the youBot ROS wrapper
//

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

using namespace std;

ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) {
	int numberOfJoints = 5;
	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);		
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);		

	return msg;
}


// Движение с заданной скоростью в течение заданного времени
void movePlatform() {
	geometry_msgs::Twist twist;

	int v_vpered, v_nalevo, t_vn;
	
	// Движение
	cout << "С какой скоростью двигаться вперед? \n"; 
	cin >> v_vpered;	
	cout << "С какой скоростью двигаться налево? \n"; 
	cin >> v_nalevo;	
	cout << "Сколько секунд двигаться всего? \n"; 
	cin >> t_vn;
	twist.linear.x = v_vpered;	// Движение вперед со скорость "v_vpered" м/с
	twist.linear.y = v_nalevo;	// Движение налево со скорость "v_nalevo" м/с
	platformPublisher.publish(twist);
	ros::Duration(t_vn).sleep();

	// Остановка
	twist.linear.x = 0;
	twist.linear.y = 0;
	platformPublisher.publish(twist);
}

// Управление манипулятором
void moveArm() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
	int coord_0, coord_1, coord_2, coord_3, coord_4;

	// Установить манипулятор в вертикальное положение на 5 секунд
	jointvalues[0] = 2.95;
	jointvalues[1] = 1.05;
	jointvalues[2] = -2.44;
	jointvalues[3] = 1.73;
	jointvalues[4] = 2.95;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);
	ros::Duration(5).sleep();
	
	// Установить манипулятор в указанное положение на 5 секунд
	cout << "Положение звена #1: от 0 до 5.8992; Вертикально: 2.9496 \n"; 
	cin >> coord_0;	
	cout << "Положение звена #2: от 0 до 2.7053; Вертикально: 1.1345 \n"; 
	cin >> coord_1;	
	cout << "Положение звена #3: от 0 до -5.1836; Вертикально: -2.5482 \n"; 
	cin >> coord_2;	
	cout << "Положение звена #4: от 0 до 3.5779; Вертикально: 1.7890 \n"; 
	cin >> coord_3;	
	cout << "Положение звена #5: от 0 до 5.8469; Вертикально: 2.9234 \n"; 
	cin >> coord_4;	
	jointvalues[0] = coord_0;
	jointvalues[1] = coord_1;
	jointvalues[2] = coord_2;
	jointvalues[3] = coord_3;
	jointvalues[4] = coord_4;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);
	ros::Duration(5).sleep();
	
	// Вернуть манипулятор в исходное положение на 2 секунды
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.11;
	jointvalues[3] = 0.11;
	jointvalues[4] = 0.111;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);
	ros::Duration(2).sleep();
}

// open and close gripper
void moveGripper() {
	brics_actuator::JointPositions msg;
	
	// open gripper
	msg = createGripperPositionCommand(0.011);
	gripperPublisher.publish(msg);

	ros::Duration(3).sleep();

	// close gripper
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_ros_hello_world");
	ros::NodeHandle n;

	platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
	sleep(1);

	movePlatform();
	moveArm();
	moveGripper();

	sleep(1);
	ros::shutdown();

	return 0;
}
