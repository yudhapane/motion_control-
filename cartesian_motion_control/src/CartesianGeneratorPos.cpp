// $Id: nAxisGeneratorCartesianPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2006 Ruben Smits <ruben.smits@mech.kuleuven.ac.be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "CartesianGeneratorPos.hpp"
#include <ocl/Component.hpp>

ORO_CREATE_COMPONENT( MotionControl::CartesianGeneratorPos)
;

namespace MotionControl {

using namespace RTT;
using namespace KDL;
using namespace std;

CartesianGeneratorPos::CartesianGeneratorPos(string name) :
	TaskContext(name, PreOperational), m_motion_profile(3,
			VelocityProfile_Trap(0, 0)), m_is_moving(false) {
	//Creating TaskContext

	//Adding Ports
	this->addPort("CartesianPoseMsr", m_position_meas_port);
	this->addPort("CartesianPoseDes", m_position_desi_port);
	this->addPort("CartesianPoseDesRos", m_position_ros_port);
	this->addPort("MoveActive", m_move_Active_port);
	//his->addPort("ControllerState_in", ControllerState_in);
	//Adding Properties
	this->addProperty("max_vel", max_velocity).doc(
			"Maximum Velocity in Trajectory");
	this->addProperty("max_acc", max_acceleration).doc(
			"Maximum Acceleration in Trajectory");

	//Adding Commands
	this->addOperation("moveTo", &CartesianGeneratorPos::moveTo, this,
			OwnThread) .doc("Set the position setpoint") .arg("setpoint",
			"position setpoint for end effector") .arg("time",
			"minimum time to execute trajectory");

	//Adding Methods
	this->addOperation("resetPosition", &CartesianGeneratorPos::resetPosition,
			this, OwnThread).doc("Reset generator's position");

}

CartesianGeneratorPos::~CartesianGeneratorPos() {
}

bool CartesianGeneratorPos::configureHook() {
/*	m_maximum_acceleration = Twist(Vector(max_acceleration,max_acceleration,max_acceleration),Vector			
	(max_acceleration,max_acceleration,max_acceleration));

	m_maximum_velocity = Twist(Vector(max_velocity,max_velocity,max_velocity),Vector(max_velocity,max_velocity,max_velocity));

	for (unsigned int i = 0; i < 3; i++) {
		m_motion_profile[i].SetMax(5,5); //(m_maximum_velocity.vel[i], m_maximum_acceleration.vel[i]);
		//m_motion_profile[i + 3].SetMax(m_maximum_velocity.rot[i],
		//		m_maximum_acceleration.rot[i]);
	}
*/

	for (unsigned int i = 0; i < 3; i++) {
		m_motion_profile[i].SetMax(max_velocity,max_acceleration);
		}


	ds_to_ros = 10;
	return true;


}

bool CartesianGeneratorPos::startHook() {
	int ControlState;

	m_is_moving = false;
	//initialize
	KDL::Vector starting_pose;
	if(m_position_meas_port.read(starting_pose)==NoData){
		log(Error)<<this->getName()<<" cannot start if "<< m_position_meas_port.getName()<<" has no input data."<<endlog();
		return false;
	}

	Pose_out.x = starting_pose(0);
	Pose_out.y = starting_pose(1);
	Pose_out.z = starting_pose(2);


	for (unsigned int i = 0; i < 3; i++) {
		m_motion_profile[i].SetMax(max_velocity, max_acceleration); //(m_maximum_velocity.vel[i], m_maximum_acceleration.vel[i]);
		//m_motion_profile[i + 3].SetMax(m_maximum_velocity.rot[i],
		//		m_maximum_acceleration.rot[i]);
	}

	m_position_desi_port.write(Pose_out);
	//Twist starting_twist = Twist::Zero();
	m_position_ros_port.write(Pose_out);
	return true;
}

void CartesianGeneratorPos::updateHook() {
	//int ControlState;
	//ControlState = 1 ;


	if (m_is_moving) {
	//ControllerState_in.read(ControlState);		
		
		//if (ControlState == 1){

			m_time_passed = os::TimeService::Instance()->secondsSince(m_time_begin);
		
			//only 3D movement
		
			if (m_time_passed > m_max_duration) {
				// set end position
				m_position_desi_local = m_traject_end;
				SetToZero(m_velocity_desi_local);
				//m_move_finished_port.write(true);
				m_is_moving = false;
				resetPosition();
			} else {
				// position
				m_position_desi_local.p(0) = m_motion_profile[0].Pos(m_time_passed)+ m_traject_begin.p(0);
				m_position_desi_local.p(1) = m_motion_profile[1].Pos(m_time_passed)+ m_traject_begin.p(1);
				m_position_desi_local.p(2) = m_motion_profile[2].Pos(m_time_passed)+ m_traject_begin.p(2);

			}

			Pose_out.x= m_position_desi_local.p(0);
			Pose_out.y=m_position_desi_local.p(1);
			Pose_out.z=m_position_desi_local.p(2);

			m_position_desi_port.write(Pose_out);

			// Writing to Ros (i.e. at a lower rate)
			if (ros_counter >= ds_to_ros){
			m_position_ros_port.write(Pose_out);
			ros_counter = 0;
			}
			else
			ros_counter++;

		//}
	}

	m_move_Active_port.write(m_is_moving);
}

void CartesianGeneratorPos::stopHook() {
}

void CartesianGeneratorPos::cleanupHook() {
}

bool CartesianGeneratorPos::moveTo(double xPos, double yPos, double zPos, double time) {
	
	Frame pose_in(Vector(xPos,yPos,zPos)); // creates an identity matrix
	

	if(!this->isRunning()){
		log(Error)<<this->getName()<<" is not running yet."<<endlog();
		return false;
	}
	m_max_duration = 0;

	m_traject_end=pose_in;

	// get current position
	

	m_position_meas_port.read(Pose);

	m_traject_begin = Frame(Pose);


	m_velocity_begin_end = diff(m_traject_begin, m_traject_end);

	// Set motion profiles
	for (unsigned int i = 0; i < 3; i++) {
		m_motion_profile[i].SetProfileDuration(0, m_velocity_begin_end(i), time);
		m_max_duration = max(m_max_duration, m_motion_profile[i].Duration());
	}

	// Rescale trajectories to maximal duration, SetProfileDuration-function is intended 
	//for trajectories below max speed and may cause the time to exceed the input for very fast movements

	for (unsigned int i = 0; i < 3; i++)
		m_motion_profile[i].SetProfileDuration(0, m_velocity_begin_end(i),m_max_duration);

	m_time_begin = os::TimeService::Instance()->getTicks();
	m_time_passed = 0;

	m_is_moving = true;
	return true;
}

void CartesianGeneratorPos::resetPosition() {
	//Frame pose;
	//m_position_meas_port.read(pose);
	m_position_meas_port.read(Pose);
//
	//pose = Frame(Pose);
	SetToZero(m_velocity_desi_local);

	Pose_out.x=Pose(0);
	Pose_out.y=Pose(1);
	Pose_out.z=Pose(2);

	m_position_desi_port.write(Pose_out);
	m_position_ros_port.write(Pose_out);
	m_is_moving = false;
}
}//namespace


