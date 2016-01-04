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

using namespace RTT;
using namespace KDL;
using namespace std;


namespace MotionControl
{

CartesianGeneratorPos::CartesianGeneratorPos(string name) :
	TaskContext(name, PreOperational), m_motion_profile(6,
			VelocityProfile_Trap(0, 0)),
	move_started_event("e_"+name+"_move_started"),
	move_finished_event("e_"+name+"_move_finished"),
	pose_reset_event("e_"+name+"_pose_reset"),
	m_is_moving(false),
	m_time_passed(0)
{
	//Creating TaskContext

	//Adding Ports
	this->addPort("CartesianPoseMsr", m_position_meas_port);
	this->addPort("CartesianPoseDes", m_position_desi_port);
	this->addPort("CartesianTwistDes", m_velocity_desi_port);
	this->addPort("events", event_port);



	//Adding Properties
	this->addProperty("max_vel", m_maximum_velocity).doc(
			"Maximum Velocity in Trajectory");
	this->addProperty("max_acc", m_maximum_acceleration).doc(
			"Maximum Acceleration in Trajectory");

	//Adding Operations
	this->addOperation("moveTo", &CartesianGeneratorPos::moveTo, this,
			OwnThread) .doc("Set the position setpoint") .arg("setpoint",
			"position setpoint for end effector") .arg("time",
			"minimum time to execute trajectory");
	this->addOperation("resetPosition", &CartesianGeneratorPos::resetPosition,
			this, OwnThread).doc("Resets generator's position");
	this->addOperation("setPose", &CartesianGeneratorPos::setPose,
			this, OwnThread).doc("Sets (step) an arbitrary pose");

}

CartesianGeneratorPos::~CartesianGeneratorPos() { }

bool CartesianGeneratorPos::configureHook()
{
	for (unsigned int i = 0; i < 3; i++) {
		m_motion_profile[i].SetMax(m_maximum_velocity.vel[i],
				m_maximum_acceleration.vel[i]);
		m_motion_profile[i + 3].SetMax(m_maximum_velocity.rot[i],
				m_maximum_acceleration.rot[i]);
	}
	return true;
}

bool CartesianGeneratorPos::startHook()
{
	m_is_moving = false;

	Frame starting_pose;
	Twist starting_twist= Twist::Zero();

	if(m_position_meas_port.read(starting_pose)==NoData){
		log(Error) << this->getName() << " cannot start if " <<
			m_position_meas_port.getName()<<" has no input data."<<endlog();
		return false;
	}

	m_position_desi_port.write(starting_pose);
	m_velocity_desi_port.write(starting_twist);
	return true;
}

void CartesianGeneratorPos::updateHook()
{
	Frame pos_dsr;
	Twist vel_dsr;

	if (m_is_moving) {
		m_time_passed = os::TimeService::Instance()->secondsSince(m_time_begin);
		if (m_time_passed > m_max_duration) {
			// set end position
			m_position_desi_local = m_traject_end;
			SetToZero(m_velocity_desi_local);
			m_is_moving = false;
			// send move_finished_event (once)
            event_port.write(move_finished_event);
		} else {
			// position
			m_velocity_delta = Twist( Vector( m_motion_profile[0].Pos(m_time_passed),
							  m_motion_profile[1].Pos(m_time_passed),
							  m_motion_profile[2].Pos(m_time_passed)),
						  Vector( m_motion_profile[3].Pos(m_time_passed),
							  m_motion_profile[4].Pos(m_time_passed),
							  m_motion_profile[5].Pos(m_time_passed)));

			m_position_desi_local = Frame( m_traject_begin.M *
						       Rot( m_traject_begin.M.Inverse(m_velocity_delta.rot)),
						       m_traject_begin.p + m_velocity_delta.vel);

			// velocity
			for (unsigned int i = 0; i < 6; i++)
				m_velocity_desi_local(i) = m_motion_profile[i].Vel(m_time_passed);

		}

		// convert to geometry msgs and send.
		m_position_desi_local=pos_dsr;
		m_velocity_desi_local=vel_dsr;

		m_position_desi_port.write(pos_dsr);
		m_velocity_desi_port.write(vel_dsr);
	}
}

void CartesianGeneratorPos::stopHook() { }

void CartesianGeneratorPos::cleanupHook() { }

bool CartesianGeneratorPos::moveTo(Frame pose, double time)
{
	KDL::Frame pose_msr;

	if(!this->isRunning()){
		log(Error)<<this->getName()<<" is not running yet."<<endlog();
		return false;
	}

	m_max_duration = 0;
	m_traject_end=pose;

	// get current position
	m_position_meas_port.read(pose_msr);
	pose_msr= m_traject_begin;

	m_velocity_begin_end = diff(m_traject_begin, m_traject_end);

	// Set motion profiles
	for (unsigned int i = 0; i < 6; i++) {
		m_motion_profile[i].SetProfileDuration(0, m_velocity_begin_end(i), time);
		m_max_duration = max(m_max_duration, m_motion_profile[i].Duration());
	}

	// Rescale trajectories to maximal duration
	for (unsigned int i = 0; i < 6; i++)
		m_motion_profile[i].SetProfileDuration(0, m_velocity_begin_end(i),
				m_max_duration);

	m_time_begin = os::TimeService::Instance()->getTicks();
	m_time_passed = 0;

	m_is_moving = true;
	// send move_started_event )
	event_port.write(move_started_event);

	return true;
}

bool CartesianGeneratorPos::setPose(KDL::Frame t_pose)
{

	if(!this->isRunning()){
		log(Error)<<this->getName()<<" is not running yet."<<endlog();
		return false;
	}

	m_is_moving = false;
	m_position_desi_port.write(t_pose);
	m_velocity_desi_port.write(KDL::Twist::Zero());

	return true;
}

void CartesianGeneratorPos::resetPosition()
{
	Frame pose;

	m_position_meas_port.read(pose);
	m_position_desi_port.write(pose);

	SetToZero(m_velocity_desi_local);
	m_velocity_desi_port.write(Twist::Zero());

	m_is_moving = false;
	event_port.write(pose_reset_event);
}

} //namespace
