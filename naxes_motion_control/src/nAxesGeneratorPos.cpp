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

#include "nAxesGeneratorPos.hpp"
#include <rtt/Component.hpp>

namespace motion_control
{
    using namespace RTT;
    using namespace KDL;
    using namespace std;
    typedef nAxesGeneratorPos MyType;

    nAxesGeneratorPos::nAxesGeneratorPos(const string& name)
      : TaskContext(name,PreOperational),
	finished_event("finished")
    {
        //Creating TaskContext

        //Adding properties
        this->addProperty("num_axes",num_axes_prop).doc("Number of Axes");
        this->addProperty("max_vel", v_max_prop).doc("Maximum Velocity in Trajectory");
        this->addProperty("max_acc", a_max_prop).doc("Maximum Acceleration in Trajectory");

        //Adding ports
        this->addPort("nAxesSensorPosition"  , p_m_port );
        this->addPort("nAxesDesiredPosition" , p_d_port );
        this->addPort("nAxesDesiredVelocity" , v_d_port );
	this->addPort("moveFinished", move_finished_port);

        //Adding Operations
        this->addOperation( "moveTo",&MyType::moveTo,this )
			  .doc("Set the position setpoint")
			  .arg("setpoint", "joint setpoint for all axes")
			  .arg("time", "minimum time to complete trajectory");
        this->addOperation( "resetPosition", &MyType::resetPosition, this )
			  .doc("Reset generator position");

    }

    nAxesGeneratorPos::~nAxesGeneratorPos()
    {}

    bool nAxesGeneratorPos::configureHook()
    {
      Logger::In in(this->getName());
      num_axes=num_axes_prop;
      if(v_max_prop.size()!=num_axes){
	log(Error)<<"Size of max_vel does not match num_axes"<<endlog();
	return false;
      }

      if(a_max_prop.size()!=num_axes){
	log(Error)<<"Size of max_acc does not match num_axes" <<endlog();
	return false;
      }

      v_max=v_max_prop;
      a_max=a_max_prop;

      //Resizing all containers to correct size
      p_m.resize(num_axes);
      p_d.resize(num_axes);
      v_d.resize(num_axes);
      motion_profile.resize(num_axes);

      //Initialise motion profiles
      for(unsigned int i=0;i<num_axes;i++)
	motion_profile[i].SetMax(v_max[i],a_max[i]);

      //Initialise output ports:
      p_d.assign(num_axes,0);
      p_d_port.setDataSample( p_d );
      v_d.assign(num_axes,0);
      v_d_port.setDataSample( v_d );
      move_finished_port.setDataSample(finished_event);
      return true;
    }


    bool nAxesGeneratorPos::startHook()
    {
      Logger::In in(this->getName());
      //check connection and sizes of input-ports
      if(!p_m_port.connected()){
	log(Error)<<p_m_port.getName()<<" not ready"<<endlog();
	return false;
      }
      if(p_m_port.read(p_m)==NoData){
	log(Error)<<"No data available on "<<p_m_port.getName()<<", I refuse to start"<<endlog();
	return false;
      }
      if(p_m.size()!=num_axes){
	log(Error)<<"Size of "<<p_m_port.getName()<<": "<<p_m.size()<<" != " << num_axes<<endlog();
	return false;
      }
      
      p_d_port.write( p_m );
      is_moving = false;

      return true;
    }

    void nAxesGeneratorPos::updateHook()
    {
        if (is_moving){
            time_passed = os::TimeService::Instance()->secondsSince(time_begin);
            if ( time_passed > max_duration ){// Profile is ended
                // set end position
                for (unsigned int i=0; i<num_axes; i++){
                    p_d[i] = motion_profile[i].Pos( max_duration );
                    v_d[i] = 0;//_motion_profile[i]->Vel( _max_duration );
                    is_moving = false;
		    move_finished_port.write(finished_event);
                }
            }else{
                for(unsigned int i=0; i<num_axes; i++){
                    p_d[i] = motion_profile[i].Pos( time_passed );
                    v_d[i] = motion_profile[i].Vel( time_passed );
                }
            }
            p_d_port.write( p_d );
            v_d_port.write( v_d );
        }
    }

    void nAxesGeneratorPos::stopHook()
    {
    }

    bool nAxesGeneratorPos::moveTo(const vector<double>& position, double time)
    {
        Logger::In in((this->getName()));
        if(position.size()!=num_axes){
            log(Error)<<"Size of position != "<<num_axes<<endlog();
            return false;
        }

        // if previous movement is finished
        if (!is_moving){
            max_duration = 0;
            // get current position/
            p_m_port.read( p_m );
            for (unsigned int i=0; i<num_axes; i++){
                // Set motion profiles
                motion_profile[i].SetProfileDuration( p_m[i], position[i], time );
                // Find lengthiest trajectory
                max_duration = max( max_duration, motion_profile[i].Duration() );
            }
            // Rescale trajectories to maximal duration
            for(unsigned int i = 0; i < num_axes; i++)
                motion_profile[i].SetProfileDuration( p_m[i], position[i], max_duration );

            time_begin = os::TimeService::Instance()->getTicks();
            time_passed = 0;

            is_moving = true;
            return true;
        }
        // still moving
        else{
            log(Warning)<<"Still moving, not executing new command."<<endlog();
            return false;
        }

    }


    void nAxesGeneratorPos::resetPosition()
    {
        p_m_port.read( p_d );
        for(unsigned int i = 0; i < num_axes; i++)
            v_d[i] = 0;
        p_d_port.write( p_d );
        v_d_port.write( v_d );
        is_moving = false;
    }
}//namespace

ORO_CREATE_COMPONENT( motion_control::nAxesGeneratorPos )



