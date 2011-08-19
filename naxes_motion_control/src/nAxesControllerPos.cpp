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


#include "nAxesControllerPos.hpp"
#include <rtt/Component.hpp>

namespace motion_control
{
  
  using namespace RTT;
  using namespace std;
  typedef nAxesControllerPos MyType;

  
  nAxesControllerPos::nAxesControllerPos(string name)
    : TaskContext(name,PreOperational),
      finished_event("finished")
  {
    //Creating TaskContext
    
    //Adding Ports
    this->addPort("nAxesSensorPosition"  , p_meas_port);
    this->addPort("nAxesDesiredPosition" , p_desi_port);
    this->addPort("nAxesOutputVelocity"  , v_out_port);
    this->addPort("finishedMeasuringOffsets", finished_measuring_offsets_port);
    
    //Adding Properties
    this->addProperty("num_axes",num_axes_prop).doc("Number of Axes");
    this->addProperty("K",gain_prop).doc("Proportional Gain");;
    this->addAttribute("nAxesVelocityOffset",offset_attr);

    //Adding Commands
    this->addOperation( "measureVelocityOffset", &MyType::startMeasuringOffsets, this )
      .doc("calculate the velocity offset on the axes")
      .arg("time_sleep", "time to wait before starting measurement")
      .arg("num_samples", "number of samples to take");
    }
  
    nAxesControllerPos::~nAxesControllerPos(){};

    bool nAxesControllerPos::configureHook()
    {
      //Check and read all properties
      num_axes=num_axes_prop;
      if(gain_prop.size()!=num_axes){
	Logger::In in(this->getName().data());
	log(Error)<<"Size of K does not match num_axes" <<endlog();
	return false;
      }

      gain=gain_prop;

      //Resizing all containers to correct size
      p_meas.resize(num_axes);
      p_desi.resize(num_axes);
      offset_measurement.resize(num_axes);

      //Initialise output ports:
      v_out.assign(num_axes,0);
      v_out_port.setDataSample( v_out );
      finished_measuring_offsets_port.setDataSample(finished_event);

      offset_attr = offset_measurement;
      return true;
    }


  bool nAxesControllerPos::startHook()
  {
    Logger::In in(this->getName());
    //check connection and sizes of input-ports
    if(!p_meas_port.connected()){
      log(Error)<<p_meas_port.getName()<<" not ready"<<endlog();
      return false;
    }
    if(!p_desi_port.connected()){
      log(Error)<<p_desi_port.getName()<<" not ready"<<endlog();
      return false;
    }
    if(p_meas_port.read(p_meas)==NoData){
      log(Error)<<"No data availabe on "<< p_meas_port.getName()<<", I refuse to start!"<<endlog();
      return false;
    }
    if(p_meas.size()!=num_axes){
      log(Error)<<"Size of "<<p_meas_port.getName()<<": "<<p_meas.size()<<" != " << num_axes<<endlog();
      return false;
    }
    if(p_desi_port.read(p_desi)==NoData){
      log(Error)<<"No data availabe on "<< p_desi_port.getName()<<", I refuse to start!"<<endlog();
      return false;
    }
    if(p_desi.size()!=num_axes){
      log(Error)<<"Size of "<<p_desi_port.getName()<<": "<<p_desi.size()<<" != " << num_axes<<endlog();
      return false;
    }
    
    //Initialize
    is_measuring = false;
    
    return true;
    
  }
  
  
  void nAxesControllerPos::updateHook()
  {
    // copy Input and Setpoint to local values
    p_meas_port.read( p_meas );
    p_desi_port.read( p_desi );
    
    // position feedback
    for(unsigned int i=0; i<num_axes; i++)
      v_out[i] = gain[i] * (p_desi[i] - p_meas[i]);
    
    // measure offsets
    if (is_measuring && os::TimeService::Instance()->secondsSince(time_begin) > time_sleep){
      for (unsigned int i=0; i<num_axes; i++)
	offset_measurement[i] += v_out[i] / num_samples;
      num_samples_taken++;
      if (num_samples_taken == num_samples){
	is_measuring = false;
	offset_attr = offset_measurement;
        finished_measuring_offsets_port.write(finished_event);
      }
    }
    
    v_out_port.write( v_out );
  }
  
  
  
  void nAxesControllerPos::stopHook()
  {
    for(unsigned int i=0; i<num_axes; i++){
      v_out[i] = 0.0;
    }
    v_out_port.write( v_out );
  }
  
  bool nAxesControllerPos::startMeasuringOffsets(double _time_sleep, int _num_samples)
  {
    Logger::In in(this->getName().data());
    log(Info) <<"Start measuring offsets"<<endlog();
    
    // don't do anything if still measuring
    if (is_measuring)
      return false;
    
    // get new measurement
    else{
      for (unsigned int i=0; i<num_axes; i++){
	offset_measurement[i] = 0;
      }
      time_sleep        = max(1.0, _time_sleep);  // min 1 sec
      time_begin        = os::TimeService::Instance()->getTicks();
      num_samples       = max(1,_num_samples);    // min 1 sample
      num_samples_taken = 0;
      is_measuring      = true;
      return true;
    }
  }

}//namespace

ORO_CREATE_COMPONENT( motion_control::nAxesControllerPos )


