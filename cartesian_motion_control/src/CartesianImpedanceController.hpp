/**************************************************************************
 *                (C) 2011 Ruben Smits                                    *
 *                ruben.smits@mech.kuleuven.be                            *
 *               Department of Mechanical Engineering,                    *
 *              Katholieke Universiteit Leuven, Belgium.                  *
 *                                                                        *
 *  You may redistribute this software and/or modify it under either the  *
 *  terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
 *  <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
 *  discretion) of the Modified BSD License:                              *
 *  Redistribution and use in source and binary forms, with or without    *
 *  modification, are permitted provided that the following conditions    *
 *  are met:                                                              *
 *  1. Redistributions of source code must retain the above copyright     *
 *  notice, this list of conditions and the following disclaimer.         *
 *  2. Redistributions in binary form must reproduce the above copyright  *
 *  notice, this list of conditions and the following disclaimer in the   *
 *  documentation and/or other materials provided with the distribution.  *
 *  3. The name of the author may not be used to endorse or promote       *
 *  products derived from this software without specific prior written    *
 *  permission.                                                           *
 *  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
 *  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
 *  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
 *  POSSIBILITY OF SUCH DAMAGE.                                           *
 *                                                                        *
 **************************************************************************/

#ifndef __CARTESIANIMPEDANCECONTROLLER_HPP__
#define __CARTESIANIMPEDANCECONTROLLER_HPP__

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames.hpp>

namespace MotionControl{


    class CartesianImpedanceController : public RTT::TaskContext
    {
    public:
        CartesianImpedanceController(const string& name): RTT::TaskContext(name){
            this->addEventPort("CartesianSensorPosition",port_pose_meas);
            this->addEventPort("CartesianDesiredPosition",port_pose_desi);
            this->addPort("CartesianOutputWrench",port_wrench_out);

            this->addProperty("stiffness",m_stiffness).doc("Cartesian Stiffness in tool frame");

        };
        
    private:
        bool startHook(){
            if(port_pose_meas.read(m_pose_desi) == RTT::NoData )
                return false;
            port_pose_desi.clear();
            return true;
        };
        void updateHook(){
            port_pose_meas.read(m_pose_meas);
            port_pose_desi.read(m_pose_desi);
            KDL::Frame pm,pd;
            tf::PoseMsgToKDL(m_pose_meas,pm);
            tf::PoseMsgToKDL(m_pose_desi,pd);
            KDL::Twist delta = diff(pm.Inverse(),pd.Inverse());
            m_wrench_out.force.x = m_stiffness.linear.x*(delta.vel.x());
            m_wrench_out.force.y = m_stiffness.linear.y*(delta.vel.y());
            m_wrench_out.force.z = m_stiffness.linear.z*(delta.vel.z());
            m_wrench_out.torque.x = m_stiffness.angular.x*(delta.rot.x());
            m_wrench_out.torque.y = m_stiffness.angular.y*(delta.rot.y());
            m_wrench_out.torque.z = m_stiffness.angular.z*(delta.rot.z());
            port_wrench_out.write(m_wrench_out);
        };
        void stopHook(){
            m_wrench_out.force.x=0;
            m_wrench_out.force.y=0;
            m_wrench_out.force.z=0;
            m_wrench_out.torque.x=0;
            m_wrench_out.torque.y=0;
            m_wrench_out.torque.z=0;
            port_wrench_out.write(m_wrench_out);
        };

        geometry_msgs::Pose m_pose_meas, m_pose_desi;
        geometry_msgs::Wrench m_wrench_out;
        geometry_msgs::Twist m_stiffness;
        
        RTT::InputPort<geometry_msgs::Pose> port_pose_meas, port_pose_desi;
        RTT::OutputPort<geometry_msgs::Wrench> port_wrench_out;
        
    };
}//namespace
        
#endif
