/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Chamzas Konstantinos <chamzask@gmail.com>
 *********************************************************************/


#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>

#include <Eigen/Geometry>

#include <QtGui/QApplication>

#include "pandora_geotiff/SaveMission.h"
#include "pandora_geotiff/geotiff_creator.h"
#include "pandora_geotiff/map_writer_plugin_interface.h"
#include "pandora_geotiff/qr_csv_creator.h"

namespace pandora_geotiff
{
  
  class MapGenerator{
    private:  
    
      GeotiffCreator* geotiffCreator;
      QrCsvCreator* qrCsvCreator; 
      std::string p_plugin_list_;
      ros::NodeHandle pn_;
      ros::ServiceServer save_mission_service;
      std::vector<boost::shared_ptr<MapWriterPluginInterface> > plugin_vector_;
      pluginlib::ClassLoader<MapWriterPluginInterface>* plugin_loader_;
      
      
    public:
      MapGenerator();
      ~MapGenerator();
    
      void writeGeotiff(const std::string& missionName);
      bool saveGeotiff(SaveMission::Request& req ,
        SaveMission::Response& res );
  };
  
}//namespace pandora_geotiff
#endif
