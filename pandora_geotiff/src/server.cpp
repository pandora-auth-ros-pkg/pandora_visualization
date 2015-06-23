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

#include <string>
#include <vector>

#include "pandora_geotiff/SaveMission.h"
#include "pandora_geotiff/server.h"

#include "pandora_geotiff/creator.h"


namespace pandora_geotiff
{
  Server::Server()
  {
    save_mission_ = nh_.advertiseService("geotiff/saveMission", &Server::handleRequest, this);
    mapSubscriber_ = nh_.subscribe("/slam/map", 1000, &Server::receiveMap, this);

    /**
     * Geotiff configuration.
     */

    ROS_INFO("Geotiff node started.");
  }

  Server::~Server()
  {
    ROS_INFO("Destroying geotiff server...");
  }

  bool Server::handleRequest(SaveMission::Request &req, SaveMission::Response &res)
  {
    ROS_INFO("SaveMission service was requested.");

    missionName_ = req.SaveMisionFileName.data;
    this -> createGeotiff(req.SaveMisionFileName.data);
    return true;
  }

  void Server::receiveMap(nav_msgs::OccupancyGrid map)
  {
    ROS_INFO("Received map.");
  }

  void Server::createGeotiff(const std::string &fileName)
  {
    ROS_INFO("Starting geotiff creation of mission: %s.", fileName.c_str());

    creator_.createBackgroundImage();

    // Save geotiff to the home directory.
    creator_.saveGeotiff("/");
  }

}  // namespace pandora_geotiff

