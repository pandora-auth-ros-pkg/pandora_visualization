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
 *   Sideris Konstantinos <siderisk@auth.gr>
 *********************************************************************/

#ifndef PANDORA_GEOTIFF_SERVER_H
#define PANDORA_GEOTIFF_SERVER_H

#include <vector>
#include <map>
#include <string>
#include <sys/types.h>

#include <unistd.h>
#include <pwd.h>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

#include "pandora_geotiff/SaveMission.h"

/**
 * Geotiff layers.
 */

#include "pandora_geotiff/creator.h"


namespace pandora_geotiff
{
  class Server
  {
    public:
      /**
       * @brief Server constructor
       */

      Server();

      /**
       * @brief Server destructor
       */

      ~Server();

      /**
       * @brief Receive geotiff requests.
       */

      bool handleRequest(SaveMission::Request &req, SaveMission::Response &res);

      /**
       * @brief Receive the map produced by slam.
       */

      void receiveMap(const nav_msgs::OccupancyGrid &map);

      /**
       * @brief Receive the robot's trajectory.
       */

      void receivePath(const nav_msgs::Path &path);

      void receiveCoverageMap(const nav_msgs::OccupancyGrid &map);

      void drawMap();

    private:

      /**
       * @brief Sets the MissionName.
       */

      void setMissionName(const std::string &missionName);

      /**
       * @brief Create the geotiff.
       */

      void createGeotiff(const std::string &fileName);

      //!< The name of the mission.
      std::string missionName_;

      //!< The mission prefix. e.g: "/RRL_2015_PANDORA_"
      std::string missionNamePrefix_;

      //!< The file extension e.g: ".tiff"
      std::string missionNameExtention_;

      //!< ROS node handler.
      ros::NodeHandle nh_;

      //!< ROS Service to handle request for the geotiff creation.
      ros::ServiceServer save_mission_;

      //!< Subscriber to receive the map.
      ros::Subscriber mapSubscriber_;

      //!< Subscriber to the robot's path.
      ros::Subscriber pathSubscriber_;

      //!< Subscriber to the covered area.
      ros::Subscriber coverageSub_;

      //!< Object to draw the geotiff.
      Creator creator_;

      //!< Map received from SLAM.
      nav_msgs::OccupancyGrid map_;

      /**
       * Topics.
       */

      std::string MAP_TOPIC;
      std::string PATH_TOPIC;
      std::string COVERAGE_TOPIC;

      /**
       * Flags.
       */

      bool mapReceived_;
      bool pathReceived_;

      /**
       * Map parameters.
       */

      std::string MAP_COLOR;
      int MAP_BOTTOM_THRESHOLD;
      int MAP_TOP_THRESHOLD;

      std::string WALL_COLOR;
      int WALL_BOTTOM_THRESHOLD;
      int WALL_TOP_THRESHOLD;

  };
}  // namespace pandora_geotiff

#endif  // PANDORA_GEOTIFF_SERVER_H
