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

#ifndef PANDORA_GEOTIFF_GEOTIFF_CREATOR_H
#define PANDORA_GEOTIFF_GEOTIFF_CREATOR_H

#include <vector>
#include <map>
#include <string>
#include <sys/types.h>

#include <QtGui>
#include <QAction>
#include <unistd.h>
#include <pwd.h>

#include "ros/ros.h"
#include "pandora_geotiff/map_creator_interface.h"


namespace pandora_geotiff {

  class GeotiffCreator : public MapWriterInterface {

    public:

      /**
       * @brief GeotiffCreator constructor
       */

      GeotiffCreator();

      /**
       * @brief GeotiffCreator destructor
       */

      ~GeotiffCreator() {};

      /**
       * @brief Creates all the GeotiffBackgroundIm.
       *
       * @details It includes TheCheckers, the Orientation, the Map Scale
       *          and the MissionName.
       * @warning This function must be called after the MapIm has been
       *          created or its size is known.
       */

      void createBackgroundIm();

      /**
       * @brief save the GeotiffFinalGeotiffImg to the specified path
       */

      void saveGeotiff(std::string homeFolderString = "/Desktop");

      /**
       * @brief sets the MissionName
       */

      void setMissionName(std::string missionName);


      virtual void drawMap(const nav_msgs::OccupancyGrid& map,
                           const std::string& color,
                           const int& bottomThres,
                           const int& topThres,
                           const int& grid_space = 0);

      virtual void drawObjectOfInterest(const Eigen::Vector2f& coords,
                                        const std::string& color,
                                        const std::string& txtcolor,
                                        const std::string& shape,
                                        const std::string& txt,
                                        const int& size);

      virtual void drawPath(const std::vector<Eigen::Vector2f>& points,
                            const std::string& color,
                            const int& width);

    private:

      void geotiffTimerCb(const ros::TimerEvent& event);

      /**
       * @brief Draws the checkers of specified size and color.
       *
       * @param colorD [&std::string] The Dark Color of the checkers.
       * @param colorL [&std::string] The Light Color of the checkers.
       * @param checkerSize [&std::string] The size of the checkers.
       *
       * @return void
       */

      void drawCheckers(const int& checkerSize, const std::string& colorD,
                        const std::string& colorL, QPainter* geotiffPainter);

      /**
       * @brief Draws the missionName with a specific color in a specific point.
       *
       * @param coords [&Eigen::Vector2f]  The coordinates of the missionName.
       * @param color [&std::string]  The color the missionName is painted.
       * @param width [&int]  The width of the pen that will be used.
       *
       * @return void
       */

      void drawMissionName(const Eigen::Vector2f& coords,
                           const std::string& color,
                           const int& width, QPainter* geotiffPainter);
      /**
       * @brief Draws the mapScale with a specific color in a specific point.
       *
       * @param coords [&Eigen::Vector2f] The coordinates of the mapscale.
       * @param color [&std::string] The color the mapScale.
       * @param width [int] The width of the pen tha will be used.
       * @param size  [int] The size of the Mapscale.
       *
       * @return void
       */

      void drawMapScale(const Eigen::Vector2f& coords,
                        const std::string& color,
                        const int& width, QPainter* geotiffPainter);

      /**
       * @brief Draws the mapOrientation with a specific color in a specific point.
       *
       * @detail The length of the arrow is decided by the size of the checker
       * @param coords [Eigen::Vector2f] The coordinates of the the mapOrientation (the point each line meets).
       * @param color [std::string] The color the mapOrientation.
       * @param width [int] The width of the pen tha will be used.
       *
       * @return void
       */

      void drawMapOrientation(const Eigen::Vector2f& coords,
                              const std::string& color,
                              const int& width, QPainter* geotiffPainter);

      Eigen::Vector2i  transformFromMetersToGeotiffPos(const Eigen::Vector2f point);

      //!< A Map that correlates all the colors name to string Colors.
      std::map<std::string, QColor> colorMap;

      // I declare these as pointers in case the are moved in another class.

      //!< The background image.
      QImage* geotiffBackgroundIm_;

      //!< The Map image.
      QImage* geotiffMapIm_;

      //!< The Map image and the Background image.
      QImage* geotiffFinalIm_;

      //!< The name of the mission.
      std::string missionName_;

      //!< The mission prefix. e.g: "/RRL_2015_PANDORA_"
      std::string missionNamePrefix_;

      //!< The file extension e.g: ".tiff"
      std::string missionNameExtention_;

      QApplication* app_;

      int fake_argc_;

      char** fake_argv_;

      // Map parameters

      int mapXoffset_;
      int mapYoffset_;

      int trimmingXoffset_;
      int trimmingYoffset_;

      float geotiffMapRes_;
      bool mapInitialized_;

      int CHECKER_SIZE;
      int MAP_OFFSET;
      int MISSION_NAME_WIDTH;
      int MAP_SCALE_WIDTH;
      int MAP_ORIENTATION_WIDTH;
      int MAP_ORIENTATION_LENGTH;

      std::string CHECKER_COLOR_LIGHT;
      std::string CHECKER_COLOR_DARK;
      std::string MISSION_NAME_COLOR;
      std::string MAP_SCALE_COLOR;
      std::string MAP_ORIENTATION_COLOR;

      Eigen::Vector2f MISSION_NAME_COORDS;
      Eigen::Vector2f MAP_SCALE_COORDS;
      Eigen::Vector2f MAP_ORIENTATION_COORDS;
  };

}  // namespace pandora_geotiff

#endif  // PANDORA_GEOTIFF_GEOTIFF_CREATOR_H
