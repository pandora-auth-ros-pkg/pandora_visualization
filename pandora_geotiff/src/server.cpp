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

#include <boost/lexical_cast.hpp>

#include "pandora_data_fusion_msgs/GeotiffSrv.h"

#include "pandora_geotiff/SaveMission.h"
#include "pandora_geotiff/server.h"
#include "pandora_geotiff/creator.h"


namespace pandora_geotiff
{
  Server::Server()
  {
    std::string nodeName = ros::this_node::getName();

    mapReceived_ = false;
    pathReceived_ = false;
    objectsReceived_ = false;

    save_mission_ = nh_.advertiseService("geotiff/saveMission", &Server::handleRequest, this);

    /**
     * Map configuration.
     */

    nh_.param(nodeName + "/explored_map/bottom_threshold", MAP_BOTTOM_THRESHOLD, 0);
    nh_.param(nodeName + "/explored_map/top_threshold", MAP_TOP_THRESHOLD, 50);
    nh_.param(nodeName + "/explored_map/map_color", MAP_COLOR, std::string("WHITE_MAX"));

    nh_.param(nodeName + "/explored_map/wall_bottom_threshold", WALL_BOTTOM_THRESHOLD, 52);
    nh_.param(nodeName + "/explored_map/wall_top_threshold", WALL_TOP_THRESHOLD, 255);
    nh_.param(nodeName + "/explored_map/wall_color", WALL_COLOR, std::string("SOLID_BLUE"));

    /**
     * Path configuration.
     */

    nh_.param(nodeName + "/arrow/color", ARROW_COLOR, std::string("DIAMOND"));
    nh_.param(nodeName + "/arrow/size", ARROW_SIZE, 20);
    nh_.param(nodeName + "/path/color", PATH_COLOR, std::string("SOLID_ORANGE"));
    nh_.param(nodeName + "/path/width", PATH_WIDTH, 10);

    /**
     * Points of interest configuration.
     */

    nh_.param(nodeName + "/general_objects/txt_color", OBJECT_TEXT_COLOR, std::string("SOLID_ORANGE"));

    nh_.param(nodeName + "/hazmat/shape", HAZMAT_SHAPE, std::string("DIAMOND"));
    nh_.param(nodeName + "/hazmat/color", HAZMAT_COLOR, std::string("SOLID_ORANGE"));
    nh_.param(nodeName + "/hazmat/size", HAZMAT_SIZE, 1);

    nh_.param(nodeName + "/qr/shape", QR_SHAPE, std::string("DIAMOND"));
    nh_.param(nodeName + "/qr/color", QR_COLOR, std::string("SOLID_ORANGE"));
    nh_.param(nodeName + "/qr/size", QR_SIZE, 1);

    nh_.param(nodeName + "/victim/shape", VICTIM_SHAPE, std::string("DIAMOND"));
    nh_.param(nodeName + "/victim/color", VICTIM_COLOR, std::string("SOLID_ORANGE"));
    nh_.param(nodeName + "/victim/size", VICTIM_SIZE, 1);

    /**
     * Topics.
     */

    nh_.param(nodeName + "/topics/map", MAP_TOPIC, std::string("/slam/map"));
    nh_.param(nodeName + "/topics/trajectory", PATH_TOPIC, std::string("/trajectory"));
    nh_.param(nodeName + "/topics/coverage", COVERAGE_TOPIC, std::string("/data_fusion/sensor_coverage/kinect_space"));

    /**
     * Services.
     */

    nh_.param(nodeName + "/services/data_fusion_objects", OBJECTS_SERVICE, std::string("data_fusion_geotiff"));

    /**
     * Register subscribers and start listening for input.
     */

    mapSubscriber_ = nh_.subscribe(MAP_TOPIC, 1000, &Server::receiveMap, this);
    pathSubscriber_ = nh_.subscribe(PATH_TOPIC, 1000, &Server::receivePath, this);
    coverageSub_ = nh_.subscribe(COVERAGE_TOPIC, 1000, &Server::receiveCoverageMap, this);

    /**
     * Register Data Fusion's service.
     */

    objectService_ = nh_.serviceClient<pandora_data_fusion_msgs::GeotiffSrv>(OBJECTS_SERVICE);

    ROS_INFO("Geotiff node started.");
  }

  Server::~Server()
  {
    ROS_INFO("Destroying geotiff server...");
  }

  void Server::getObjects()
  {
    pandora_data_fusion_msgs::GeotiffSrv dataFusionSrv;

    ROS_INFO("Calling %s", objectService_.getService().c_str());

    if (!objectService_.call(dataFusionSrv)) {
      ROS_ERROR("Cannot receive Objects, service %s has failed.", objectService_.getService().c_str());
      return;
    }

    ROS_INFO("Service %s responded.", objectService_.getService().c_str());

    victims_ = dataFusionSrv.response.victims;
    qrs_ = dataFusionSrv.response.qrs;
    hazmats_ = dataFusionSrv.response.hazmats;

    objectsReceived_ = true;
  }

  void Server::drawObject(std::vector<geometry_msgs::PoseStamped> &object, std::string &color, std::string &shape,
                          int size)
  {
    Eigen::Vector2f coords;
    std::string txt;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    tfScalar pitch;
    tfScalar roll;
    tfScalar yaw;

    tf::Vector3 origin;

    float x;
    float y;

    for (int i = 0; i < object.size(); i++) {
      try {
        listener.waitForTransform("/map", object[i].header.frame_id, object[i].header.stamp, ros::Duration(3));
        listener.lookupTransform ("/map", object[i].header.frame_id, object[i].header.stamp, transform);
      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      /**
       * Get origin and roll, pitch, yaw.
       */

      transform.getBasis().getRPY(roll, pitch, yaw);
      origin = transform.getOrigin();

      /**
       * Add the origin vector.
       */

      x = object[i].pose.position.x + origin.x();
      y = object[i].pose.position.y + origin.y();

      /**
       * Rotate according to yaw and pitch.
       */

      coords = Eigen::Vector2f(x * cos(yaw) + y * sin(yaw), y * cos(yaw) + x * sin(yaw));
      txt = boost::lexical_cast<std::string> (i + 1);

      creator_.drawPOI(coords, color, OBJECT_TEXT_COLOR, shape, txt, size);
    }
  }

  void Server::drawObjects()
  {
    this -> getObjects();

    if (!objectsReceived_) {
      ROS_ERROR("Objects are not available");
      return;
    }

    ROS_INFO("Drawing Data Fusion's objects.");

    this -> drawObject(hazmats_, HAZMAT_COLOR, HAZMAT_SHAPE, HAZMAT_SIZE);
    this -> drawObject(qrs_, QR_COLOR, QR_SHAPE, QR_SIZE);
    this -> drawObject(victims_, VICTIM_COLOR, VICTIM_SHAPE, VICTIM_SIZE);
  }

  bool Server::handleRequest(SaveMission::Request &req, SaveMission::Response &res)
  {
    ROS_INFO("SaveMission service was requested.");

    missionName_ = req.SaveMisionFileName.data;
    this -> createGeotiff(req.SaveMisionFileName.data);
    return true;
  }

  void Server::receiveMap(const nav_msgs::OccupancyGrid &map)
  {

    if (!mapReceived_) {
      ROS_INFO("Received map.");
      mapReceived_ = true;
    }

    map_ = map;
  }

  void Server::receivePath(const nav_msgs::Path &path)
  {

    if (!pathReceived_) {
      ROS_INFO("Received path.");
      pathReceived_ = true;
    }

    path_ = path;
  }

  void Server::receiveCoverageMap(const nav_msgs::OccupancyGrid &map)
  {
  }

  void Server::drawMap()
  {
    creator_.drawMap(map_, MAP_COLOR, MAP_BOTTOM_THRESHOLD, MAP_TOP_THRESHOLD, 1);
    creator_.drawMap(map_, WALL_COLOR, WALL_BOTTOM_THRESHOLD, WALL_TOP_THRESHOLD, 0);
  }

  void Server::drawPath()
  {
    ROS_INFO("Drawing the path...");

    std::vector<geometry_msgs::PoseStamped> &path_vector(path_.poses);

    size_t size = path_vector.size();

    std::vector<Eigen::Vector2f> pointVector;
    pointVector.resize(size);

    ROS_INFO("Path size: %u", size);

    for (size_t i = 0; i < size; ++i) {
      const geometry_msgs::PoseStamped &pose(path_vector[i]);
      pointVector[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
    }

    if (size > 0) {
      creator_.drawPath(pointVector, PATH_COLOR, PATH_WIDTH);
      creator_.drawPOI(pointVector[0], ARROW_COLOR, "", "ARROW", "", ARROW_SIZE);
    }

    ROS_INFO("The robot's path is ready.");
  }

  void Server::createGeotiff(const std::string &fileName)
  {
    ROS_INFO("Starting geotiff creation for mission: %s.", fileName.c_str());

    if (!mapReceived_)
      ROS_ERROR("Map is not available.");

    if (!pathReceived_)
      ROS_ERROR("Path is not available.");

    this -> drawMap();
    this -> drawPath();
    this -> drawObjects();

    creator_.createBackgroundImage();

    // Save geotiff to the home directory.
    creator_.saveGeotiff("");

    // Reset the environment.
    mapReceived_ = false;
    pathReceived_ = false;
  }

}  // namespace pandora_geotiff

