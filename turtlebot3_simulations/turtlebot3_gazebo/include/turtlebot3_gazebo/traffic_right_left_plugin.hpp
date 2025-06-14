// #ifndef TURTLEBOT3_GAZEBO__TRAFFIC_RIGHT_LEFT_PLUGIN_HPP_
// #define TURTLEBOT3_GAZEBO__TRAFFIC_RIGHT_LEFT_PLUGIN_HPP_
#ifndef TURTLEBOT3_GAZEBO__TRAFFIC_LIGHT_PLUGIN_HPP_
#define TURTLEBOT3_GAZEBO__TRAFFIC_LIGHT_PLUGIN_HPP_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class TrafficRightLeft : public ModelPlugin  // ✅ 클래스명 변경
{
public:
  TrafficRightLeft();
  ~TrafficRightLeft();

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate();

private:
  double traffic_cycle;
  int status;
  std::vector<std::string> textures;

  common::Time last_time;
  event::ConnectionPtr update_connection;
  gazebo::transport::NodePtr node;
  gazebo::msgs::Visual msg;
  gazebo::transport::PublisherPtr visPub;
  physics::ModelPtr model;
  physics::WorldPtr world;
};
GZ_REGISTER_MODEL_PLUGIN(TrafficRightLeft);  // ✅ 플러그인 등록 클래스명 변경
}  // namespace gazebo

#endif  // TURTLEBOT3_GAZEBO__TRAFFIC_RIGHT_LEFT_PLUGIN_HPP_
