// 파일명 traffic_right_left_plugin.cpp

#include "turtlebot3_gazebo/traffic_right_left_plugin.hpp"

namespace gazebo
{
TrafficRightLeft::TrafficRightLeft()
: traffic_cycle(5.0),
  status(0),
  textures{"traffic_right", "traffic_left"}
{
}
TrafficRightLeft::~TrafficRightLeft()
{
}
void TrafficRightLeft::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
  this->visPub = this->node->Advertise<gazebo::msgs::Visual>("/gazebo/default/visual");
  this->msg.set_name(this->model->GetName() + "::traffic_right_left");
  this->msg.set_parent_name(this->model->GetName());
  std::cout << this->model->GetName() << " plugin load success!" << std::endl;
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&TrafficRightLeft::OnUpdate, this));
  this->last_time = this->world->SimTime();
}
void TrafficRightLeft::OnUpdate()
{
  common::Time current_time = this->world->SimTime();
  if ((current_time - this->last_time).Double() >= this->traffic_cycle) {
    // 상태 0 → 1 → 0 → 1 순서로 반복
    status = (status + 1) % 2;  // 상태를 0 또는 1로 토글
    traffic_cycle = 2.0;        // 주기

    // 텍스처 이름 설정
    msg.mutable_material()->mutable_script()->set_name(textures[status]);
    // 메시지 발행
    visPub->Publish(this->msg);
    // 시간 갱신
    this->last_time = current_time;

    // std::cout << "Changing texture to: " << textures[status] << std::endl;
  }
}
}  // namespace gazebo
