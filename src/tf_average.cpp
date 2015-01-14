#include <tf/tf.h>
#include <iostream>

int main (int argc, char ** argv) 
{
  tf::Transform t1(tf::Quaternion(tf::Vector3(0,0,1),-.8), tf::Vector3(1,1,1));
  tf::Transform t2(tf::Quaternion(tf::Vector3(0,0,1),.2), tf::Vector3(2,2,2));
  
  t1.setOrigin(t1.getOrigin() / 2);
  t2.setOrigin(t2.getOrigin() / 2);
  t1.setRotation(t1.getRotation() / 2);
  t2.setRotation(t2.getRotation() / 2);

  tf::Transform res(tf::Quaternion(tf::Vector3(0,0,1),0));
  res.setOrigin(t1.getOrigin() + t2.getOrigin());
  res.setRotation(t1.getRotation() + t2.getRotation());
  std::cout << res.getOrigin().getX() << std::endl;
  std::cout << res.getRotation().getAngle() << std::endl;
}
