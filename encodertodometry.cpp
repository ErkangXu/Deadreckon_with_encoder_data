#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <clearpath_base/Encoders.h>

double _PreviousLeftEncoderCounts = 0;
double _PreviousRightEncoderCounts = 0;
double leftEncoderVelocity;
double rightEncoderVelocity;
double averageVelocity;

double x=0;
double y=0;
double th=0;

double vx;
double vy;
double vth;

double deltaLeft;
double deltaRight;

//tf::TransformBroadcaster odom_broadcaster;
ros::Publisher *publisherPointer_ptr;
ros::Time *current_time_encoder_ptr;
ros::Time *last_time_encoder_ptr;

void WheelCallback(const clearpath_base::Encoders::ConstPtr& encoder_msg)
{

  *current_time_encoder_ptr = ros::Time::now();
  double dt = (*current_time_encoder_ptr-*last_time_encoder_ptr).toSec();
  bool turned_or_not=true;
  double radius=0;
  double delta_x=0;
  double delta_y=0;
  leftEncoderVelocity=encoder_msg->encoders[0].speed;
  rightEncoderVelocity=encoder_msg->encoders[1].speed;
  averageVelocity=(leftEncoderVelocity+rightEncoderVelocity)/2; //average speed is the average of left speed and right speed
  
  
  deltaLeft = encoder_msg->encoders[0].travel - _PreviousLeftEncoderCounts;
  deltaRight = encoder_msg->encoders[1].travel - _PreviousRightEncoderCounts;
  
  if (deltaLeft==deltaRight) // both sides go the same distance, so the facing angle will not change, cover the stand still condition
  {
    delta_x=deltaLeft*cos(th);
    delta_y=deltaLeft*sin(th);
    x+=delta_x;
    y+=delta_y;
    turned_or_not=false;
    vth=0;
  }
  
  if ((deltaLeft>0) & (deltaRight>0) & (deltaLeft!=deltaRight))  //both sides going forward but there are not equal
  {
    bool turn_direction;
    double longarc;
    double shortarc;
    double deltaangle;
    if (deltaLeft-deltaRight) //right turn(clockwise)
      {
	turn_direction=false;
	longarc=deltaLeft;
	shortarc=deltaRight;
      }
    else  //left turn(conterclockwise) 
      {
	turn_direction=true;
	shortarc=deltaLeft;
	longarc=deltaRight;
      }
      
    deltaangle=(longarc-shortarc)/0.49;  // always use the obsolute value of the angle change
    radius=longarc/deltaangle-0.245; // the distance between turning center and the center of the robot
      
    delta_x=sin(deltaangle)*radius*cos(th);    // should use the old "th" here
    //sin(deltaangle)*radius is the displacement on the direction at the last time of record. 
    //Then we multiply is by cos(th) to convert it tothe displacement on the  direction of x axis(the initial orientation of the robot)
    delta_y=sin(deltaangle)*radius*sin(th); // we need to rotate x axis 90 degree conterclockwise to get y axis
    
    if(turn_direction)
    {
    th+=deltaangle; //conterclockwise turn (should increase the degree)
    vth=deltaangle/dt;
    }
    else 
    {
    th-=deltaangle; //clockwise turn (should decrease the degree)
    vth=-deltaangle/dt;
    }
    
    x+=delta_x;
    y+=delta_y;
    
   }
   
   else if ((deltaLeft<0) & (deltaRight<0) & (deltaLeft!=deltaRight))
   {
      bool turn_direction;
      double longarc;
      double shortarc;
      double deltaangle;
      if (deltaLeft-deltaRight) //right turn(clockwise)  left side back less than the right side
	{
	  turn_direction=false; //turn clockwise
	  longarc=-deltaRight;
	  shortarc=-deltaLeft;
	}
      else  //left turn(conterclockwise) 
	{
	  turn_direction=true;
	  shortarc=-deltaRight;
	  longarc=-deltaLeft;
	}
	
      deltaangle=(longarc-shortarc)/0.49;  // always use the obsolute value of the angle change
      radius=longarc/deltaangle-0.245;
      
      delta_x=-sin(deltaangle)*radius*cos(th);    // should use the old "th" here
      //-sin(deltaangle)*radius is the displacement on the direction at the last time of record. 
      //Then we multiply is by cos(th) to convert it tothe displacement on the  direction of x axis(the initial orientation of the robot)
      delta_y=-sin(deltaangle)*radius*sin(th);
      
      if(turn_direction)
      th+=deltaangle; //conterclockwise turn (should increase the degree)
      else 
      th-=deltaangle; //clockwise turn (should decrease the degree)
      
      x+=delta_x;
      y+=delta_y;
      vth=deltaangle/dt;
      
   }
   
   else if (deltaLeft*deltaRight<0) // one side of wheels go one direction the other side go the opposite direction (skiddingly rotate in the same place).
    // for the sake of simplicity, i will not calculate the lnear displacement of the robot(it's neglectable actually)
   {  
      bool rotatedirection;
      double pos_arc;
      double neg_arc;
      double rotateangle;
      
      if (deltaLeft<0)
      {
	rotatedirection=true;
	pos_arc=deltaRight;
	neg_arc=deltaLeft;
	rotateangle=(pos_arc-neg_arc)/0.49;
	vth=rotateangle/dt;
	th+=rotateangle;	
      }
      else
      {
	rotatedirection=false;
	pos_arc=deltaLeft;
	neg_arc=deltaRight;
	rotateangle=-(pos_arc-neg_arc)/0.49;  // unlike deltaangle, rotateangle has direction
	vth=rotateangle/dt;
	th+=rotateangle;
      }
      
    }
    
    
    else if ((deltaLeft*deltaRight==0) & (deltaLeft!=deltaRight))// one side of wheels move the other side of wheels don't move (skidding rotation)
    {
      double rotatedegree;
      radius=0;
      if(deltaLeft==0)
      {
	rotatedegree=deltaRight/0.49; //no matter the right side was moving forward or backward
	vth=rotatedegree/dt;
	th+=rotatedegree;
      }
      if(deltaRight==0)
      {
	rotatedegree=-deltaLeft/0.49; //no matter the right side was moving forward or backward
	vth=rotatedegree/dt;
	th+=rotatedegree;
      }
    } 
    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    /*
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_encoder;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    */ 
    // since we no longer use navigation stack we don't need to publish on tf any more!
      
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = *current_time_encoder_ptr;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    //odom.pose.pose.orientation = odom_quat;    Adam only need the angle with the world coordinate, so commented
    odom.pose.pose.orientation.w = th;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.angular.z = vth;
    if(turned_or_not)
    {
      //odom.twist.twist.linear.x = averageVelocity*cos(th);   Adam Only need pure velocity   so commented
      //odom.twist.twist.linear.y = averageVelocity*sin(th);
      odom.twist.twist.linear.x = averageVelocity;
      odom.twist.twist.linear.y = 0;
    }
    else
    {
      odom.twist.twist.linear.x = 0; // if it was just rotating at the same place. the linear speed should be zero.  
      odom.twist.twist.linear.y = 0;
    }
    
    //publish the message
    publisherPointer_ptr->publish(odom); 

  _PreviousLeftEncoderCounts = encoder_msg->encoders[0].travel;
  _PreviousRightEncoderCounts = encoder_msg->encoders[1].travel;
  *last_time_encoder_ptr = *current_time_encoder_ptr;
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  ros::Time current_time_encoder;
  ros::Time last_time_encoder;
  //tf::TransformBroadcaster odom_broadcaster;
 
  current_time_encoder_ptr=&current_time_encoder;
  last_time_encoder_ptr=&last_time_encoder;
  
  last_time_encoder=ros::Time::now();
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);  
  publisherPointer_ptr = &odom_pub; 
  ros::Subscriber sub = n.subscribe("/clearpath/robots/husky/data/encoders", 100, WheelCallback);
  ros::spin();
}