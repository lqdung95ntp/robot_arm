#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/JointState.h"
// #include "robot_state_publisher/robot_state_publisher.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"
#include "visualization_msgs//Marker.h"

#define NUMBER_OF_POINT 1000
#define PI 3.14159268

// RobotStatePublisher(const KDL::Tree& tree);
geometry_msgs::Point p, pre_p[NUMBER_OF_POINT];
float x_end, y_end, z_end;
float x_last=0, y_last=0, z_last=0;

void transformPoint(const tf::TransformListener& listener, ros::Publisher& pub){
    geometry_msgs::PointStamped end_effector;
    end_effector.header.frame_id="L3";

    end_effector.header.stamp=ros::Time();

    end_effector.point.x=0;
    end_effector.point.y=0;
    end_effector.point.z=0;

    try{
        geometry_msgs::PointStamped base_end_effector;
        listener.transformPoint("base_link", end_effector, base_end_effector);

        ROS_INFO("end_effector: (%.2f, %.2f, %.2f)",
        // end_effector.point.x*1000, end_effector.point.y*1000, end_effector.point.z*1000, 
        base_end_effector.point.x*1000, base_end_effector.point.y*1000, -(base_end_effector.point.z*1000-290));

        //publish end_effector position in milimet:
        // posPublish.header.stamp=ros::Time::now();
        x_end=base_end_effector.point.x*1000;
        y_end=base_end_effector.point.y*1000;
        z_end=-(base_end_effector.point.z*1000-290);
        

        //draw out
        visualization_msgs::Marker point;
        point.header.frame_id="/base_link";
        point.header.stamp=ros::Time::now();
        point.ns="points_and_lines";
        point.action=visualization_msgs::Marker::ADD;
        point.pose.orientation.w=1.0;
        point.id=0;
        point.type=visualization_msgs::Marker::POINTS;
        point.scale.x=0.005;
        point.scale.y=0.005;
        point.color.g=1.0f;
        point.color.a=1.0;

        for (int i=0; i<NUMBER_OF_POINT-1;i++){
            pre_p[i].x=pre_p[i+1].x;
            pre_p[i].y=pre_p[i+1].y;
            pre_p[i].z=pre_p[i+1].z;
            point.points.push_back(pre_p[i]);
        }
        pre_p[NUMBER_OF_POINT-1].x=p.x;
        pre_p[NUMBER_OF_POINT-1].y=p.y;
        pre_p[NUMBER_OF_POINT-1].z=p.z;
        point.points.push_back(pre_p[NUMBER_OF_POINT-1]);
        p.x=base_end_effector.point.x;
        p.y=base_end_effector.point.y;
        p.z=base_end_effector.point.z;
        point.points.push_back(p);

        pub.publish(point);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from L3 to base_link: %s", ex.what());
    }
}

/*---------------------------------------------------------------------------------------
 *---------------------------------------- DRAW LINE ------------------------------------
 (---------------------------------------------------------------------------------------*/

float theta1[10000], d2[10000], d3[10000];
int index_count=0;

void inverse_kinematic(float x, float y, float z, int totalPoint, int i_now){
    theta1[index_count]=atan(-x/y);
    d2[index_count]=-0.25+(y/cos(atan(-x/y)));
    d3[index_count]=0.29-(0.29-z);
    index_count++;
}

void draw_start_point(float x, float y, float z){
    float x_temp, y_temp, z_temp;
    for (int i=0;i<=100;i++){
        theta1[index_count]=atan(-x/y)/100.0*i;
        d2[index_count]=-0.25+(y/cos(atan(-x/y)))/100.0*i;
        d3[index_count]=0.29-z/100.0*i;
        index_count++;
    }
    x_last=x;
    y_last=y;
    z_last=z;
}


void draw_line(float x1, float y1, float z1, float x2, float y2, float z2){
    float length=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2));
    if (length==0) return;
    int numberOfPoints=(int)(length*1000)+1;
    float x_temp, y_temp, z_temp;
    for (int i=0; i<=numberOfPoints;i++){
        x_temp=(x2-x1)*i/numberOfPoints+x1;
        y_temp=(y2-y1)*i/numberOfPoints+y1;
        z_temp=(z2-z1)*i/numberOfPoints+z1;
        if (y_temp==0) y_temp=0.00001;
        theta1[index_count]=atan(-x_temp/y_temp);
        d2[index_count]=-0.25+(y_temp/cos(atan(-x_temp/y_temp)));
        d3[index_count]=0.29-(z_temp);
        index_count++;
        // inverse_kinematic(x_temp, y_temp, z_temp, numberOfPoints, i);
    }
    x_last=x2;
    y_last=y2;
    z_last=z2;
}

float draw_circle(float x0, float y0, float z0, float r, int matphang=1){
    //default: matphang=xOy
    float y_start=y0-r;
    draw_line(x_last,y_last,z_last, x0, y_start, z0);
    float chuvi=2*PI*r;
    int numberOfPoints=(int)(chuvi*1000)+1;
    float x_temp, y_temp, z_temp;
    for (int i=0; i<=numberOfPoints/2;i++){
        y_temp=i/(numberOfPoints/2.0)*(2*r)+y_start;
        x_temp=sqrt(r*r-(y0-y_temp)*(y0-y_temp))+x0;
        z_temp=z0;
        if (y_temp==0) y_temp=0.00001;
        theta1[index_count]=atan(-x_temp/y_temp);
        d2[index_count]=-0.25+(y_temp/cos(atan(-x_temp/y_temp)));
        d3[index_count]=0.29-(z_temp);
        index_count++;
    }
    for (int i=0;i<=numberOfPoints/2;i++){
        y_temp=-i/(numberOfPoints/2.0)*2*r+(y_start+2*r);
        x_temp=-sqrt(r*r-(y0-y_temp)*(y0-y_temp))+x0;
        z_temp=z0;
        if (y_temp==0) y_temp=0.00001;
        theta1[index_count]=atan(-x_temp/y_temp);
        d2[index_count]=-0.25+(y_temp/cos(atan(-x_temp/y_temp)));
        d3[index_count]=0.29-(z_temp);
        index_count++;
    }
    x_last=x0;
    y_last=y0-r;
    z_last=z0;
}

void end_point(){
    for (int i=10000; i>index_count;i--){
        theta1[i]=theta1[index_count-10];
        d2[i]=d2[index_count-10];
        d3[i]=d3[index_count-10];
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "robot_arm");
    ros::NodeHandle n;
    ros::Publisher robotMsg=n.advertise<sensor_msgs::JointState>("robotJointState", 1000);
    ros::Publisher marker_pub=n.advertise<visualization_msgs::Marker>("visualization_marker",10);
    ros::Publisher x_position=n.advertise<std_msgs::Int32>("x_pos",1);
    ros::Publisher y_position=n.advertise<std_msgs::Int32>("y_pos",1);
    ros::Publisher z_position=n.advertise<std_msgs::Int32>("z_pos",1);
    
    sensor_msgs::JointState jointstate;
    std_msgs::Int32 x_positionPub;
    std_msgs::Int32 y_positionPub;
    std_msgs::Int32 z_positionPub;

    jointstate.name.resize(3);
    jointstate.position.resize(3);
    // jointstate.name[0]="base_link";
    jointstate.name[0]="L1_joint";
    jointstate.name[1]="L2_joint";
    jointstate.name[2]="L3_joint";

    ros::Rate loop_rate(10);

    // tf::TransformBroadcaster broadcaster;
    // tf::Transform transform;

    tf::TransformListener listener(ros::Duration(10));
    ros::Timer timer=n.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener),boost::ref(marker_pub)));
    // ros::Subscriber setR=n.subscribe("rotation", 1, setRotation);
    // ros::Subscriber setP1 =n.subscribe("primatic1", 1, setPrimatic1);
    // ros::Subscriber setP2 = n.subscribe("primatic2", 1, setPrimatic2);
    // ros::Publisher endEffectorPostion=n.advertise<
    // int start_count=0;
    // bool start_flag=1;
    // float x=200/1000.0, y=200/1000.0, z=100/1000.0;
    // draw_start_point(0.15, 0.15, 0.2);
    // draw_line(0.020, 0.040, 0.060, 0.150, 0.200, 0.180);
    draw_circle(0.1, 0.1, 0.2, 0.1);
    // end_point();
    int time_count=0;
    while (ros::ok())
    {
        //point to start x, y, z 
        if (time_count<index_count){
            jointstate.header.stamp=ros::Time::now();
            jointstate.position[0]=theta1[time_count];
            jointstate.position[1]=d2[time_count];
            jointstate.position[2]=d3[time_count];

            x_positionPub.data=x_end;
            y_positionPub.data=y_end;
            z_positionPub.data=z_end;

            x_position.publish(x_positionPub);
            y_position.publish(y_positionPub);
            z_position.publish(z_positionPub);
        
            // publishMsg.publish(posPublish);
            // start_flag=0;
            // jointstate.position[0]=atan(-x/y)/500.0*start_count;
            // jointstate.position[1]=-0.25+(y/cos(atan(-x/y)))/500.0*start_count;
            // jointstate.position[2]=0.29-(0.29-z)/500.0*start_count;
            robotMsg.publish(jointstate);
        }
        // jointstate.header.stamp=ros::Time::now();
        // // float rad=angle*3.14/180.0;
        // //-------------Rotate-----------------//
        // jointstate.position[0]=t*9*PI/180.0;//rad;
        //  //-------------Priamis----------------//d2 luon duong//
        // jointstate.position[1]=t*5;
        // jointstate.position[1]=(jointstate.position[1]-250)/1000.0;
        // //-------------Priamis----------------//d3 luon am//
        // jointstate.position[2]=t*-10;//-50;//linear2/1000.0; 
        // jointstate.position[2]=(jointstate.position[2]+290)/1000.0;
        // robotMsg.publish(jointstate);

            // publishTransforms("L3", 100);
            // broadcaster.sendTransform(tf::StampedTransform(
            //     tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(10,20,0)),
            //     ros::Time::now(), "base_link", "L3"));
            // tf::TransformListener listener(ros::Duration(10));

            // transform.setOrigin(tf::Vector3(0,0,0));
            // transform.setRotation(tf::Quaternion(0,0,0,1));
            // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "L3"));

            // std_msgs::String msg;
            // std::stringstream ss;
            // ss<<numberOfPoints;//jointstate.position[0]*180/3.1415;
            // msg.data=ss.str();
            // ROS_INFO("Angle: %s", msg.data.c_str());
            
            ros::spinOnce();
            loop_rate.sleep();
            // ++angle;
            // ++linear1;
            // --linear2;
            // if (linear1>250) linear1=0;
            // if (linear2<-290) linear2=0;
            time_count++;

    }
    return 0;
}