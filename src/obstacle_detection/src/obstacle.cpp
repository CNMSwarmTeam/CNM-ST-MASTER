#include <ros/ros.h>

//ROS libraries
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

//ROS messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>

using namespace std;

//Globals
double collisionDistance = 0.3; //meters the ultrasonic detectors will flag obstacles
string publishedName;
char host[128];

//Publishers are gets
ros::Publisher obstaclePublish;

//Subscriber
//ros::Subscriber velocitySubscriber;

//Callback handlers function proto
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);
//don't need collision distance
//void setCollisionDistance();

int main(int argc, char** argv) {
    gethostname(host, sizeof (host));
    string hostname(host);
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Obstacle module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));
    ros::NodeHandle oNH;
    
    //Created velocity subscriber to get speed from mobility
    //velocitySubscriber is the speed of swarmie
    //velocitySubscriber = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);

    obstaclePublish = oNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);
    
    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(oNH, (publishedName + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(oNH, (publishedName + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(oNH, (publishedName + "/sonarRight"), 10);	
    //Time Sync??
    message_filters::TimeSynchronizer<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSync(sonarLeftSubscriber, sonarCenterSubscriber,      sonarRightSubscriber, 10);
    //SonarSync??
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));
    //Spin waits till all processes are done
    ros::spin();
    return EXIT_SUCCESS;
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
    std_msgs::UInt8 obstacleMode;

    bool collisionLeft, collisionRight, collisionCenter;

    //set flags when collision detected

    if(sonarLeft->range < collisionDistance){
        collisionLeft = true;
    }
    if (sonarRight->range < collisionDistance){
        collisionRight = true;
    }
    if (sonarCenter->range < collisionDistance){
        collisionCenter = true;
    }
    if (sonarLeft->range > collisionDistance){
        collisionLeft = false;
    }
    if (sonarCenter->range > collisionDistance){
        collisionCenter = false;
    }
    if (sonarRight->range > collisionDistance){
        collisionRight = false;
    }
    
    if (!collisionLeft && !collisionRight && !collisionCenter){
        obstacleMode.data = 0;//No Collision 
    }
    else if(collisionLeft)//If obstacle is to the left
    {
        obstacleMode.data = 1;
    }
    else if(collisionRight){//If obstacle is to the right
        obstacleMode.data = 2; 
    }
    else if(collisionCenter){//If target is in the center 
        obstacleMode.data = 3;
    }

    //else if(collisionRight && collisionCenter){//If obstacle is to the right and center
    //    obstacleMode.data = 4;
    //}

    //else if(collisionLeft && collisionCenter){//If obstacle is to the left and center
    //    obstacleMode.data = 5;
    //}

    //else if (collisionLeft && collisionRight && collisionCenter){
      //  obstacleMode.data = 6;//all sonar detects
   // }
    
    obstaclePublish.publish(obstacleMode);
    //collisionLeft = collisionRight = collisionCenter = false;//reset collision
}

