#include <ros/ros.h>
#include <fstream>
//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

using namespace std;

//Random number generator
random_numbers::RandomNumberGenerator* rng;

//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

//Numeric Variables

geometry_msgs::Pose2D circleLocation;  //CNM-ST added circle location to store location
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D driftLocation;
geometry_msgs::Pose2D driftCompensation;
geometry_msgs::Pose2D goalLocation;

//GEOMETRY_MSGS
/*

geometry_msgs provides messages for common geometric primitives such as points, vectors, and poses.
These primitives are designed to provide a common data type and facilitate interoperability
throughout the system.

*/

//POSE2D

/*

This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta  <- angle

*/

int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
std_msgs::Int16 targetDetected; //ID of the detected target
bool targetsCollected[512] = { 0 }; //array of booleans indicating whether each target ID has been found

// state machine states
#define STATE_MACHINE_DRIVE	0  // DRIVE
#define STATE_MACHINE_ROTATE	1  // ROTATE LEFT
#define STATE_MACHINE_TRANSLATE	2  // FIND ANGLE

int stateMachineState = STATE_MACHINE_DRIVE;

; geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

static bool foundCircle = false;
bool rotBool = true;
bool transBool = true;
bool centerFind = false;

int circleCounter = 0;
float branchCounter = 0.1;
float searchCounter = 1.0;

bool leaving = true;
int searchCounterLoop = 0;
bool initialRun = true;
float searchDist = 0.2; //change this value to set how much larger search radius becomes on each loop. 0.5 will increase radius by 50cm
bool first = true;
bool hasTarget = false;
bool centerOffset = true;
int driftCounter = 0;
int obstacleCounter = 0;
int resetCounter = 0;
int resetCheck = 0;
int looping = 0;
int safetyCheck = 0;
bool latch = false;

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher targetCollectedPublish;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const shared_messages::TagsImage::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message);
void killSwitchTimerEventHandler(const ros::TimerEvent& event);

//CNM Function Definitions
void CNMCenterSearch();
void CNMTargetSearch();
void CNMInitial();

int main(int argc, char **argv) {

	//Returns the hostname of the network machine that the planning scene was 
	//recorded on.  (string)

	// CALL FUNCTION TO RETRIEVE HOST NAME
	gethostname(host, sizeof(host));
	// STORE HOST NAME FROM CHAR ARRAY TO STD STRING FORMAT
	string hostname(host);

	rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator

//  M_PI located in MATH.H FILE.  Contains PI to 3.14159265358979323846

//  .theta references the theta float in goallocation

//  Generate a random real within given bounds: [lower_bound, upper_bound)

//  Find a random angle to search for the goals location

	targetDetected.data = -1; //initialize target detected


//  ???????????????????????????????????????
	if (argc >= 2) {
		publishedName = argv[1];
		cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
	}
	else {
		publishedName = hostname;
		cout << "No Name Selected. Default is: " << publishedName << endl;
	}

	// NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
	ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
	ros::NodeHandle mNH;

	signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

	joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
	modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
	targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
	obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
	odometrySubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, odometryHandler);
	targetsCollectedSubscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);

	status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
	velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
	stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
	targetCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
	targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetPickUpImage"), 1, true);
	targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetDropOffImage"), 1, true);

	publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
	killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
	stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);

	ros::spin();

	return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {
	std_msgs::String stateMachineMsg;
	/*
	state machine states
	#define STATE_MACHINE_DRIVE     0  // DRIVE
	#define STATE_MACHINE_ROTATE    1  // ROTATE
	#define STATE_MACHINE_TRANSLATE 2  // FIND ANGLE
	*/

	//DRIVING LOOP
	if (currentMode == 2 || currentMode == 3) //Robot is in automode
	{

		//if first loop through mobility
		if (first == true)
		{

			CNMInitial();

		}

		if (centerOffset == true && first == false && rotBool == true)
		{
			circleLocation.x = currentLocation.x;
			circleLocation.y = currentLocation.y;
			centerOffset = false;

		}

		switch (stateMachineState)
		{

			//Search for tag 256, if not found perform search pattern
		case STATE_MACHINE_DRIVE:
			//--------------------------------------------------------------------------------------------------------------------------------------------

			//if CENTER has not been found rotate in a 360
			if (foundCircle == false)
			{

				CNMCenterSearch();

				//If angle between current and goal is significant
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1)
				{

					stateMachineState = STATE_MACHINE_ROTATE; //rotate

				}

				//If goal has not yet been reached
				//changed from pi/2 to 10 degrees
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < (10 * (M_PI / 180)))
				{

					stateMachineState = STATE_MACHINE_TRANSLATE; //translate

				}

			}
			//--------------------------------------------------------------------------------------------------------------------------------------------

			//if center HAS been found
			else if (foundCircle == true)
			{

				//If returning with a target
				if (targetDetected.data != -1)
				{
					//If goal has not yet been reached
					if (hypot((circleLocation.x - currentLocation.x), (circleLocation.y - currentLocation.y)) > 0.125)
					{
						if (latch == false)
						{

							//set angle to center as goal heading
							//where you want to go - where you are
							goalLocation.theta = atan2((circleLocation.y - currentLocation.y), (circleLocation.x - currentLocation.x));
							stateMachineState = STATE_MACHINE_ROTATE;

							//set center as goal position
							goalLocation.x = circleLocation.x;
							goalLocation.y = circleLocation.y;
						}

					}

					else
					{

						foundCircle = false;
						branchCounter = 0.1;
					}


				}

				//If NOT returning with a target
				else (targetDetected.data == -1)
				{
				
					CNMTargetSearch();
			
				}

				//If angle between current and goal is significant
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1)
				{

					stateMachineState = STATE_MACHINE_ROTATE; //rotate

				}

				//If goal has not yet been reached
				//changed from pi/2 to 10 degrees
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < (10 * (M_PI / 180)))
				{

					stateMachineState = STATE_MACHINE_TRANSLATE; //translate

				}

			}


		case STATE_MACHINE_ROTATE:

			statemachineMsg.data = "ROTATING";

			if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1)
			{
				setVelocity(0.0, 0.2); //rotate left
			}

			else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1)
			{
				setVelocity(0.0, -0.2); //rotate right
			}

			else
			{
				setVelocity(0.0, 0.0); //stop
				stateMachineState = STATE_MACHINE_TRANSLATE; //move to translate step

			}
			break;

			//Calculate angle between currentLocation.x/y and goalLocation.x/y
			//Drive forward
			//Stay in this state until angle is at least PI/2

		case STATE_MACHINE_TRANSLATE:

			stateMachineMsg.data = "TRANSLATING";

			//changed from pi/2 to 10 degrees
			if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI / 2)
			{
				setVelocity(0.2, 0.0);
			}

			else
			{
				rotBool = true;
				setVelocity(0.0, 0.0); //stop
				stateMachineState = STATE_MACHINE_DRIVE; //move back to transform step

			}
			break;

		default:

			break;

		}

	}

	// mode is NOT auto
	else
	{

		// publish current state for the operator to see
		stateMachineMsg.data = "WAITING";
	}

	// publish state machine string for user, only if it has changed, though
	if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0)
	{

		stateMachinePublish.publish(stateMachineMsg);
		sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());

	}


}

void setVelocity(double linearVel, double angularVel)
{
	// Stopping and starting the timer causes it to start counting from 0 again.
	// As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
	// the rover's kill switch wont be called.
	//killSwitchTimer.stop();
	//killSwitchTimer.start();

	velocity.linear.x = linearVel * 1.5;
	velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
	velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const shared_messages::TagsImage::ConstPtr& message)
{

	/*if(targetDetected.data != -1 && foundCircle == false) //we shouldnt need this with the restructured code
	{targetDetected.data = -1;
	hasTarget = false;}*/

	//if this is the goal target
	if (message->tags.data[0] == 256)
	{
		if (foundCircle == false && hasTarget == false)
		{
			circleLocation.x = currentLocation.x + (0.1 * cos(currentLocation.theta));
			circleLocation.y = currentLocation.y + (0.1 * sin(currentLocation.theta));
			foundCircle = true;
			setVelocity(0.0, 0.0);
			stateMachineState = STATE_MACHINE_DRIVE;
			rotBool = true;
			targetDetected.data = -1;
		}

		if (centerFind == false)
		{
			centerFind = true;
			driftLocation = circleLocation;
		}

		//update centers location
		circleLocation.x = currentLocation.x + (0.1 * cos(currentLocation.theta));
		circleLocation.y = currentLocation.y + (0.1 * sin(currentLocation.theta));



		//if we were returning with a target
		if (targetDetected.data != -1)
		{

			//publish to scoring code
			if (targetDetected.data != -1 && message->tags.data[0] == 256)
			{
				rotBool = true;
				targetDropOffPublish.publish(message->image);
				targetDetected.data = -1;
				hasTarget = false;
				looping = 0;
				safetyCheck = 0;
				if (leaving == true) //A check that will cause the robot to research an area incase of clusters
				{
					leaving = false;
					if (searchCounterLoop == 0)
					{
						searchCounterLoop = 7;
						if (searchCounter > 1)
						{
							searchCounter = searchCounter - searchDist;
						}
						else
						{
							searchCounterLoop = 0;
						}
					}
					else if (searchCounterLoop < 8) { searchCounterLoop = searchCounterLoop - 2; }
				}

				if (centerFind == true)
				{
					if (hypot((driftLocation.x - circleLocation.x), (driftLocation.y - circleLocation.y)) > 0.1)
					{
						if (driftCounter > 0)
						{
							driftCompensation.x = (circleLocation.x - driftLocation.x) / driftCounter;
							driftCompensation.y = (circleLocation.y - circleLocation.y) / driftCounter;
							driftCounter = 0;
						}
					}
					driftLocation.x = circleLocation.x;
					driftLocation.y = circleLocation.y;
				}
			}
		}
	}


	//if target has not previously been detected
	else if (targetDetected.data == -1)
	{
		targetDetected.data = message->tags.data[0];

		//check if target has not yet been collected
		if (!targetsCollected[targetDetected.data])
		{
			hasTarget = true;
			//set angle to center as goal heading
			goalLocation.theta = atan2(circleLocation.y - currentLocation.y, circleLocation.x - currentLocation.x);

			//set center as goal position
			//ORIGINALLY set to 0,0  NOT RIGHT
			goalLocation.x = circleLocation.x;
			goalLocation.y = circleLocation.y;

			//publish detected target
			targetCollectedPublish.publish(targetDetected);

			//publish to scoring code
			targetPickUpPublish.publish(message->image);

			//Rotate
			stateMachineState = STATE_MACHINE_ROTATE;

		}
		else if (hasTarget == false)
		{
			targetDetected.data = -1;
		}
	}
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
	setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
	if (message->data > 0)
	{
		//obstacle on any side
		if (message->data == 1 || message->data == 2 || message->data == 3)
		{
			safetyCheck++;
			obstacleCounter++;
			if (rotBool == true && obstacleCounter > 60)
			{
				obstacleCounter = 0;
			}
			if (obstacleCounter > 60)
			{
				//select new heading 0.4 radians to the left
				goalLocation.theta = currentLocation.theta + 0.4;
				rotBool = false;
				//select new position 30 cm from current location
				goalLocation.x = currentLocation.x + (0.3 * cos(goalLocation.theta));
				goalLocation.y = currentLocation.y + (0.3 * sin(goalLocation.theta));
			}
			else
			{
				latch = true;
				rotBool = false;
				setVelocity(0.0, 0.0);
			}
			if (safetyCheck > 2400)
			{
				searchCounter = 0.8;
			}
		}

		//switch to transform state to trigger collision avoidance
		stateMachineState = STATE_MACHINE_DRIVE;
	}
	else
	{
		obstacleCounter = 0;
		rotBool = true;
		latch = false;
	}

}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	//Get (x,y) location directly from pose
	currentLocation.x = message->pose.pose.position.x;
	currentLocation.y = message->pose.pose.position.y;

	//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	currentLocation.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
	if (currentMode == 0 || currentMode == 1)
	{
		setVelocity(message->linear.x, message->angular.z);
	}
}

void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
	std_msgs::String msg;
	msg.data = "CNM-SwarmTeam Online";
	status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
	// No movement commands for killSwitchTime seconds so stop the rover 
	setVelocity(0, 0);
	double current_time = ros::Time::now().toSec();
	ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message) {
	targetsCollected[message->data] = 1;
}

void sigintEventHandler(int sig)
{
	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

void CNMCenterSearch()
{

	if (circleCounter < 0)
	{
		circleCounter = 0; rotBool = true;
	}

	if (circleCounter == 0 && rotBool == true) //octigonal search pattern
	{
		circleCounter++;
		rotBool = false;
		goalLocation.x = circleLocation.x + branchCounter; //sets drive location to the east branch ammount
		goalLocation.y = circleLocation.y - branchCounter / 2; //and south half that ammount
		goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	}
	else if (circleCounter == 1 && rotBool == true)
	{
		circleCounter++;
		rotBool = false;
		goalLocation.x = circleLocation.x + branchCounter; //sets drive location to the east branch ammount
		goalLocation.y = circleLocation.y + branchCounter / 2; //and north half that ammount
		goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	}
	else if (circleCounter == 2 && rotBool == true) //by now we have driven north branch ammount in a location branch east
	{
		circleCounter++;
		rotBool = false;
		goalLocation.x = circleLocation.x + branchCounter / 2; //drive northwest
		goalLocation.y = circleLocation.y + branchCounter;
		goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	}
	else if (circleCounter == 3 && rotBool == true)
	{
		circleCounter++;
		rotBool = false;
		goalLocation.x = circleLocation.x - branchCounter / 2; //drive west
		goalLocation.y = circleLocation.y + branchCounter;
		goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	}
	else if (circleCounter == 4 && rotBool == true)
	{
		circleCounter++;
		rotBool = false;
		goalLocation.x = circleLocation.x - branchCounter; //drive south west
		goalLocation.y = circleLocation.y + branchCounter / 2;
		goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	}
	else if (circleCounter == 5 && rotBool == true)
	{
		circleCounter++;
		rotBool = false;
		goalLocation.x = circleLocation.x - branchCounter; //drive south
		goalLocation.y = circleLocation.y - branchCounter / 2;
		goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	}
	else if (circleCounter == 6 && rotBool == true)
	{
		circleCounter++;
		rotBool = false;
		goalLocation.x = circleLocation.x - branchCounter / 2; //drive south east
		goalLocation.y = circleLocation.y - branchCounter;
		goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	}
	else if (circleCounter == 7 && rotBool == true)
	{
		circleCounter++;
		rotBool = false;
		goalLocation.x = circleLocation.x + branchCounter / 2; //drive east
		goalLocation.y = circleLocation.y - branchCounter;
		goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	}
	/*else if(circleCounter == 8 && rotBool == true)
	{
	circleCounter++;
	rotBool = false;
	goalLocation.x = circleLocation.x + branchCounter; //drive north east back to where we started
	goalLocation.y = circleLocation.y - branchCounter/2;
	goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
	straight east movement before resuming octigonal pattern
	}*/
	if (circleCounter >= 8 && rotBool == true)
	{
		circleCounter = 0;
		branchCounter = branchCounter + searchDist * 2; //select a new radius to search this will cause
														//driftCounter++; 
		circleLocation.x = circleLocation.x; +driftCompensation.x;
		circleLocation.y = circleLocation.y; +driftCompensation.y;
	}


}

void CNMTargetSearch()
{

	//if no target found but center has been found
	else if (targetDetected.data == -1)
	{
		leaving = true; //bool used after a target has been returned to cause reasearching of some area in order to handle clusters better		
		if (searchCounterLoop < 0)
		{
			searchCounterLoop = 0; rotBool = true;
		}
		if (searchCounterLoop == 0 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x + searchCounter;
			goalLocation.y = circleLocation.y - searchCounter / 2;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		else if (searchCounterLoop == 8 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x + searchCounter;
			goalLocation.y = circleLocation.y + searchCounter / 2;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		else if (searchCounterLoop == 7 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x + searchCounter / 2;
			goalLocation.y = circleLocation.y + searchCounter;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		else if (searchCounterLoop == 6 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x - searchCounter / 2;
			goalLocation.y = circleLocation.y + searchCounter;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		else if (searchCounterLoop == 5 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x - searchCounter;
			goalLocation.y = circleLocation.y + searchCounter / 2;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		else if (searchCounterLoop == 4 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x - searchCounter;
			goalLocation.y = circleLocation.y - searchCounter / 2;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		else if (searchCounterLoop == 3 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x - searchCounter / 2;
			goalLocation.y = circleLocation.y - searchCounter;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		else if (searchCounterLoop == 2 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x + searchCounter / 2;
			goalLocation.y = circleLocation.y - searchCounter;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		else if (searchCounterLoop == 1 && rotBool == true)
		{
			searchCounterLoop++;
			rotBool = false;
			goalLocation.x = circleLocation.x + searchCounter;
			goalLocation.y = circleLocation.y - searchCounter / 2;
			goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
			looping++;
		}
		if (searchCounterLoop >= 9 && rotBool == true)
		{
			searchCounter = searchCounter + searchDist;
			searchCounterLoop = 0;
			circleLocation.y = circleLocation.y; +driftCompensation.y;
			circleLocation.x = circleLocation.x; +driftCompensation.x;
		}
		if (looping > 7)
		{
			looping = 0;
			driftCounter++;
		}

	}//end else if for no targets found


}

void CNMInitial()
{

	first = false;
	rotBool = false;
	goalLocation.theta = currentLocation.theta;

	//drive forward 50cm
	goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
	goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
	circleLocation.x = 0;
	circleLocation.y = 0;
	driftCompensation.x = 0;
	driftCompensation.y = 0;

	if (currentLocation.theta <= 45 * M_PI / 180)
	{
		searchCounterLoop = 3;
	}
	else if (currentLocation.theta <= 90 * M_PI / 180)
	{
		searchCounterLoop = 2;
	}
	else if (currentLocation.theta <= 135 * M_PI / 180)
	{
		searchCounterLoop = 1;
	}
	else if (currentLocation.theta <= 180 * M_PI / 180)
	{
		searchCounterLoop = 8;
	}
	else if (currentLocation.theta <= 225 * M_PI / 180)
	{
		searchCounterLoop = 7;
	}
	else if (currentLocation.theta <= 270 * M_PI / 180)
	{
		searchCounterLoop = 6;
	}
	else if (currentLocation.theta <= 310 * M_PI / 180)
	{
		searchCounterLoop = 5;
	}
	else if (currentLocation.theta <= 360 * M_PI / 180)
	{
		searchCounterLoop = 4;
	}

}