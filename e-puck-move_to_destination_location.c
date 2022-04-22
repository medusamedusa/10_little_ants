//E-puck (PROTO)E-puck "e-puck3"E-puck "e-puck3"E-puck "e-puck3"

/*
 * Copyright 2021 Albert Alfrianta
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * 
 * Created on: 2021-08, Bogor, Indonesia
 * 
 * Contact: albert.alfrianta@gmail.com or https://www.linkedin.com/in/albert-alfrianta/
 * 
 * Description:
 * 	Please read the header file for the method explanations.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h> // new

#include "e-puck-move_to_destination_location.h"
#include "motor_controller.h"
#include "positioning_controller.h"
#include "cartesian.h"
#include "robot_controller.h" // new
#include <stdbool.h> // new
#include <webots/distance_sensor.h> // new

#include <webots/robot.h>

// tangensial/linear speed in m/s. 
// Tangensial speed = angular speed * wheel radius 
// Tangensial speed = 6.28 rad * 2.05 cm = 0.12874 m/s
#define TANGENTIAL_SPEED 0.12874

// Speed of robot to spinning in place (in cycles per second)
// 1 cycle = 360 degrees. 
// Robot rotational speed = tangensial speed / (phi * axle length) 
// note: axle length is distance between wheels
// Robot rotational speed = 0.12874 / (phi*0.052) = 0.787744755
#define ROBOT_ROTATIONAL_SPEED 0.772881647

// Speed of robot to spinning in place (in degrees per second)
// Robot angular speed in degrees = robot rotational speed * 360 
// Robot angular speed in degrees = 0.787744755*360 = 283.588111888 
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 278.237392796



// new
#define NUMBER_OF_DISTANCE_SENSORS 8
static WbDeviceTag distance_sensors[NUMBER_OF_DISTANCE_SENSORS];
static const char *distance_sensors_names[NUMBER_OF_DISTANCE_SENSORS] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
#define SENSOR_VALUE_DETECTION_THRESHOLD 140




int time_step;



/*
Method to get simulator time step.
Webots have simulator time step. 
The basic time step is the time step increment used by Webots to advance the virtual time and perform physics simulation.
*/
int getTimeStep()
{
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;

    // robot_controller_init(time_step); // ADDED TODAY from the other code
}





// ADDED TODAY from the other code:

static void init_robot() {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  /* get simulator time step */
	time_step = (int)wb_robot_get_basic_time_step();
	
	/* init the controller */
	robot_controller_init(time_step);	
}

// new:

bool * get_sensors_condition()
{
	static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = {false};
	
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
		/*
		 * Obstacle detected condition is true if the sensor values is larger then the threshold value
		 * */
		if (wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_VALUE_DETECTION_THRESHOLD) {
			sensors_condition[i] = true;
		} else {
			sensors_condition[i] = false;
		}
	}
	
	return sensors_condition;
}






/*
This command is to perform simulation steps. 
This needed for the controller time step. 
The controller time step is the time increment of time executed at each iteration of the control loop of a controller. 
We must call this to synchronize our program and the simulator condition. 
It will return -1 if the simulation is stopped. 
If we not call this command, the robot will do nothing. 
For example the wb_motor_set_velocity(left_motor, MAX_SPEED) only set the motor speed value. 
So we need to call and looping the wb_robot_step(time_step) command to make the robot move.
*/
void step()
{
    if (wb_robot_step(time_step) == -1)
    {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

static void init()
{
	time_step = getTimeStep();
	
	motorControllerInit(time_step);
	
	positioningControllerInit(time_step);
	
    step();
}

void rotateHeading(const double thetaDot)
{
	// if thetaDot is zero
	if (!cartesianIsThetaEqual(thetaDot, 0))
	{
		// the duration required for the robot to rotate the body by the specified thetaDot
		double duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
		printf("duration to face the destination: %.4f\n", duration);

		// if thetaDot > 0, robot will rotate to left
		if (thetaDot > 0)
		{
			// set robot motor to rotate left
			motorRotateLeft();
		}
		// if thetaDot < 0, robot will rotate to right
		else if (thetaDot < 0)
		{
			// set robot motor to rotate right
			motorRotateRight();
		}

		// run the simulator
		double start_time = wb_robot_get_time();
		do
		{
			step();
		}
		while (wb_robot_get_time() < start_time + duration);
	}
}

void moveForward(double distance)
{
	// the duration required for the robot to move by the specified distance
	double duration = distance / TANGENTIAL_SPEED;
	printf("duration to reach target location: %.4f\n", duration);

	// set robot motor to move forward
	motorMoveForward();

	// run the simulator
	double start_time = wb_robot_get_time();
	do
	{
		step();	
	}
	while (wb_robot_get_time() < start_time + duration);
	
	// stop the motor
    motorStop();
	step();
}

void moveToDestination(const double destinationCoordinate[2])
{
	double * currentCoordinate = positioningControllerGetRobotCoordinate();
	printf("Initial Coordinate: %-0.3f %.0f\n", currentCoordinate[0], currentCoordinate[1]);

	printf("Destination Coordinate: %-0.3f %.0f\n", destinationCoordinate[0], destinationCoordinate[1]);
	
	// if the robot is already at the destination location
	if (cartesianIsCoordinateEqual(positioningControllerGetRobotCoordinate(), destinationCoordinate))
	{
		printf("Robot is already at the destination location\n");
		return;
	}

	// thetaDot is the degree of rotation needed by the robot to face the destination
	// thetaDot is zero if robot is facing the destination
	double thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate);
	printf("thetaDotToDestination: %.0f\n", thetaDotToDestination);

	rotateHeading(thetaDotToDestination);

	// the distance needed for the robot to reach its destination
	double distanceToDestination = positioningControllerCalcDistanceToDestination(destinationCoordinate);
	printf("distanceToDestination: %.0f\n", distanceToDestination);
	
	moveForward(distanceToDestination);

	currentCoordinate = positioningControllerGetRobotCoordinate();
	printf("Stop Coordinate: %-0.3f %.0f\n", currentCoordinate[0], currentCoordinate[1]);
}


int main(int argc, char **argv)
{

	wb_robot_init();

	init();

	
    const double destinationCoordinate[20][2] = {{0.05, -0.24}, {-0.21, 0.02}, {0.46, -0.09}, 
    {-0.32, -0.39}, {-0.13, -0.46}, {0.24, -0.15}, {-0.18, -0.49}, {0.42, -0.47}, {-0.18, -0.07}, {-0.12, -0.16},
	{0.45, -0.09}, {0.46, -08.}, {0.46, -0.09}, {0.46, -0.08}, {0.45, -0.09}, {0.45, -0.09}, {0.45, -0.09},
	{0.45, -0.08}, {0.46, -0.09}, {0.45, -0.09}}; // , {0.46, -0.09}

    const char *wb_robot_get_name();

    //bool *is_sensors_active = get_sensors_condition();
    bool *get_sensors_condition();
    //bool *is_sensors_active = get_sensors_condition();

    //bool *is_sensors_active = get_sensors_condition();

    /*

    if (strcmp(wb_robot_get_name(), "e-puck1") == 0 || strcmp(wb_robot_get_name(), "e-puck2") == 0
    		|| strcmp(wb_robot_get_name(), "e-puck3") == 0 || strcmp(wb_robot_get_name(), "e-puck4") == 0
    		|| strcmp(wb_robot_get_name(), "e-puck5") == 0 || strcmp(wb_robot_get_name(), "e-puck6") == 0
    		|| strcmp(wb_robot_get_name(), "e-puck7") == 0 || strcmp(wb_robot_get_name(), "e-puck8") == 0
    		|| strcmp(wb_robot_get_name(), "e-puck9") == 0 || strcmp(wb_robot_get_name(), "e-puck10") == 0) {		
		//print_sensor_values();
		
		bool *is_sensors_active = get_sensors_condition();
		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft();
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight();
		}
  }
  */
    

   	if (strcmp(wb_robot_get_name(), "e-puck1") == 0) {
   		moveToDestination(destinationCoordinate[0]);
   		
   		bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

   		moveToDestination(destinationCoordinate[10]);

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}
   		
   		moveToDestination(destinationCoordinate[10]); 
   		 
   	} // else if...
   	if (strcmp(wb_robot_get_name(), "e-puck2") == 0) {
   		moveToDestination(destinationCoordinate[1]);

   		bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

   		moveToDestination(destinationCoordinate[11]); 

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

		moveToDestination(destinationCoordinate[11]); 
   		
   		
   	}
   	if (strcmp(wb_robot_get_name(), "e-puck3") == 0) {
    	moveToDestination(destinationCoordinate[2]);

    	bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

    	moveToDestination(destinationCoordinate[12]); 
		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

		moveToDestination(destinationCoordinate[12]); 

    	 
    }
    if (strcmp(wb_robot_get_name(), "e-puck4") == 0) {
    	moveToDestination(destinationCoordinate[3]);

    	bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}
    	

    	moveToDestination(destinationCoordinate[13]); 

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

		moveToDestination(destinationCoordinate[13]); 

		 
    }
    if (strcmp(wb_robot_get_name(), "e-puck5") == 0) {
    	moveToDestination(destinationCoordinate[4]);

    	bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

    	moveToDestination(destinationCoordinate[14]); 

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}
   		
   		moveToDestination(destinationCoordinate[14]); 
    	 
    }
    if (strcmp(wb_robot_get_name(), "e-puck6") == 0) {
    	moveToDestination(destinationCoordinate[5]);

    	bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}


    	moveToDestination(destinationCoordinate[15]); 

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

		moveToDestination(destinationCoordinate[15]); 

    	 
    }
    if (strcmp(wb_robot_get_name(), "e-puck7") == 0) {
    	moveToDestination(destinationCoordinate[6]);

    	bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

    	moveToDestination(destinationCoordinate[16]); 

 
		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}
   		
   		moveToDestination(destinationCoordinate[16]); 
    	 
    }
    if (strcmp(wb_robot_get_name(), "e-puck8") == 0) {
    	moveToDestination(destinationCoordinate[7]);

    	bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}


    	moveToDestination(destinationCoordinate[17]); 
    	
    	
    	
		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

		moveToDestination(destinationCoordinate[17]); 

    	 
    }
    if (strcmp(wb_robot_get_name(), "e-puck9") == 0) {
    	moveToDestination(destinationCoordinate[8]);

    	bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

    	moveToDestination(destinationCoordinate[18]); 

    	
		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}
   		
   		moveToDestination(destinationCoordinate[18]); 
    	 
    }
    if (strcmp(wb_robot_get_name(), "e-puck10") == 0) {
    	moveToDestination(destinationCoordinate[9]);

    	bool *is_sensors_active = get_sensors_condition();

		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

    	moveToDestination(destinationCoordinate[19]); 
		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motorRotateLeft(180);
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motorRotateRight(180);
		}

		moveToDestination(destinationCoordinate[19]); 

    	 
    }
    //else {
   	//	for (int i = 0; i < 21; i++) {
    //	moveToDestination(destinationCoordinate[i]);  }
    //}

    
	wb_robot_cleanup();
    return EXIT_SUCCESS;
}