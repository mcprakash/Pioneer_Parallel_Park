/*

This is the source code for the Parallel Parking Robot developed using Aria library
and Pioneer robot in requirement for partial fullfillment of the course
CSCI 5551 at the University of Minnesota

Copyright (C) 2014  Chandra Mangipudi, Nicholas Saari, George Brown and Nathan Ische-Knobloauch

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

*/


#include "Aria.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <vlc/vlc.h>
using namespace std;

//<------------- Pointers for Sensor readings-----------------> 
std::list<ArSensorReading *> *readings;
std::list<ArSensorReading *>::iterator it;


//<------------- Pointer for the Sick Laser Scanner-----------------> 
ArSick* mySick;

//<------------- Log files for storing experimental data and debugging-----------------> 
std::ofstream myfile;
std::ofstream myfile2;
std::ofstream myfile3;
std::ofstream myfile4;

//<------------- Constant Parameters used for Hough Transform-----------------> 
const int NUM_READINGS = 90;
const int PHI = 18;
const int PHI_MAX = 180;
const int RHO_MAX = 3000;
const int RHO_INT = 25;
int alpha = 0;
std::list<int> nineties;
std::list<int> zeros;
std::list<int>::iterator itn;
std::list<int>::iterator itz;

//<------------- Flags and Variables used to detect parking spots and current position-----------------> 
bool isParkSpace = false, seenCar = false, carPassed = false, started = false, carNearPassed = false;
int nextCarDist = -1, wallDist, carDist;
double x, y, th, sums, tol = 30;

//<----------------- Get the lines corresponding to 0 deg and 90 deg to determine parking spot---------------------> 
void getnew(){

	//<------------- Get Raw data from Line Scanner-----------------> 

	mySick->lockDevice();
	readings = (list<ArSensorReading *, allocator<ArSensorReading *> > *)mySick->getRawReadings();
	double points[NUM_READINGS];
	int lines = 10;
	int i = 0;
	if (NULL != readings) {
		if ((readings->end() != readings->begin())) {
			for (it = readings->begin(); it != readings->end() && i < NUM_READINGS; it++) {
				myfile3 << (*it)->getRange() << ",";
				points[i] = (*it)->getRange();
				i = i + 1;
			}
			myfile3 << std::endl;
		}
		else {
			//myfile << "(readings->end() == readings->begin())" << std::endl;
		}
	}
	else {
		//myfile << "NULL == readings" << std::endl;
	}
	mySick->unlockDevice();

	//<---------------------------------- Perform Hough Line Transform--------------------------------------------> 

	int rhoI;
	double phi, rho, theta;
	int accu[2 * (RHO_MAX / RHO_INT)][PHI] = { 0 };
	int j = 0;

	//<------------- Create Accumulator Array-----------------> 
	for (i = 0; i < NUM_READINGS; i++){
		for (j = 0; j < PHI; j++){
			phi = (j * (PHI_MAX / PHI) * M_PI) / 180;
			theta = ((i)* M_PI) / 180;
			rho = points[i] * cos(theta - phi);
			rho = ((round(rho / RHO_INT) * RHO_INT) + RHO_MAX) / RHO_INT;
			if (rho < 2 * (RHO_MAX / RHO_INT)) {
				rhoI = int(rho);
				accu[rhoI][j] = accu[rhoI][j] + 1;
			}
		}
	}

	//<------------- Threshold the accumulator-----------------> 
	int k = 0;
	for (k = 1; k <= lines; k++){
		int mx[3] = { 0, 0, 0 };
		for (i = 0; i < 2 * (RHO_MAX / RHO_INT); i++){
			for (j = 0; j < PHI; j++){
				if (accu[i][j] > mx[0]){
					mx[0] = accu[i][j];
					mx[1] = i;
					mx[2] = j;
					accu[i][j] = 0;
				}
			}
		}

		//<------------- Detect 0 deg and 90 deg lines-----------------> 

		int phiI = mx[2] * (PHI_MAX / PHI);
		int rhoI = (mx[1] * RHO_INT) - RHO_MAX;
		myfile2 << rhoI << "," << phiI << std::endl;
		if (phiI == 90 || phiI == 80 || phiI == 100 || phiI == 70 || phiI == 110){
			if (!seenCar){
				seenCar = true;
				carDist = rhoI;
			}
			nineties.push_back(rhoI);
			//printf("line has %d points and equation %d = rcos(THETA - %d)\n" ,
			//mx[0], rhoI , phiI) ;
			mx[0] = 0; mx[1] = 0; mx[2] = 0;
		}
		if (phiI == 0){
			zeros.push_back(rhoI);
			//printf("line has %d points and equation %d = rcos(THETA - %d)\n" ,
			//mx[0], rhoI , phiI) ;
			mx[0] = 0; mx[1] = 0; mx[2] = 0;
		}

	}
	myfile2 << std::endl;
	nineties.sort();
	zeros.sort();

	//<------------- Debug by printing out all the detected lines-----------------> 
	//for (itn = nineties.begin(); itn != nineties.end(); itn++) {
	//	//std::cout << *itn;
	//   }
	//   std::cout << std::endl;
	//   for (itz = zeros.begin(); itz != zeros.end(); itz++) {
	//       //std::cout << *itz;
	//   }
	//   std::cout << std::endl;


	//<------------- Estimate Position based on detected lines-----------------> 
	sums = 0;
	if (zeros.size() > 0)
	{
		wallDist = zeros.back();
	}
	//else {wallDist = 0;}
	for (int i = 0; i < 50; i++){
		x = points[i];
		th = (M_PI / 180) * i;
		y = wallDist / cos(th);
		sums = sums + fabs(x - y);
	}

	//<------------- Check for a parking spot-----------------> 
	myfile2 << "sums :" << sums << std::endl;
	nextCarDist = -1;
	if (nineties.size() > 0){
		if (nineties.front() >= 1250 && sums < (50 * tol) && wallDist > 600 && seenCar){
			//isParkSpace = true;
			if (carPassed and nextCarDist == -1){
				nextCarDist = nineties.front();
				isParkSpace = true;
			}
			myfile << "Found Spot! car dist:" << carDist << "next car dist:" << nextCarDist << endl;
			cout << nextCarDist;
		}
	}

	nineties.clear();
	zeros.clear();
}

int main(int argc, char **argv)
{

	//<------------- Store data in files for debugging-----------------> 

	myfile.open("logs.txt");
	myfile2.open("lines.txt");
	myfile3.open("points.txt");
	myfile4.open("time.txt");


	std::string filename = "1scans.2d";


	//<------------- Put music for different segments of the robot-----------------> 
	libvlc_instance_t *inst;
	libvlc_media_player_t *mp;
	libvlc_media_t *m;
	libvlc_instance_t *inst2;
	libvlc_media_player_t *mp2;
	libvlc_media_t *m2;
	libvlc_instance_t *inst3;
	libvlc_media_player_t *mp3;
	libvlc_media_t *m3;
	inst = libvlc_new(0, NULL, NULL);
	m = libvlc_media_new(inst, "musak.mp3", NULL);
	mp = libvlc_media_player_new_from_media(m, NULL);
	libvlc_media_release(m);
	libvlc_media_player_play(mp, NULL);

	inst2 = libvlc_new(0, NULL, NULL);
	m2 = libvlc_media_new(inst, "beep.wav", NULL);
	mp2 = libvlc_media_player_new_from_media(m2, NULL);
	libvlc_media_release(m2);

	inst3 = libvlc_new(0, NULL, NULL);
	m3 = libvlc_media_new(inst3, "happy.mp3", NULL);
	mp3 = libvlc_media_player_new_from_media(m3, NULL);
	libvlc_media_release(m3);

	//<------------- Robot classes-----------------> 
	int ret;
	// robot
	ArRobot robot;
	// the laser
	ArSick sick;
	ArSerialConnection laserCon;

	ArSerialConnection con;
	//int ret;
	std::string str;
	const int DELHEAD = 4;

	//<------------- Timers for calculating total run time-----------------> 

	struct timeval start;
	struct timeval end;
	struct timeval total;

	//<------------- Sensors and Controls Classes-----------------> 

	// sonar, must be added to the robot
	ArSonarDevice sonar;
	// the actions we'll use to wander
	// recover from stalls
	ArActionStallRecover recover;
	// react to bumpers
	ArActionBumpers bumpers;
	// limiter for close obstacles
	ArActionLimiterForwards limiter("speed limiter near", 100, 300, 100, 1.1);
	// limiter for far away obstacles
	ArActionLimiterForwards limiterFar("speed limiter far", 300, 600, 175, 1.1);
	// limiter for the table sensors
	ArActionLimiterTableSensor tableLimiter;
	// actually move the robot
	ArActionConstantVelocity constantVelocity("Constant Velocity", 250);
	// turn the robot if its slowed down
	ArActionTurn turn;

	//<------------- Initialize the Aria library-----------------> 
	Aria::init();

	//<------------- Connect to robot-----------------> 
	ArSimpleConnector connector(&argc, argv);
	connector.parseArgs();

	if (argc > 1)
	{
		connector.logOptions();
		exit(1);
	}

	//<------------- Add sensors-----------------> 
	// add the sonar to the robot
	robot.addRangeDevice(&sonar);
	// add the laser to the robot
	robot.addRangeDevice(&sick);


	//<------------- Open ther Robot-----------------> 
	if ((ret = con.open("/dev/ttyUSB3")) != 0)
	{
		str = con.getOpenMessage(ret);
		printf("Open failed: %s\n", str.c_str());
		Aria::shutdown();
		return 1;
	}

	// set the robot to use the given connection
	robot.setDeviceConnection(&con);

	// do a blocking connect, if it fails exit
	if (!robot.blockingConnect())
	{
		printf("Could not connect to robot... exiting\n");
		Aria::shutdown();
		return 1;
	}

	// turn on the motors, turn off amigobot sounds
	//robot.comInt(ArCommands::SONAR, 0);
	robot.comInt(ArCommands::SOUNDTOG, 0);

	// start the robot running, true so that if we lose connection the run stops
	robot.runAsync(true);

	//<------------- Start the Sick Laser Scanner----------------->
	sick.setDeviceConnection(&laserCon);
	if ((ret = laserCon.open("/dev/ttyUSB4")) != 0){
		Aria::shutdown();
		return 1;
	}
	sick.configureShort(false, ArSick::BAUD38400, ArSick::DEGREES180, ArSick::INCREMENT_ONE);

	sick.runAsync();
	if (!sick.blockingConnect()){
		printf("Could not get sick...exiting\n");
		Aria::shutdown();
		return 1;
	}
	printf("We are connected to the laser!");


	mySick = &sick;
	ArGlobalFunctor fu(&getnew);
	ArFunctor *f = &fu;

	//<------------- Define parameters of the robot ----------------->
	robot.lock();
	robot.comInt(ArCommands::ENABLE, 1);
	robot.unlock();
	mySick->addDataCB(f);

	robot.lock();
	robot.setVel(300);
	robot.unlock();

	//<------------- -------------------------------PARALLEL PARKING LOGIC------------------------------------------------>

	// Robot starts lined up with first car.  Moves forward
	// until its rear is lined up with the back bumper of
	// the second car, and pauses for a second while shifiting
	// into reverse.
	while (1)
	{

		//<------------- Print out values when started----------------->
		if (started){
			gettimeofday(&end, NULL);
			timersub(&end, &start, &total);
			myfile4 << (total.tv_sec + total.tv_usec / float(1000000)) << "S " << "Cardist =" << carDist << "\t" << (double(carDist) / 300.0) << std::endl;

			if ((total.tv_sec + total.tv_usec / float(1000000)) > (double(carDist) / 300.0) && !carPassed){
				myfile << "CAR PASSED2! car dist:" << carDist << "next car dist:" << nextCarDist << endl;
				carPassed = true;
			}
		}

		//<------------- Parallel Parking manuver----------------->
		if (isParkSpace && carPassed){
			myfile << "FOUND SPACE sums:" << sums << "car dist:" << carDist << " next car dist:" << nextCarDist << endl;
			gettimeofday(&start, NULL);
			//ArUtil::sleep(4500); 
			bool arrived = false;
			while (!arrived)
			{
				gettimeofday(&end, NULL);
				timersub(&end, &start, &total);
				if (nextCarDist != -1)
				{
					if ((total.tv_sec + total.tv_usec / float(1000000)) > (double(nextCarDist) / 300.0))
					{
						arrived = true;
						myfile << "Arrived! car dist:" << carDist << "next car dist:" << nextCarDist << endl;
					}
				}
				else if ((total.tv_sec + total.tv_usec / float(1000000)) > (1400 / 300.0))
				{
					arrived = true;
					myfile << "Arrived! car dist:" << carDist << "next car dist:" << nextCarDist << endl;
				}
			}

			robot.lock();
			robot.stop();
			robot.unlock();
			libvlc_media_player_stop(mp, NULL);
			libvlc_media_player_play(mp2, NULL);
			ArUtil::sleep(2000);

			// Robot starts backing up while simultaneously turning
			// with a constant rotational velocity
			robot.lock();
			robot.setVel(-100);
			robot.unlock();
			robot.lock();
			robot.setRotVel(7);
			robot.unlock();
			ArUtil::sleep(6300);

			// The robot turns in the opposite direction (same magnitude
			// of rotational velocity as beginning, just different sign)
			// It spends the same amount of time on this step as it did
			// on the first step.
			robot.lock();
			robot.setRotVel(-7);
			robot.unlock();
			ArUtil::sleep(6300);
			robot.lock();
			robot.stop();
			robot.unlock();
			libvlc_media_player_stop(mp2, NULL);

			// Rotational velocity is set to zero.  Robot moves
			// forward for a short while to center itself in the spot.
			robot.lock();
			robot.setRotVel(0);
			robot.setVel(150);
			robot.unlock();
			ArUtil::sleep(2700);
			robot.lock();
			robot.stop();
			robot.unlock();
			libvlc_media_player_play(mp3, NULL);
			break;
		}

		if (seenCar && !carPassed && !started){
			gettimeofday(&start, NULL);
			started = true;
		}

		//<------------- Align robot parallel to wall at start (Not Working) ----------------->

		/*	if(!carNearPassed)
		{
		if (wallDist < 600 && wallDist != -1){
		robot.lock();
		robot.setDeltaHeading(DELHEAD);
		robot.unlock();
		alpha = alpha + DELHEAD;
		ArUtil::sleep(300);
		robot.lock();
		//robot.setDeltaHeading(-DELHEAD);
		robot.unlock();
		}
		if (wallDist > 750 && wallDist != -1){
		robot.lock();
		robot.setDeltaHeading(-DELHEAD);
		robot.unlock();
		alpha = alpha - DELHEAD;
		ArUtil::sleep(300);
		robot.lock();
		//robot.setDeltaHeading(DELHEAD);
		robot.unlock();
		}
		}
		*/


	}

	robot.waitForRunExit();
	return 0;
}
