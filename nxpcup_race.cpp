/****************************************************************************
 *
 *   Copyright 2019 NXP.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */
#include "nxpcup_race.h"
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <stdlib.h>
#include <math.h>
#define X_CENTER         (pixy.frameWidth/2) // center of the road
using namespace std;


roverControl raceTrack(Pixy2 &pixy)
{

	roverControl control{};
	control.speed=1.0;
	control.steer=1;
	int res;


	   //Get latest data from Pixy, including main vector, new intersections and new barcodes.
	  res = pixy.line.getMainFeatures();



	  // If error or nothing detected, stop motors
	  if (res<=0)
	  {
		control.speed=-0.2;
		control.steer=0;
	    PX4_INFO("STOP");
	    return control;
	  }


	  pixy.line.vectors->print();// display the vector coordinates

	  // We found the vector...
	  if (res&LINE_VECTOR){

		  control.speed=1.0;//straight line speed
		  //If the difference between the points is less than the 2, this mean that the inclination of the vector is relative 0
		  if((pixy.line.vectors->m_x1-pixy.line.vectors->m_x0<=2)&&(pixy.line.vectors->m_x1-pixy.line.vectors->m_x0>=-2)){
			  //Increase the value of the speed because we are on one straight line
			  control.speed = control.speed+1.0;
			  //The steer is 0
			  control.steer = 0.0;
		  }else{
			  if((pixy.line.vectors->m_x1>31)&&(pixy.line.vectors->m_x0<14)){ // right turn
				  //The first case is when we need to turn the wheels so much.
				  if((pixy.line.vectors->m_y1-pixy.line.vectors->m_y0)<10){
					PX4_INFO("virage à droite");
					control.speed=0.40;
					control.steer=0.9;
					return control;
				  }else{
					//The second one is when the angle is smaller.
					PX4_INFO("virage à droite");
					control.speed=0.60;
					control.steer=0.5;
					return control;
				  }
			  }else if((pixy.line.vectors->m_x1<44)&&(pixy.line.vectors->m_x0>61)){ // left turn
				  //The first case is when we need to turn the wheels so much.
				  if((pixy.line.vectors->m_y1-pixy.line.vectors->m_y0)<10){
						PX4_INFO("virage à droite");
						control.speed=0.40;
						control.steer=-0.9;
						return control;
					}else{
						PX4_INFO("virage à droite");
						control.speed=0.60;
						control.steer=-0.5;
						return control;
					}
			  }else if((pixy.line.vectors->m_x0<39)&&(pixy.line.vectors->m_x1>62)){ // inverse vector problem
				  PX4_INFO("virage à gauche 2");
				  control.speed=0.35;
				  control.steer=-0.9;
			  }else{ //position control
				  if((pixy.line.vectors->m_x1>X_CENTER)&&(pixy.line.vectors->m_x0>X_CENTER)){ //vector on the right side
					if (pixy.line.vectors->m_x1>53){
						PX4_INFO("décale à gauche 1");
						control.steer=-0.3;
					}else{
						PX4_INFO("décale à droite 1");
						control.steer=0.3;
					}
				  }else if((pixy.line.vectors->m_x1<=X_CENTER)&&(pixy.line.vectors->m_x0<=X_CENTER)){ //vector on the left side
					if (pixy.line.vectors->m_x1>18){
						PX4_INFO("décale à droite 2");
						control.steer=0.3;
					}else{
						PX4_INFO("décale à gauche 2");
						control.steer=-0.3;
					}
				  }else if((pixy.line.vectors->m_x1>=X_CENTER)&&(pixy.line.vectors->m_x0<=X_CENTER)){
					  PX4_INFO("décale à droite 2");
					  control.speed=0.35;
					  control.steer=0.3;
				   }else if((pixy.line.vectors->m_x1<=X_CENTER)&&(pixy.line.vectors->m_x0>=X_CENTER)){
					   PX4_INFO("décale à droite 2");
					  control.speed=0.35;
					  control.steer=-0.3;
				   }else{
					   control.speed=0.2;
					   control.steer=0.0;
				   }

			  }
		  }
	  }
	  return control;
}
