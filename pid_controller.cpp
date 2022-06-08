/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * Initialize PID coefficients (and errors, if needed)
   **/
   this->Kp = Kpi;
   this->Ki = Kdi;
   this->Kd = Kii;
     
   this->output_lim_max = output_lim_maxi;
   this->output_lim_min = output_lim_mini;
  
   this->cte_i = 0.0;

   this->lim_fl = false;
  
   this->dt = 0.0;
  
}


void PID::UpdateError(double cte) {
   /**
   * Update PID errors based on cte.
   **/
   this->cte = cte;
}

double PID::TotalError() {
   /**
   * Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control, cte_d=0.0;
        
    // integrate error if within limits
    if (!lim_fl)
       cte_i += cte*dt;
        
    // diff error (ignore noise)   
    // divide by zero protection
    if (dt != 0.0) 
       cte_d = (cte - cte_p)/dt;
    else
       cte_d = 0.0;

    // PID formula 
    control = Kp*cte + Ki*cte_i + Kd*cte_d; 
        
        
    if (control > output_lim_max){
        control = output_lim_max;
        lim_fl = true;
    }  
    else if (control < output_lim_min){
        control = output_lim_min;
        lim_fl = true;
    }  
    else lim_fl = false;

    return control;
}

void PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
    this->dt = new_delta_time;
}   
