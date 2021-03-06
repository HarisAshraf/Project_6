/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Errors
    */
    double cte;     // current error
    double cte_i;   // error integral
    double cte_p;   // previous error
    double cte_d;   // error derivative
    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;
    /*
    * Output limits
    */
    double output_lim_max;
    double output_lim_min;
  
    bool lim_fl;     // flag if one of the limits is met
    /*
    * Delta time
    */
    double dt;
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    void UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H
