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
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   tau_p = Kpi;
   tau_d = Kdi;
   tau_i = Kii;
   output_lim_min = output_lim_mini;
   output_lim_max = output_lim_maxi;
   cte_curr = 0.0;
   cte_prev = 0.0;
   diff_cte = 0.0;
   int_cte  = 0.0;
   delta_time = 0.0;
}


void PID::UpdateError(double cte) {

   /**
   * TODO: Update PID errors based on cte.
   **/
   if (delta_time > 0.0) {
    cte_curr = cte;
    diff_cte = (cte_curr - cte_prev)/delta_time;
    int_cte += cte_curr * delta_time;
    cte_prev = cte;
   }
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
	
	control = +tau_p * cte_curr + tau_d * diff_cte + tau_i * int_cte;
	
	if (control > output_lim_max) {
       control = output_lim_max;
	}
    if (control < output_lim_min) {
       control = output_lim_min;
	}
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   delta_time = new_delta_time;
}