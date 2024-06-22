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
   coef_proportional = Kpi;
   coef_differential = Kdi;
   coef_integral = Kii;

   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;

   cte = 0.0;
   cte_diff = 0.0;
   cte_total = 0.0;
   delta_time = 0.0;
}


void PID::UpdateError(double new_cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   cte_diff = delta_time > 0 ? (new_cte - cte)/delta_time : 0.0;
   cte = new_cte;
   cte_total += new_cte * delta_time;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double total_error = coef_proportional * cte + coef_differential * cte_diff + coef_integral * cte_total;
    return std::clamp(total_error, output_lim_min, output_lim_max);
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   delta_time = new_delta_time;

   return delta_time;
}