/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 * Copyright 2021 Pascal Pieper <info@pascalpieper.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <cmath>
#include "pid.h"

using namespace std;

PID::PID() :
    _pre_output(0),
    _pre_error(0),
    _integral(0)
{};

PID::Settings PID::getDefault()
{
  return PID::Settings {.Kp = 1, .Ki = 0, .Kd = 0,
                   .dt = 1, .max = NAN, .min = NAN,
                   .max_dv = NAN};
}

double PID::calculate( double setpoint, double pv,
                       const Settings& set )
{
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = set.Kp * error;

    // Integral term
    const double previous_integral = _integral;
    _integral += error * set.dt;
    double Iout = set.Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / set.dt;
    double Dout = set.Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    bool output_was_limited = false;

    // Restrict delta value
    if( !isnan(set.max_dv) ) {
        double delta_v = (output - _pre_output) / set.dt;
        if(delta_v > set.max_dv) {
          output = _pre_output + (set.max_dv * set.dt);
          output_was_limited = true;
        }
        else if (delta_v < -set.max_dv) {
          output = _pre_output - (set.max_dv * set.dt);
          output_was_limited = true;
        }
    }

    // Restrict to max/min
    double min = isnan(set.min) ? -set.max : set.min;
    if( !isnan(set.max) && output > set.max ) {
        output = set.max;
        output_was_limited = true;
    }
    else if( !isnan(min) && output < min ) {
        output = min;
        output_was_limited = true;
    }

    if(output_was_limited)
      _integral = previous_integral;  // cap integral to limit long-term error buildup

    // Save error to previous error
    _pre_error = error;

    // here to suppress erroneous behavior on acceleration change (NAN -> valid)
    _pre_output = output;
    return output;
}

