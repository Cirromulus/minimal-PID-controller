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
    _pre_error(0),
    _integral(0)
{}

double PID::calculate( double setpoint, double pv,
                       const Settings& set )
{
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = set.Kp * error;

    // Integral term
    _integral += error * set.dt;
    double Iout = set.Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / set.dt;
    double Dout = set.Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    double min = isnan(set.min) ? -set.max : set.min;
    if( !isnan(set.max) && output > set.max )
        output = set.max;
    else if( !isnan(min) && output < min )
        output = min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

