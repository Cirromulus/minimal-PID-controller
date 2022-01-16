/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
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

#include "pid.h"
#include <stdio.h>
#include <iostream>
#include <math.h> //for NAN

using namespace std;

int main() {

    const PID::Settings set{.Kp = .28, .Ki = 1, .Kd = .01,
      .dt = .1, .max = 600, .min = NAN, .max_dv = 500,
      .overshoot_integral_adaptation = 1
    };
    PID pid;

    double val = 0;
    const double target = 3;
    const double dampening = .6;
    double opposing_force = 50;

    //simulate stuck wheel
    for (int i = 0; i < 0; i++) {
        double inc = pid.calculate(target, val, set);
        printf("stuck val:% 7.3f inc:% 7.3f\n", val, inc);
    }

    cerr << "Kp: " << set.Kp << ", Ki: " << set.Ki << ", Kd: " << set.Kd << endl;
    cerr << "dv: " << set.max_dv << ", Fast integral overshoot adapt.: " << set.overshoot_integral_adaptation << endl;
    cerr << "Val, Setpoint, Err, P, I, D*10" << endl;

    for (int i = 0; i < 100; i++) {
        cerr << val << "," << target << ",";
        double inc = pid.calculate(target, val, set);
        printf("val:% 7.3f inc:% 7.3f\n", val, inc);
        val = val*dampening + (val + inc - opposing_force)*(1-dampening);

        if(i == 50)
          opposing_force = 10;
    }

    return 0;
}
