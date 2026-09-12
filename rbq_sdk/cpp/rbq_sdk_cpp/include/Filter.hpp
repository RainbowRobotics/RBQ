#pragma once

#include <math.h>

namespace rbq_sdk {
namespace Filter {

float LPF(const float &input, const float &output, const float &f_cut, const float &ts) {
    float tau;
    float result = 0;
    if (f_cut != 0) {
        tau = 1.0 / (1.0 + 2*M_PI*f_cut*ts);
        result = tau*output + (1.0 - tau)*input;
    }
    return result;
}

} // namespace Filter
} // namespace rbq_sdk

