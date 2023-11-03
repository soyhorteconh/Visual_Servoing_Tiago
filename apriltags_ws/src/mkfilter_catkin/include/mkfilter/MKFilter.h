#ifndef MKFILTER_MKFILTER_H
#define MKFILTER_MKFILTER_H

#include <vector>

namespace mkfilter {

/**
 * Enum for filter type.
 * For defining a Chebyshev filter, 
 * an additional passband ripple 
 * in dB must be provided.
 */
enum class FilterType_t {
    Bessel,
    Butterworth,
    Chebyshev,
};

/**
 * Enum for pass type.
 * For Lowpass and Highpass, only
 * must be defined.
 * for Bandpass and Bandstop, 
 */
enum class PassType_t {
    Lowpass,
    Highpass,
    Bandpass,
    Bandstop,
};

/**
 * Compute the coefficients for an 
 * Infinite Impulse Respond filter based
 * on mkfilter implementation from Tony Fisher.
 * See https://www-users.cs.york.ac.uk/~fisher/mkfilter/
 *
 * Throw std::logic_error if an error occurs.
 * The implementation is thread safe.
 *
 * Gain and IIR coefficient are defined by:
 * sIn = coefIn.size()
 * sOut = coefOut.size()
 * x[n] = next input value / gain;
 * y[n] = 
 *     coefIn[0]*x[n-0] +
 *     ...
 *     coefIn[sIn-1]*x[n-sIn-1] +
 *     coefOut[1]*y[n-1] +
 *     ...
 *     coefOut[sOut-1]*y[n-sOut-1] +
 *
 * @param filter Filter type.
 * @param pass Pass type.
 * @param ripple Passband ripple in dB only 
 * for Chebyshev filter type. Else not used.
 * @param order Filter order in 1:10.
 * @param cornerRatio1 Corner (-dB) frequency as a fraction
 * of the sampling rate.
 * @param cornerRatio2 Upper corner frequency used 
 * for Bandpass and Bandstop pass types. Else not used.
 * 0 < cornerRatio1 < 0.5
 * 0 < cornerRatio2 < 0.5
 * cornerRatio1 < cornerRatio2
 *
 * @param[out] coefIn Coefficients over input signal.
 * @param[out] coefOut Coefficients over recursive 
 * previously computed output values.
 * @param[out] gain Gain to divide input value.
 */
void MKFilter(
    FilterType_t filter, 
    PassType_t pass, 
    double ripple,
    unsigned int order, 
    double cornerRatio1, 
    double cornerRatio2,
    std::vector<double>& coefIn,
    std::vector<double>& coefOut,
    double& gain);

}

#endif

