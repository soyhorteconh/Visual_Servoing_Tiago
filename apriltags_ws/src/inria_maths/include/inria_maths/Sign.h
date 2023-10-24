#ifndef INRIA_MATHS_SIGN_H
#define INRIA_MATHS_SIGN_H

#include <algorithm>
#include <stdexcept>

namespace inria {

/**
 * @return 1.0 if given value is
 * zero or positive, else return -1.0 
 */
constexpr double Sign(double value)
{
    return (value >= 0.0) ? 1.0 : -1.0;
}

/**
 * @return true if the two given 
 * values have the same sign
 */
constexpr bool IsSameSign(double val1, double val2)
{
    return (val1*val2 >= 0.0);
}

}

#endif

