#include <iostream>
#include <mkfilter/MKFilter.h>

void testLowPass()
{
    double gain;
    std::vector<double> coefIn;
    std::vector<double> coefOut;

    mkfilter::MKFilter(
        mkfilter::FilterType_t::Butterworth,
        mkfilter::PassType_t::Lowpass,
        0.0,
        3, 10.0/500.0, 0.0,
        coefIn, coefOut, gain);

    std::cout << "==== LowPass:" << std::endl;
    std::cout << "gain = " << gain << std::endl;
    for (size_t i=0;i<coefIn.size();i++) {
        std::cout << "CoefIn[" << i << "] = " << coefIn[i] << std::endl;
    }
    for (size_t i=0;i<coefOut.size();i++) {
        std::cout << "CoefOut[" << i << "] = " << coefOut[i] << std::endl;
    }
}

void testHighPass()
{
    double gain;
    std::vector<double> coefIn;
    std::vector<double> coefOut;

    mkfilter::MKFilter(
        mkfilter::FilterType_t::Butterworth,
        mkfilter::PassType_t::Highpass,
        0.0,
        3, 10.0/500.0, 0.0,
        coefIn, coefOut, gain);

    std::cout << "==== HighPass:" << std::endl;
    std::cout << "gain = " << gain << std::endl;
    for (size_t i=0;i<coefIn.size();i++) {
        std::cout << "CoefIn[" << i << "] = " << coefIn[i] << std::endl;
    }
    for (size_t i=0;i<coefOut.size();i++) {
        std::cout << "CoefOut[" << i << "] = " << coefOut[i] << std::endl;
    }
}

void testBandPass()
{
    double gain;
    std::vector<double> coefIn;
    std::vector<double> coefOut;

    mkfilter::MKFilter(
        mkfilter::FilterType_t::Butterworth,
        mkfilter::PassType_t::Bandpass,
        0.0,
        3, 10.0/500.0, 20.0/500.0,
        coefIn, coefOut, gain);

    std::cout << "==== BandPass:" << std::endl;
    std::cout << "gain = " << gain << std::endl;
    for (size_t i=0;i<coefIn.size();i++) {
        std::cout << "CoefIn[" << i << "] = " << coefIn[i] << std::endl;
    }
    for (size_t i=0;i<coefOut.size();i++) {
        std::cout << "CoefOut[" << i << "] = " << coefOut[i] << std::endl;
    }
}

int main()
{
    testLowPass();
    testHighPass();
    testBandPass();

    return 0;
}

