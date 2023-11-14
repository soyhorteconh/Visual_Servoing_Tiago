#include <sstream>
#include <cstring>
#include <mkfilter/MKFilter.h>

void mkfilter_process(
    int argc, char** argv,
    std::vector<double>& argCoefIn, 
    std::vector<double>& argCoefOut, 
    double& argGain);

namespace mkfilter {

void MKFilter(
    FilterType_t filter, 
    PassType_t pass, 
    double ripple,
    unsigned int order, 
    double cornerRatio1, 
    double cornerRatio2,
    std::vector<double>& coefIn,
    std::vector<double>& coefOut,
    double& gain)
{
    //Create textual command line option
    std::vector<std::string> parts;
    parts.push_back("mkfilter");
    //Filter type
    if (filter == FilterType_t::Bessel) {
        parts.push_back("-Be");
    }
    if (filter == FilterType_t::Butterworth) {
        parts.push_back("-Bu");
    }
    if (filter == FilterType_t::Chebyshev) {
        parts.push_back("-Ch");
        parts.push_back(std::to_string(ripple));
    }
    //Pass type
    if (pass == PassType_t::Lowpass) {
        parts.push_back("-Lp");
    }
    if (pass == PassType_t::Highpass) {
        parts.push_back("-Hp");
    }
    if (pass == PassType_t::Bandpass) {
        parts.push_back("-Bp");
    }
    if (pass == PassType_t::Bandstop) {
        parts.push_back("-Bs");
    }
    //Filter order
    parts.push_back("-o");
    parts.push_back(std::to_string(order));
    //Corner frequency ratio
    parts.push_back("-a");
    parts.push_back(std::to_string(cornerRatio1));
    if (
        pass == PassType_t::Bandpass ||
        pass == PassType_t::Bandstop
    ) {
        parts.push_back(std::to_string(cornerRatio2));
    }

    //Conversion argument list
    //to C style array
    int argc = parts.size();
    char** argv = new char*[argc+1];
    for (size_t i=0;i<parts.size();i++) {
        argv[i] = new char[1024];
        std::strcpy(argv[i], parts[i].c_str());
    }
    argv[argc] = nullptr;

    //Call original mkfilter implementation
    gain = 0.0;
    coefIn.clear();
    coefOut.clear();
    mkfilter_process(
        argc, argv, coefIn, coefOut, gain);

    //Memory free
    for (size_t i=0;i<parts.size();i++) {
        delete[] argv[i];
    }
    delete[] argv;
}

}

