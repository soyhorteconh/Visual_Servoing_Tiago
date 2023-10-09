#include <sys/time.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <inria_utils/SpaceMouse.hpp>
#include <inria_utils/File.h>
#include <inria_utils/String.h>

namespace inria {

SpaceMouse::SpaceMouse() :
    _timeStart(),
    _fd(-1),
    _devicePath(),
    _isContinue(false),
    _thread(),
    _inputLinX(0.0),
    _inputLinY(0.0),
    _inputLinZ(0.0),
    _inputAngX(0.0),
    _inputAngY(0.0),
    _inputAngZ(0.0),
    _timeLinX(0.0),
    _timeLinY(0.0),
    _timeLinZ(0.0),
    _timeAngX(0.0),
    _timeAngY(0.0),
    _timeAngZ(0.0),
    _isButtonLeft(false),
    _isButtonRight(false)
{
}

void SpaceMouse::openDevice(const std::string& devicePath)
{
    //Open device in blocking mode
    _fd = ::open(devicePath.c_str(), O_RDONLY);
    _devicePath = devicePath;
    if (_fd < 0) {
        throw std::runtime_error(
            "inria::SpaceMouse::openDevice: Unable to open device: " 
            + _devicePath);
    }

    //Start reader thread
    _isContinue.store(true);
    _thread = std::thread([this](){this->mainReader();});
}

void SpaceMouse::openDefault()
{
    std::vector<std::string> devices = inria::SpaceMouse::listDevices();
    if (devices.size() == 0) {
        throw std::runtime_error(
            "inria::SpaceMouse::openDefault: No device found.");
    }
    openDevice(devices[0]);
}

SpaceMouse::~SpaceMouse()
{
    //Wait for reader thread to end
    _isContinue.store(false);
    _thread.join();
    //Close the device
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
        _inputLinX.store(0.0);
        _inputLinY.store(0.0);
        _inputLinZ.store(0.0);
        _inputAngX.store(0.0);
        _inputAngY.store(0.0);
        _inputAngZ.store(0.0);
        _timeLinX.store(0.0);
        _timeLinY.store(0.0);
        _timeLinZ.store(0.0);
        _timeAngX.store(0.0);
        _timeAngY.store(0.0);
        _timeAngZ.store(0.0);
        _isButtonLeft.store(false);
        _isButtonRight.store(false);
    }
}

Eigen::Vector3d SpaceMouse::getLin() const
{
    Eigen::Vector3d input;
    input.x() = _inputLinX.load();
    input.y() = _inputLinY.load();
    input.z() = _inputLinZ.load();
    //Apply normalization
    for (size_t i=0;i<3;i++) {
        input(i) = input(i)/250.0;
        if (input(i) > 1.0) {
            input(i) = 1.0;
        }
        if (input(i) < -1.0) {
            input(i) = -1.0;
        }
    }
    return input;
}
Eigen::Vector3d SpaceMouse::getAng() const
{
    Eigen::Vector3d input;
    input.x() = _inputAngX.load();
    input.y() = _inputAngY.load();
    input.z() = _inputAngZ.load();
    //Apply normalization
    for (size_t i=0;i<3;i++) {
        input(i) = input(i)/250.0;
        if (input(i) > 1.0) {
            input(i) = 1.0;
        }
        if (input(i) < -1.0) {
            input(i) = -1.0;
        }
    }
    return input;
}

bool SpaceMouse::isButtonLeft() const
{
    return _isButtonLeft.load();
}
bool SpaceMouse::isButtonRight() const
{
    return _isButtonRight.load();
}
        
std::vector<std::string> SpaceMouse::listDevices()
{
    //Read all system input devices
    std::vector<std::string> lines = StrExplode(
        FileRead("/proc/bus/input/devices"), '\n');
    //Parse system file
    std::vector<std::string> devices;
    for (size_t i=0;i<lines.size();i++) {
        if (
            lines[i].find("Vendor=256f") != std::string::npos &&
            (lines[i].find("Product=c635") != std::string::npos ||
            lines[i].find("Product=c62e") != std::string::npos)
        ) {
            for (
                size_t j=1;
                i+j<lines.size() && lines[i+j].size() >= 1 && lines[i+j][0] != 'I';
                j++
            ) {
                size_t pos = lines[i+j].find("Handlers=");
                if (pos != std::string::npos) {
                    std::vector<std::string> tmpTerms = StrExplode(
                        lines[i+j].substr(pos+9, lines[i+j].size()-pos), 
                        ' ');
                    for (size_t k=0;k<tmpTerms.size();k++) {
                        if (tmpTerms[k].find("event") != std::string::npos) {
                            devices.push_back("/dev/input/" + tmpTerms[k]);
                        }
                    }
                }
            }
        }
    }

    return devices;
}

void SpaceMouse::mainReader()
{
    //Check that device is initialized
    if (_fd < 0) {
        throw std::runtime_error(
            "inria::SpaceMouse::updateRead: "
            "Device not initialized: " + _devicePath);
    }

    //Set time offset
    struct timeval tv;
    ::gettimeofday(&tv, nullptr);
    _timeStart = tv.tv_sec;
    
    //Polling structure setup
    struct pollfd fds;
    fds.fd = _fd;
    fds.events = POLLIN | POLLPRI;

    //Parameters
    const double timeOut = 0.2;
    const double deadbandLin = 5.0;
    const double deadbandAng = 20.0;

    //Keep reading events with timeout
    struct Event_t event;
    while (_isContinue.load()) {
        //Set velocity to zero if no new command 
        //has been received after a timeout
        double timeNow = getTimeNow();
        if (timeNow-_timeLinX.load() > timeOut) {
            _inputLinX.store(0.0);
        }
        if (timeNow-_timeLinY.load() > timeOut) {
            _inputLinY.store(0.0);
        }
        if (timeNow-_timeLinZ.load() > timeOut) {
            _inputLinZ.store(0.0);
        }
        if (timeNow-_timeAngX.load() > timeOut) {
            _inputAngX.store(0.0);
        }
        if (timeNow-_timeAngY.load() > timeOut) {
            _inputAngY.store(0.0);
        }
        if (timeNow-_timeAngZ.load() > timeOut) {
            _inputAngZ.store(0.0);
        }
        //Check if data are available to be read with a timeout
        int retevent = poll(&fds, 1, 20);
        if (retevent == -1) {
            throw std::logic_error(
                "inria::SpaceMouse: "
                "Error polling data: " +
                std::string(strerror(errno)));
        }
        for (int k=0;k<retevent;k++) {
            //Read available element
            ssize_t bytes = ::read(_fd, &event, sizeof(event));
            if (bytes == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                } else {
                    throw std::runtime_error(
                        "inria::SpaceMouse::updateRead: "
                        "Read error: " + std::string(strerror(errno)));
                }
            }
            if (bytes == sizeof(event)) {
                if (event.type == 2 || event.type == 3) {
                    if (event.code == 0) {
                        _inputLinY.store(deadband(-event.value, deadbandLin, true));
                        _timeLinY.store(getTimeFrom(event.time));
                    }
                    if (event.code == 1) {
                        _inputLinX.store(deadband(-event.value, deadbandLin, true));
                        _timeLinX.store(getTimeFrom(event.time));
                    }
                    if (event.code == 2) {
                        _inputLinZ.store(deadband(-event.value, deadbandLin, true));
                        _timeLinZ.store(getTimeFrom(event.time));
                    }
                    if (event.code == 3) {
                        _inputAngY.store(deadband(-event.value, deadbandAng, true));
                        _timeAngY.store(getTimeFrom(event.time));
                    }
                    if (event.code == 4) {
                        _inputAngX.store(deadband(-event.value, deadbandAng, true));
                        _timeAngX.store(getTimeFrom(event.time));
                    }
                    if (event.code == 5) {
                        _inputAngZ.store(deadband(-event.value, deadbandAng, true));
                        _timeAngZ.store(getTimeFrom(event.time));
                    }
                }
                if (event.type == 1) {
                    if (event.code == 256) {
                        _isButtonLeft.store(event.value);
                    }
                    if (event.code == 257) {
                        _isButtonRight.store(event.value);
                    }
                }
            } else {
                throw std::runtime_error(
                    "inria::SpaceMouse::updateRead: "
                    "Read failed: " + _devicePath);
            }
        }
    }
}

double SpaceMouse::getTimeNow() const
{
    struct timeval tv; 
    ::gettimeofday(&tv, nullptr);
    return getTimeFrom(tv);
}
double SpaceMouse::getTimeFrom(const struct timeval tv) const
{
    return (double)(tv.tv_sec-_timeStart) + ((double)tv.tv_usec)*1e-6;
}

double SpaceMouse::deadband(
    double value, double width, bool isContinuous) const
{
    if (width < 0.0) {
        width = 0.0;
    }

    if (isContinuous) {
        if (value >= width) {
            return value - width;
        } else if (value <= -width) {
            return value + width;
        } else {
            return 0.0;
        }
    } else {
        if (std::fabs(value) <= width) {
            return 0.0;
        } else {
            return value;
        }
    }
}

}

