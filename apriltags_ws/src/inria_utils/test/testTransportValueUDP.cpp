#include <iostream>
#include <inria_utils/TransportValueUDP.hpp>

int main()
{
    inria::TransportValueUDPServer server;
    auto callback = [](
        const std::vector<std::string>& namesBool,
        const std::vector<std::string>& namesInt,
        const std::vector<std::string>& namesFloat,
        const std::vector<bool>& valuesBool,
        const std::vector<int64_t>& valuesInt,
        const std::vector<double>& valuesFloat)
    {
        std::cout << "Processing:" << std::endl;
        for (size_t i=0;i<namesBool.size();i++) {
            std::cout << "bool: " << namesBool[i] << " " << valuesBool[i] << std::endl;
        }
        for (size_t i=0;i<namesInt.size();i++) {
            std::cout << "int: " << namesInt[i] << " " << valuesInt[i] << std::endl;
        }
        for (size_t i=0;i<namesFloat.size();i++) {
            std::cout << "float: " << namesFloat[i] << " " << valuesFloat[i] << std::endl;
        }
    };
    server.listen(9994, callback);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    inria::TransportValueUDPClient client;
    client.connect("127.0.0.1", 9994);
    client.setBool("/path/val1", true);
    client.setFloat("/path/val2", 2.0);
    client.setFloat("/path/val3", 3.0);
    client.send();
    client.setFloat("/path/val3", 4.0);
    client.send();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return 0;
}

