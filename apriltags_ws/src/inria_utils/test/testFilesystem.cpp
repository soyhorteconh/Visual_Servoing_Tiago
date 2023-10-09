#include <iostream>
#include <inria_utils/Filesystem.h>

int main()
{
    std::string path1 = "/tmp/test";
    std::string path2 = "file:///tmp/test";
    std::string path3 = "file://src/";
    std::string path4 = "src/inria_talos_control/inria_utils/test/testFilesystem.cpp";
    std::string path5 = "./";
    std::string path6 = "inria_utils";
    std::string path7 = "package://talos_description/meshes/torso/torso_1.STL";
    std::string path8 = "talos_description";
    std::string path9 = "~/.bashrc";

    std::cout << "Test filesystem SystemResolvePath():" << std::endl << std::endl;
    std::cout << path1 << std::endl 
        << "---> " << inria::SystemResolvePath(path1) << std::endl;
    std::cout << path2 << std::endl 
        << "---> " << inria::SystemResolvePath(path2) << std::endl;
    std::cout << path3 << std::endl 
        << "---> " << inria::SystemResolvePath(path3) << std::endl;
    std::cout << path4 << std::endl 
        << "---> " << inria::SystemResolvePath(path4) << std::endl;
    std::cout << path5 << std::endl 
        << "---> " << inria::SystemResolvePath(path5) << std::endl;
    std::cout << path6 << std::endl 
        << "---> " << inria::SystemResolvePath(path6) << std::endl;
    std::cout << path7 << std::endl 
        << "---> " << inria::SystemResolvePath(path7) << std::endl;
    std::cout << path8 << std::endl 
        << "---> " << inria::SystemResolvePath(path8) << std::endl;
    std::cout << path9 << std::endl 
        << "---> " << inria::SystemResolvePath(path9) << std::endl;

    return 0;
}

