#include "config.hpp"


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./bin/run_vo config/default.yaml" << std::endl;
        return -1;
    }
    
    Config::setParameterFile(argv[1]);
    
    std::cout << "dataset_dir: " << Config::get<std::string>("dataset_dir") << std::endl;
    
    return 0;
}
