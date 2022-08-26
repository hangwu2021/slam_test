#include "config.hpp"

void Config::setParameterFile(const std::string& filename)
{
    if (config_ == nullptr)
    {
        config_ = std::shared_ptr<Config>(new Config);
    }
    
    config_->file_ = cv::FileStorage(filename, cv::FileStorage::READ);
    if (config_->file_.isOpened() == false)
    {
        std::cerr << "Parameter File Open Failed." << std::endl;
        return;
    }
}

Config::~Config()
{
    if (file_.isOpened())
    {
        file_.release();
    }
}

std::shared_ptr<Config> Config::config_ = nullptr;
