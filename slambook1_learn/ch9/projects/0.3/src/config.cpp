#include "config.hpp"

std::shared_ptr<Config> Config::config_ = nullptr;

void Config::setParameterFile(const std::string& filename)
{
    if (config_ == nullptr)
    {
        config_ = std::shared_ptr<Config>(new Config);
    }
    
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false)
    {
        std::cerr << "Parameter File: " << filename << "is Opened fail or doest not exist!" << std::endl;
        config_->file_.release();
        return ;
    }
}

Config::~Config()
{
    if (file_.isOpened())
    {
        file_.release();
    }
}
