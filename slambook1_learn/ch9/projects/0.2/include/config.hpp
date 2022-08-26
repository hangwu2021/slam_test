#pragma once

#include "common_include.hpp"

class Config
{
private:
    Config() {}
    
public:
    ~Config();
    
    static void setParameterFile(const std::string& filename);
    
    template<typename T>
    static T get(const std::string& key)
    {
        return T(Config::config_->file_[key]);
    }
    
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;
};
