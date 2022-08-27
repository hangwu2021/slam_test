#pragma once

#include "common_include.hpp"

class Config 
{
private:
    //typedef std::shared_ptr<Config> Ptr;
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;
    
    Config() {}
    
public:
    ~Config();
    
    static void setParameterFile(const std::string& filename);
    
    template<typename T>
    static T get(const std::string& key)
    {
        return T(Config::config_->file_[key]);
    }
};
