//
// Created by yangcheng on 2019/3/22.
//

#ifndef LOCATION_CONFIG_H
#define LOCATION_CONFIG_H


#include <string>
#include <map>
#include <exception>

class Config {
private:
    std::string delimiter;
    std::string comment;
    std::map<std::string, std::string> cfg_settings;

public:

    Config(std::string file_name, std::string delimiter = "=", std::string comment = "#");

    std::string Read(const std::string &key) const;

    bool FileExist(const std::string file_name);
    void ReadFile(const std::string file_name, const std::string delimiter, const std::string comment);

    friend std::istream &operator>>(std::istream &is, Config &cfg);
    friend std::ostream &operator<<(std::ostream &os, const Config &cfg);

    void Trim(std::string &s);
    void SetDelimiter(std::string &d);
    void SetComment(std::string &c);

};


#endif //LOCATION_CONFIG_H
