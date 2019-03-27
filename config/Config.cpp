//
// Created by yangcheng on 2019/3/22.
//

#include <fstream>
#include "Config.h"
#include <string>
#include "iostream"
#include <exception>

using namespace std;

class FileNotFoundException : public  exception {
    virtual const char* what() const throw()
    {
        return "file not found!";
    }
};

Config::Config(string file_name, string delimiter, string comment) : delimiter(delimiter), comment(comment) {
    std::ifstream in_file(file_name);
    cout << file_name << endl;
    if (!in_file) throw FileNotFoundException();
    in_file >> (*this);
    in_file.close();
}


istream &operator>>(istream &is, Config &cfg) {

    string line;
    while (getline(is, line)) {

        // 忽略开头注释和空行
        string::size_type pos = line.find(cfg.comment);
        if (line.length() == 0 || pos == 0) continue;

        // 忽略末尾注释
        string param = line;
        if (pos < string::npos) param = line.substr(0, pos + cfg.comment.length());

        // key value截取
        string::size_type pos2 = param.find(cfg.delimiter);
        if (pos2 < string::npos) {
            string key = param.substr(0, pos2);
            string value = param.replace(0, pos2 + 1, "");
            // 删除前后空白和一些特殊字符
            cfg.Trim(key);
            cfg.Trim(value);
            cfg.cfg_settings[key] = value;
        }
    }
    return is;
}

ostream &operator<<(std::ostream &os, const Config &cfg) {
    map<string, string>::const_iterator key_value;
    for (key_value = cfg.cfg_settings.begin(); key_value != cfg.cfg_settings.end(); ++key_value) {
        os << key_value->first << " : " << key_value->second << endl;
    }
    return os;
}

void Config::Trim(std::string &s) {
    // 删掉头尾的特殊符号
    const char whitespace[] = " \n\t\v\r\f";
    s.erase(0, s.find_first_not_of(whitespace));
    s.erase(s.find_last_not_of(whitespace) + 1);
}

void Config::SetDelimiter(std::string &d) {
    this->delimiter = d;
}

void Config::SetComment(std::string &c) {
    this->comment = c;
}

std::string Config::Read(const std::string &key) const {
    auto value = this->cfg_settings.find(key);
    if (value != this->cfg_settings.end()) {
        return value->second;
    } else {
        return "";
    }
}

bool Config::FileExist(const std::string file_name) {
    bool exist = false;
    ifstream in(file_name);
    if (in) {
        exist = true;
    }
    in.close();
    return exist;
}

void Config::ReadFile(const std::string file_name, const std::string delimiter, const std::string comment) {
    this->delimiter = delimiter;
    this->comment = comment;
    ifstream in(file_name);
    if (!in) {
        throw "file not found";
    } else {
        in >> (*this);
    }
}

