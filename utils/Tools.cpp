//
// Created by yangcheng on 2019/7/7.
//

#include "Tools.h"

Tools::Tools() {};
Tools::~Tools() {};

std::vector<std::string> Tools::split(const std::string &s, std::string split_tag) {
    std::string::size_type pos1, pos2;
    pos2 = s.find(split_tag);
    pos1 = 0;

    std::vector<std::string> v;
    while (std::string::npos != pos2) {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + split_tag.size();
        pos2 = s.find(split_tag, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
    return v;
}