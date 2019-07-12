//
// Created by yangcheng on 2019/7/7.
//

#ifndef LOCATION_TOOLS_H
#define LOCATION_TOOLS_H

#include <vector>
#include <string>

class Tools {

public:
    Tools();
    ~Tools();

   std::vector<std::string> split(const std::string &s, std::string split_tag);
};


#endif //LOCATION_TOOLS_H
