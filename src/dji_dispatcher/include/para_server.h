#ifndef _PARA_SERVER_H_
#define _PARA_SERVER_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include "yaml-cpp/yaml.h"
using namespace std;

struct Vec3 {
    double x, y, z; /* etc - make sure you have overloaded operator== */
    bool operator ==(Vec3& r)
    {
        return (this->x == r.x)&&(this->y == r.y)&&(this->z==r.z);
    }
};


namespace YAML {
template<>
struct convert<Vec3> {
  static Node encode(const Vec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, Vec3& rhs) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    return true;
  }
};
}


class ParaServer
{
private:
    typedef struct{double lontitude; double latitude; double altitude;} area_location;
    YAML::Node server_node;



public:
    ParaServer(string file_name);
};


#endif

