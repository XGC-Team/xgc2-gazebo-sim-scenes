// The model SDF strings the scene adapter writes for obstacles, for
// sdf_parser_test and sdf_parser_benchmark. Not part of the package interface.
#pragma once

#include <iomanip>
#include <sstream>
#include <string>

namespace xgc2_gazebo_scene {

inline std::string GeometrySdf(int kind, int part) {
    std::ostringstream out;
    out << std::setprecision(17);
    const double a = 0.1 + part * 0.3141592653589793;
    switch (kind % 4) {
    case 0:
        out << "<box><size>" << a << " " << a * 2.7 << " " << a / 3 << "</size></box>";
        break;
    case 1:
        out << "<sphere><radius>" << a << "</radius></sphere>";
        break;
    case 2:
        out << "<cylinder><radius>" << a << "</radius><length>" << a * 4.1 << "</length></cylinder>";
        break;
    default:
        out << "<mesh><uri>/tmp/xgc2-scene-meshes-AbCdEf/0123456789abcdef.obj</uri><scale>1 1 1</scale></mesh>";
        break;
    }
    return out.str();
}

// The structure CompileScene writes for an obstacle.
inline std::string ObstacleSdf(int index, int parts) {
    std::ostringstream out;
    out << std::setprecision(17) << "<sdf version='1.6'><model name='xgc2_obstacle_scene_" << index << "'><pose>"
        << index * 0.7 << " " << index / 3.0 << " 0.5 0 0 " << index * 0.01 << "</pose><static>true</static>"
        << "<link name='body'><gravity>false</gravity>";
    for (int part = 0; part < parts; ++part) {
        const std::string geometry = GeometrySdf(index + part, part);
        out << "<collision name='part_" << part << "'><pose>0 0 " << part * 0.1 << " 0 0 0</pose><geometry>" << geometry
            << "</geometry></collision><visual name='part_" << part << "'><pose>0 0 " << part * 0.1
            << " 0 0 0</pose><geometry>" << geometry << "</geometry><material><ambient>0.1 0.2 0.3 1</ambient>"
            << "<diffuse>0.1 0.2 0.3 1</diffuse></material><transparency>" << 0.25 * (part % 3)
            << "</transparency></visual>";
    }
    out << "</link></model></sdf>";
    return out.str();
}

} // namespace xgc2_gazebo_scene
