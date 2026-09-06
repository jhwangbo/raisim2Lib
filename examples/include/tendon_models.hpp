// Embedded URDF keeps the mechanisms asset-free and supports World XML export.
#pragma once
#include <locale>
#include <sstream>
#include <string>

namespace raisim_examples::tendons {
inline std::string boxLink(const std::string& name, const std::string& size,
                           const std::string& center, const std::string& color,
                           const std::string& extraVisuals="") {
  const auto origin="<origin xyz=\""+center+"\"/>";
  const auto geometry="<geometry><box size=\""+size+"\"/></geometry>";
  return "<link name=\""+name+"\"><inertial>"+origin+
    "<mass value=\"1\"/><inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"1\" iyz=\"0\" izz=\"1\"/>"
    "</inertial><visual>"+origin+geometry+"<material name=\""+name+"_color\"><color rgba=\""+
    color+"\"/></material></visual><collision>"+origin+geometry+"</collision>"+extraVisuals+"</link>";
}
inline std::string cableEye(const std::string& name, const std::string& position,
                            const std::string& color) {
  return "<visual><origin xyz=\""+position+"\"/><geometry><sphere radius=\"0.032\"/></geometry>"
    "<material name=\""+name+"\"><color rgba=\""+color+"\"/></material></visual>";
}
inline std::string fixedBase(const std::string& name, double x, double y) {
  std::ostringstream position; position.imbue(std::locale::classic());
  position << x << ' ' << y << " 0";
  return "<robot name=\""+name+"\"><link name=\"world\"/><link name=\""+name+"_base\"/>"
    "<joint name=\""+name+"_fixed\" type=\"fixed\"><parent link=\"world\"/><child link=\""+
    name+"_base\"/><origin xyz=\""+position.str()+"\"/></joint>";
}
inline std::string pulleyModel(const std::string& name, bool divided, double x, double y) {
  return fixedBase(name,x,y)+
    boxLink(name+"_left_body","0.36 0.36 0.44","0 0 0","0.08 0.46 0.86 1")+
    boxLink(name+"_right_body","0.36 0.36 0.44","0 0 0","1 0.58 0.08 1")+
    "<joint name=\""+name+"_left\" type=\"prismatic\"><parent link=\""+name+"_base\"/>"
    "<child link=\""+name+"_left_body\"/><origin xyz=\"-0.48 0 1.25\"/><axis xyz=\"0 0 1\"/>"
    "<limit lower=\"-0.7\" upper=\"0.7\" effort=\"1000\" velocity=\"100\"/></joint>"
    "<joint name=\""+name+"_right\" type=\"prismatic\"><parent link=\""+name+"_base\"/>"
    "<child link=\""+name+"_right_body\"/><origin xyz=\""+(divided ? "1.05" : "0.48")+
    " 0 1.55\"/><axis xyz=\"0 0 1\"/>"
    "<limit lower=\"-0.9\" upper=\"0.9\" effort=\"1000\" velocity=\"100\"/></joint></robot>";
}
inline std::string jointModel(const std::string& name, double x, double y) {
  const auto mount="<visual><origin xyz=\"0.275 -0.16 -1.05\"/><geometry>"
    "<box size=\"0.55 0.05 0.05\"/></geometry><material name=\""+name+"_bracket\">"
    "<color rgba=\"0.36 0.42 0.48 1\"/></material></visual>";
  return fixedBase(name,x,y)+
    boxLink(name+"_upper","0.2 0.28 1.05","0 0 -0.525","0.08 0.46 0.86 1",
      mount+cableEye(name+"_a_end","0 -0.16 -0.7","1 0.42 0.08 1")+
            cableEye(name+"_b_start","0.55 -0.16 -1.05","0.06 0.9 0.75 1"))+
    boxLink(name+"_lower","0.2 0.28 0.8","0 0 -0.4","1 0.58 0.08 1",
      cableEye(name+"_b_end","0 -0.16 -0.65","0.06 0.9 0.75 1"))+
    "<joint name=\""+name+"_shoulder\" type=\"revolute\"><parent link=\""+name+"_base\"/>"
    "<child link=\""+name+"_upper\"/><origin xyz=\"0 0 2.6\"/><axis xyz=\"0 1 0\"/>"
    "<limit lower=\"-1.2\" upper=\"1.2\" effort=\"1000\" velocity=\"100\"/></joint>"
    "<joint name=\""+name+"_elbow\" type=\"revolute\"><parent link=\""+name+"_upper\"/>"
    "<child link=\""+name+"_lower\"/><origin xyz=\"0 0 -1.05\"/><axis xyz=\"0 1 0\"/>"
    "<limit lower=\"-1.2\" upper=\"1.2\" effort=\"1000\" velocity=\"100\"/></joint></robot>";
}
}  // namespace raisim_examples::tendons
