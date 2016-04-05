#include <rl_common/core.hh>

std::ostream& operator<< (std::ostream& out, const Eigen::Vector3d& v) {
  out << "[" << v(0) << ", " << v(1) << ", " << v(2) << "]";
}

std::ostream& operator<< (std::ostream& out, const geometry_msgs::Point& v) {
  out << "[" << v.x << ", " << v.y << ", " << v.z << "]";
}

std::ostream& operator<< (std::ostream& out, const geometry_msgs::Vector3& v) {
  out << "[" << v.x << ", " << v.y << ", " << v.z << "]";
}

std::ostream& operator<< (std::ostream& out, const geometry_msgs::Quaternion& v) {
  out << "[" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << "]";
}
