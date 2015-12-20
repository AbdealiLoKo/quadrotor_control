#include <rl_common/core.hh>

std::ostream& operator<< (std::ostream& out, const Eigen::Vector3d& v) {
  out << "[" << v(0) << ", " << v(1) << ", " << v(2) << "]";
}
