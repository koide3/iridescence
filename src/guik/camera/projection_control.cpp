#include <guik/camera/projection_control.hpp>

namespace guik {

std::istream& operator>>(std::istream& ist, ProjectionControl& cam) {
  cam.load(ist);
  return ist;
}

std::ostream& operator<<(std::ostream& ost, const ProjectionControl& cam) {
  cam.save(ost);
  return ost;
}

}