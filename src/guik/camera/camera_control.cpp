#include <guik/camera/camera_control.hpp>

namespace guik {

std::istream& operator>> (std::istream& ist, CameraControl& cam) {
  cam.load(ist);
  return ist;
}

std::ostream& operator<< (std::ostream& ost, const CameraControl& cam) {
  cam.save(ost);
  return ost;
}

}