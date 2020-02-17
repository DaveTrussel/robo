#pragma once

#include "robo/rotation.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>

#include <cmath>

namespace robo {

namespace util {

inline
void verify(const bool condition, const std::string& message) {
  if (not condition) { throw std::runtime_error(message); }
}

/// get the current epoch time in seconds
inline
double now() {
  using namespace std::chrono;
  return duration_cast<duration<double>>(
      high_resolution_clock::now().time_since_epoch()).count();
}

/// Measure elapsed time since creation of instance
class stop_watch {
private:
  const double start_;
public:
  stop_watch(): start_(now()) {};
  double elapsed_time() const { return now() - start_; }
};

} // namespace util

namespace math {

inline double pi() { return 3.1415926535897932384626433832795; }

/// @return deg [degrees] converted to [radians].  May be vectorized.
template<typename T>
T deg_to_rad(const T& deg) {
  return deg / 180.0 * pi();
}

/// @return rad [radians] converted to [degrees].  May be vectorized.
template<typename T>
T rad_to_deg(const T& rad) {
  return rad / pi() * 180.0;
}

/// Returns x - threshold if that is positive, 0 otherwise.
inline double excess(const double x, const double threshold) {
  return x > threshold ? x - threshold : 0.0;
}

} // namespace math

/// @return Log file name used for logging kinematics / dynamics related events
inline const char* logfile() { return "/tmp/.robo-log"; }

/// takes roll, pitch, yaw [deg] and converts them to a rotation matrix
inline Matrix3d
nautical_angles_to_matrix(double roll, double pitch, double yaw){
  Matrix3d ret;
  ret =   rotate_around_axis(Vector3d::UnitX(), math::deg_to_rad(roll ))
        * rotate_around_axis(Vector3d::UnitY(), math::deg_to_rad(pitch))
        * rotate_around_axis(Vector3d::UnitZ(), math::deg_to_rad(yaw  ));
  return ret;
}

inline Matrix3d
array_to_matrix(std::array<double, 9> arr){
  return Matrix3d(arr.data());
}

/// takes a STL Container of roll, pitch, yaw [deg] and converts them to a
/// rotation matrix
template <typename Container>
inline Matrix3d
nautical_angles_to_matrix(const Container& v){
  util::verify(3 == v.size(), "Size missmatch: #nautical angles != 3");
  return nautical_angles_to_matrix(v.at(0), v.at(1), v.at(2));
}


inline VectorXd rand_joint_vector(int size, double min, double max){
  double range = max - min;
  VectorXd r = (VectorXd::Random(size) + VectorXd::Ones(size)) / 2.0;
  return range*r + VectorXd::Constant(size, min);
}

/// Prints sorted timing results to std::cout
inline
void print_timing_result(std::vector<double> timings) {

  size_t len = timings.size();
  std::sort(timings.begin(), timings.end());
  double max = timings.back();
  double min = timings.front();
  double sum = std::accumulate(timings.begin(), timings.end(), 0.0);
  double average = sum / len;
  double median = timings.at(len/2);
  std::cout << "Timing results (in microseconds) " << len << " runs:"
            << "\nAverage: " << average
            << "\nMedian:  " << median
            << "\nMin:     " << min
            << "\nMax:     " << max << std::endl;
}

/// Generates a VectorXd view of a STL Container
template<typename Container>
inline VectorXd eigen_view(const Container& v){
  return VectorXd::Map(v.data(), v.size());
}

/// Makes a VectorXd copy of a STL Container
template<typename Container>
inline VectorXd eigen_copy(const Container& v){
  const unsigned int n = v.size();
  VectorXd ret(n);
  for(unsigned int i=0; i<n; i++){
    ret[i] = v.at(i);
  }
  return ret;
}

/// Makes a std::vector copy of a robo::VectorXd
template<typename Scalar>
inline std::vector<Scalar> from_eigen(VectorXd v){
  return std::vector<Scalar>(v.data(), v.data() + v.size());
}

} // namespace robo
