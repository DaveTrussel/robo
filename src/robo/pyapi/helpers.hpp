/*
@(C) F&P Robotics AG, 2017 and onwards
@Project: MCM

@Created: 13-JAN-2016 by Lucas Giger
@Modified: SEP-2017 by Gerhard Wesp, KISS Technologies

Includes and helpers for Python APIs
*/

#pragma once

// TODO: Document this.  Confusing, because we're creating a shared library.
#define BOOST_PYTHON_STATIC_LIB
#include "robo/typedef.hpp"
#include "util/assert.hpp"

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>


// Yes, we pollute the namespace
using namespace boost::python;
namespace  p = boost::python; // TODO do not polute the namespace

namespace detail_ {

template<typename CONT>
list container_to_py_list(const CONT& cont) {
  list l;
  for (const auto& ele : cont) {
    l.append(ele);
  }
  return l;
}

/// Converts a std::vector<T> to a Python list
template<class T>
list std_vector_to_py_list(const std::vector<T>& v)
{
  return container_to_py_list(v);
}

/// Converts a Python list to a std::vector<T>.  Throws if an element
/// can not be converted.
template <typename T>
std::vector<T> py_list_to_std_vector(const object& l)
{
  return std::vector<T>(stl_input_iterator<T>(l),
                        stl_input_iterator<T>());
}

/// Converts a robo::VectorXd to a Python list
/// (same as Eigen::VectorXd but with a maximum size for stack allocation)
inline list eigen_vector_to_py_list(const robo::VectorXd v)
{ 
  const int N = v.size();
  list ret;
  for(int i=0; i<N; ++i){
    ret.append(v[i]);
  }
  return ret;
}

/// Converts a Python list to a robo::VectorXd
/// (same as Eigen::VectorXd but with a maximum size for stack allocation)
inline robo::VectorXd py_list_to_eigen_vector(const object& l)
{
  const int N = len(l);
  robo::VectorXd ret(N);
  for(int i=0; i<N; ++i){
    ret[i] = extract<double>(l[i]);
  }
  return ret;
}


// TODO templatize
inline robo::Matrix3d
py_list_to_eigen_matrix(p::list l){
  robo::Matrix3d ret = robo::Matrix3d::Zero();
  util::verify(9 == p::len(l), "robo: Wrong size to convert to 3x3 matrix");
  for (auto r=0; r<3; ++r) { // row
    for (auto c=0; c<3; ++c) { // col
      ret(r,c) = p::extract<double>(l[3*r+c]);
    }
  }
  return ret;
}

inline p::list
eigen_matrix_to_py_list(robo::Matrix3d mat) {
  p::list ret;
  for (auto i=0; i<3; ++i) { // row
    for (auto j=0; j<3; ++j) { // col
      ret.append(mat(i,j));
    }
  }
  return ret;
}

//Class to acquire the GIL
class AcquireGIL 
{
public:
    AcquireGIL(){
        state = PyGILState_Ensure();
    }

    ~AcquireGIL(){
        PyGILState_Release(state);
    }
private:
    PyGILState_STATE state;
};

/// Class to do a scoped GIL release
struct ScopedGILRelease
{
    ScopedGILRelease()
    {
        thread_state_ = PyEval_SaveThread();
    }

    ~ScopedGILRelease()
    {
        PyEval_RestoreThread(thread_state_);
    }

private:
    PyThreadState* thread_state_;
};

} // namespace detail_
