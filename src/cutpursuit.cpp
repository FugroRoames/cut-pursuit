#include <iostream>
#include <cstdio>
#include <vector>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <numpy/ndarrayobject.h>
#include "boost/tuple/tuple.hpp"
#include "boost/python/object.hpp"
#include <../include/API.h>

// #include "jlcxx/jlcxx.hpp"
// #include "jlcxx/array.hpp"
// #include "jlcxx/functions.hpp"

namespace bpn = boost::python::numpy;
namespace bp =  boost::python;

typedef boost::tuple< std::vector< std::vector<uint32_t> >, std::vector<uint32_t> > Custom_tuple;

struct VecToArray
{//converts a vector<uint32_t> to a numpy array
    static PyObject * convert(const std::vector<uint32_t> & vec) {
    npy_intp dims = vec.size();
    PyObject * obj = PyArray_SimpleNew(1, &dims, NPY_UINT32);
    void * arr_data = PyArray_DATA((PyArrayObject*)obj);
    memcpy(arr_data, &vec[0], dims * sizeof(uint32_t));
    return obj;
    }
};

struct FloatVecToArray
{//converts a vector<uint32_t> to a numpy array
    static PyObject * convert(const std::vector<float> & vec, uint32_t n_ver, uint32_t n_obs) {
    npy_intp dims[2];
    dims[0] = (npy_intp)n_ver;
    dims[1] = (npy_intp)n_obs;
    PyObject * obj = PyArray_SimpleNew(2, dims, NPY_FLOAT32);
    void * arr_data = PyArray_DATA((PyArrayObject*)obj);
    memcpy(arr_data, &vec[0], n_ver * n_obs * sizeof(float));
    return obj;
    }
};

template<class T>
struct VecvecToList
{//converts a vector< vector<T> > to a list
        static PyObject* convert(const std::vector< std::vector<T> > & vecvec)
    {
        boost::python::list* pylistlist = new boost::python::list();
        for(size_t i = 0; i < vecvec.size(); i++)
        {
            boost::python::list* pylist = new boost::python::list();
            for(size_t j = 0; j < vecvec[i].size(); j++)
            {
                pylist->append(vecvec[i][j]);
            }
            pylistlist->append((pylist, pylist[0]));
        }
        return pylistlist->ptr();
    }
};

struct to_py_tuple
{//converts output to a python tuple
    static PyObject* convert(const Custom_tuple& c_tuple){
        bp::list values;
        //add all c_tuple items to "values" list

        PyObject * vecvec_pyo = VecvecToList<uint32_t>::convert(c_tuple.get<0>());
        PyObject * vec_pyo = VecToArray::convert(c_tuple.get<1>());

        values.append(bp::handle<>(bp::borrowed(vecvec_pyo)));
        values.append(bp::handle<>(bp::borrowed(vec_pyo)));

        return bp::incref( bp::tuple( values ).ptr() );
    }
};

PyObject * cutpursuit_seg(const bpn::ndarray & xyz, const bpn::ndarray & obs, const bpn::ndarray & source, const bpn::ndarray & target,const bpn::ndarray & edge_weight,  float lambda, float mode, float speed)
{//read data and run the L0-cut pursuit partition algorithm
    const uint32_t n_ver = bp::len(obs);
    const uint32_t n_edg = bp::len(source);
    const uint32_t n_obs = bp::len(obs[0]);
    const float * xyz_data = reinterpret_cast<float*>(xyz.get_data());
    const float * obs_data = reinterpret_cast<float*>(obs.get_data());
    const uint32_t * source_data = reinterpret_cast<uint32_t*>(source.get_data());
    const uint32_t * target_data = reinterpret_cast<uint32_t*>(target.get_data());
    const float * edge_weight_data = reinterpret_cast<float*>(edge_weight.get_data());
//    float solution [n_ver * n_obs];
    std::vector<float> solution(n_ver * n_obs, 0.0f);
    std::vector<float> node_weight(n_ver, 1.0f);
    std::vector<uint32_t> in_component(n_ver,0);
    std::vector< std::vector<uint32_t> > components(1,std::vector<uint32_t>(1,0.f));
    CP::cut_pursuit<float>(n_ver, n_edg, n_obs, xyz_data, obs_data, source_data, target_data, edge_weight_data, &node_weight[0]
             , solution.data(), in_component, components, lambda, mode, speed, 2.f);
//    delete[]solution
    return to_py_tuple::convert(Custom_tuple(components, in_component));
}

// void cutpursuit_seg_jl(jlcxx::ArrayRef<double> xyz, jlcxx::ArrayRef<double> obs, jlcxx::ArrayRef<uint32_t*> source, jlcxx::ArrayRef<uint32_t*> target, jlcxx::ArrayRef<double*> edge_weight,  float lambda, float mode, float speed)
// {
//     const uint32_t n_ver = obs.size();
//     const uint32_t n_edg = source.size();
//     const uint32_t n_obs = obs.size();
//     const float * xyz_data = reinterpret_cast<float*>(xyz.data());
//     const float * obs_data = reinterpret_cast<float*>(obs.data());
//     const uint32_t * source_data = reinterpret_cast<uint32_t*>(source.data());
//     const uint32_t * target_data = reinterpret_cast<uint32_t*>(target.data());
//     const float * edge_weight_data = reinterpret_cast<float*>(edge_weight.data());
// //    float solution [n_ver * n_obs];
//     std::vector<float> solution(n_ver * n_obs, 0.0f);
//     std::vector<float> node_weight(n_ver, 1.0f);
//     std::vector<uint32_t> in_component(n_ver,0);
//     std::vector< std::vector<uint32_t> > components(1,std::vector<uint32_t>(1,0.f));
//     CP::cut_pursuit<float>(n_ver, n_edg, n_obs, xyz_data, obs_data, source_data, target_data, edge_weight_data, &node_weight[0]
//              , solution.data(), in_component, components, lambda, mode, speed, 2.f);
// }

std::string cutpursuit_test()
{
   return "hello, world";
}

// PyObject * cutpursuit_reg(const bpn::ndarray & obs, const bpn::ndarray & source, const bpn::ndarray & target,const bpn::ndarray & edge_weight,  float lambda)
// {//read data and run the L0-cut pursuit partition algorithm
//     const uint32_t n_ver = bp::len(obs);
//     const uint32_t n_edg = bp::len(source);
//     const uint32_t n_obs = bp::len(obs[0]);
//     const float * obs_data = reinterpret_cast<float*>(obs.get_data());
//     const uint32_t * source_data = reinterpret_cast<uint32_t*>(source.get_data());
//     const uint32_t * target_data = reinterpret_cast<uint32_t*>(target.get_data());
//     const float * edge_weight_data = reinterpret_cast<float*>(edge_weight.get_data());
// //    float solution [n_ver * n_obs];
//     std::cout << n_ver << " " << n_obs << std::endl;
//     std::vector<float> solution(n_ver * n_obs, 0.0f);
//     std::vector<float> node_weight(n_ver, 1.0f);
//     std::vector< std::vector<uint32_t> > components(1,std::vector<uint32_t>(1,0.f));
//     CP::cut_pursuit<float>(n_ver, n_edg, n_obs, obs_data, source_data, target_data, edge_weight_data, &node_weight[0]
//              , solution.data(), lambda, 1.f, 2.f, 2.f);
// //    delete[]solution
//     return FloatVecToArray::convert(solution, n_ver, n_obs);
// }

BOOST_PYTHON_MODULE(libcp)
{
    _import_array();
    Py_Initialize();
    bpn::initialize();
    bp::to_python_converter< Custom_tuple, to_py_tuple>();
    def("cutpursuit", cutpursuit_seg);
    // def("cutpursuit_reg", cutpursuit_reg);
}


// JLCXX_MODULE define_julia_module(jlcxx::Module& mod)
// {
//     mod.method("cutpursuit_test", cutpursuit_test);
// }

