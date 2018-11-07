#include <string>
#include <../include/API.h>

#include <boost/type_index.hpp>
#include <iostream>

#include "jlcxx/jlcxx.hpp"
#include "jlcxx/functions.hpp"
#include "jlcxx/array.hpp"
#include "jlcxx/tuple.hpp"

// #include "boost/tuple/tuple.hpp"
// typedef boost::tuple< std::vector< std::vector<uint32_t> >, std::vector<uint32_t> > Custom_tuple;

// Testing commands:
// open julia
// > using CxxWrap; @wrapmodule(joinpath("/home/josh/dev/superpoint_graph/partition/cut-pursuit/src/", "libcpjl"))
// > cut_pursuit(rand(3,10), rand(10,2), Int32.(rand(1:10, 10)), Int32.(rand(1:10,10)), ones(Float64,10))

// template <typename T>
// jlcxx::Array<float> transform_julia(std::vector<T> in)
// {
//     jlcxx::Array<float> out[in.size()];
//     std::transform(in.begin(), in.end(), out, [](const double d) { return 0.5*d; });
//     return out;
// }

struct Component
{
    Component(std::vector<uint32_t> comp) : component(comp.begin(), comp.end()){}
    std::vector<uint32_t> component;
};

struct Result
{
    Result(std::vector<float> s, std::vector<uint32_t> in_comp, std::vector< std::vector<uint32_t> > comp) : solution(s.begin(), s.end()),
                                                               in_components(in_comp.begin(), in_comp.end()),
                                                               components(comp.begin(), comp.end()){}

    std::vector<float> solution;
    std::vector<uint32_t> in_components;
    std::vector<std::vector<uint32_t>> components;

    ~Result() { std::cout << "Destroying Result with message " << std::endl; }
};



// jlcxx::ArrayRef<std::vector<unsigned int>, 2>
// std::tuple<jlcxx::ArrayRef<float, 1>, jlcxx::ArrayRef<uint32_t, 1>, jlcxx::ArrayRef<size()uint32_t, 2>>
Result cutpursuit_seg(const uint32_t n_nodes,
                      const uint32_t n_edge,
                      const uint32_t n_obs,
                      jlcxx::ArrayRef<double, 2> obs,
                      jlcxx::ArrayRef<int, 1> source,
                      jlcxx::ArrayRef<int, 1> target,
                      jlcxx::ArrayRef<double, 1> edge_weight,
                      jlcxx::ArrayRef<double, 1> node_weight,
                      float lambda,
                      float mode,
                      float speed,
                      float verbose)
{
    float obs_data[obs.size()];
    std::transform(obs.begin(), obs.end(), obs_data, [](double d) { return static_cast<float>(d);});

    uint32_t source_data[source.size()];
    std::transform(source.begin(), source.end(), source_data, [](double d) { return static_cast<uint32_t>(d);});

    uint32_t target_data[source.size()];
    std::transform(target.begin(), target.end(), target_data, [](double d) { return static_cast<uint32_t>(d);});

    float edge_weight_data[source.size()];
    std::transform(edge_weight.begin(), edge_weight.end(), edge_weight_data, [](double d) { return static_cast<float>(d);});

    float node_weight_data[source.size()];
    std::transform(node_weight.begin(), node_weight.end(), node_weight_data, [](double d) { return static_cast<float>(d);});

    // float solution [n_ver * n_obs];
    std::vector<float> solution(n_nodes * n_obs, 0.0f);
    std::vector<uint32_t> in_component(n_nodes, 0);
    std::vector< std::vector<uint32_t> > components(1, std::vector<uint32_t>(1, 0.f));

    CP::cut_pursuit<float>(n_nodes, n_edge, n_obs, obs_data, source_data, target_data, edge_weight_data, node_weight_data, solution.data(), in_component, components, lambda, mode, speed, verbose);

    // auto solution_out = jlcxx::ArrayRef<float, 1>(solution.data(), solution.size());
    // auto in_component_out = jlcxx::ArrayRef<uint32_t, 1>(in_component.data(), in_component.size());
    // auto components_out = jlcxx::ArrayRef<uint32_t, 2>(components.data(), components.size());
    // auto out = jlcxx::make_julia_array(components.data(), components.size(), components[0].size());

    return Result(solution, in_component, components);
}

// std::cout << "Type of T: "
//           << boost::typeindex::type_id<T>().pretty_name()
//   << std::endl;

// template <typename T>
// struct StdVector
// {
//     StdVector(std::vector<uint32_t> i) : ptr(i){}
//     std::vector<uint32_t> ptr;
// };

JLCXX_MODULE define_julia_module(jlcxx::Module& mod)
{
    // jlcxx::static_type_mapping<StdVector>::set_julia_type((jl_datatype_t*)jlcxx::julia_type("StdVector"));

    mod.method("cut_pursuit", &cutpursuit_seg);

    mod.add_type<Result>("Result")
        .method("in_components_test", [](Result& f) { return std::make_tuple(&(f.in_components[0]),
                                                                             &(f.in_components[0])+ f.in_components.size()/sizeof(uint32_t),
                                                                             f.in_components.capacity()); })
        // .method("components", [](Result& f, int i) { return f.components.size()); })
        .method("num_components", [](Result& f) { return f.components.size(); })
        .method("in_components_disp", [](Result& f) {
            for (int i = 0; i < f.in_components.size(); i++)
            {
                std::cout << i << ": " << f.in_components[i] << std::endl;
            }
            return "hi";
            });
    //     .method("in_components", [](Result& f) { return jlcxx::ArrayRef<uint32_t>(&(f.in_components[0]), f.in_components.size()); })

}
