#include <string>
#include <../include/API.h>

#include "jlcxx/jlcxx.hpp"
#include "jlcxx/array.hpp"

struct Result
{
    Result(std::vector<float> s, std::vector<uint32_t> in_comp, std::vector< std::vector<uint32_t> > comp) : solution(s.begin(), s.end()),
                                                               in_components(in_comp.begin(), in_comp.end()),
                                                               components(comp.begin(), comp.end()){}

    std::vector<float> solution;
    std::vector<uint32_t> in_components;
    std::vector<std::vector<uint32_t>> components;

    ~Result() { std::cout << "Destroying CutPursuit Result" << std::endl; }
};

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
    //read data and run the L0-cut pursuit partition algorithm
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

    std::vector<float> solution(n_nodes * n_obs, 0.0f);
    std::vector<uint32_t> in_component(n_nodes, 0);
    std::vector< std::vector<uint32_t> > components(1, std::vector<uint32_t>(1, 0.f));

    CP::cut_pursuit<float>(n_nodes, n_edge, n_obs, obs_data, source_data, target_data, edge_weight_data, node_weight_data, solution.data(), in_component, components, lambda, mode, speed, verbose);

    return Result(solution, in_component, components);
}

JLCXX_MODULE define_julia_module(jlcxx::Module& mod)
{
    mod.method("cut_pursuit", &cutpursuit_seg);

    // TODO Work out how to return std::vector<std::vector<>>
    mod.add_type<Result>("Result")
        .method("in_components", [](Result& f) { return std::make_tuple(&(f.in_components[0]),
                                                                             &(f.in_components[0])+ f.in_components.size()/sizeof(uint32_t),
                                                                             f.in_components.capacity()); });
}
