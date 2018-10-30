#include <string>
#include <../include/API.h>

#include "jlcxx/jlcxx.hpp"
#include "jlcxx/array.hpp"

// Testing commands:
// open julia
// > using CxxWrap; @wrapmodule(joinpath("/home/josh/dev/superpoint_graph/partition/cut-pursuit/src/", "libcpjl"))
// > cut_pursuit(rand(3,10), rand(10,2), Int32.(rand(1:10, 10)), Int32.(rand(1:10,10)), ones(Float64,10))

std::string cutpursuit_seg(jlcxx::ArrayRef<double, 2> obs, jlcxx::ArrayRef<int, 1> source, jlcxx::ArrayRef<int, 1> target, jlcxx::ArrayRef<double, 1> edge_weight)
{
    const uint32_t n_ver = 1;
    const uint32_t n_edg = 1;
    const uint32_t n_obs = 1;

    float obs_data[obs.size()];
    std::transform(obs.begin(), obs.end(), obs_data, [](double d) { return static_cast<float>(d);});

    uint32_t source_data[source.size()];
    std::transform(source.begin(), source.end(), source_data, [](double d) { return static_cast<uint32_t>(d);});

    uint32_t target_data[source.size()];
    std::transform(target.begin(), target.end(), target_data, [](double d) { return static_cast<uint32_t>(d);});

    float edge_weight_data[source.size()];
    std::transform(edge_weight.begin(), edge_weight.end(), edge_weight_data, [](double d) { return static_cast<float>(d);});

    // float solution [n_ver * n_obs];
    std::vector<float> solution(n_ver * n_obs, 0.0f);
    std::vector<float> node_weight(n_ver, 1.0f);
    std::vector<uint32_t> in_component(n_ver,0);
    std::vector< std::vector<uint32_t> > components(1,std::vector<uint32_t>(1,0.f));

    float lambda = 2.0f;
    float mode = 2.0f;
    float speed = 2.0f;

    CP::cut_pursuit<float>(n_ver, n_edg, n_obs, obs_data, source_data, target_data, edge_weight_data, &node_weight[0]
             , solution.data(), in_component, components, lambda, mode, speed, 2.f);

    return "cutpursuit seg";
}

JLCXX_MODULE define_julia_module(jlcxx::Module& mod)
{
    mod.method("cut_pursuit", &cutpursuit_seg);
}
