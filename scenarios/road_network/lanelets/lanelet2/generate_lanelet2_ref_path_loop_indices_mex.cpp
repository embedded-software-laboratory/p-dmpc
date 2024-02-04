#include "mex.hpp"
#include "mexAdapter.hpp"

#include <map>

#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_projection/CPM.h>


using matlab::mex::ArgumentList;

// helpful: https://de.mathworks.com/help/matlab/matlab_external/matlab-to-c-data-type-mapping.html
//          https://de.mathworks.com/help/matlab/matlab-data-array.html


class MexFunction : public matlab::mex::Function {
public:
    inline void operator()(ArgumentList outputs, ArgumentList inputs) {
        // inputs[0]: filename as matlab::data::CharArray
        // const std::string filename = std::string(inputs[0][0]);
        // matlab::data::MATLABString filename_m = inputs[0][0];
        // const std::string filename = std::string(filename_m);
        const std::string filename = matlab::engine::convertUTF16StringToUTF8String(inputs[0][0]);

        using namespace lanelet;

        // Read lanelet2 map
        LaneletMapPtr map = load(filename, projection::CpmProjector{Origin({0,0})});
        traffic_rules::TrafficRulesPtr trafficRules =
            traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
        routing::RoutingGraphUPtr routing_graph = routing::RoutingGraph::build(*map, *trafficRules);


//////////////////////////// Create map from lanelet id to index //////////////////////////
        // id_to_index stores for all lanelet Ids which index they have in our matlab arrays to uniquely indentify them
        {
            auto lanelet_it = map->laneletLayer.begin();
            for (size_t i=0; i<map->laneletLayer.size(); ++i, ++lanelet_it){
                id_to_index.emplace(lanelet_it->id(), i);
            }
        }

//////////////////////////// Create a random circle (provided as lanelet indices) //////////////////////////

        srand(42); // Use seed for random number generator

        // choose start lanelet randomly
        int random_lanelet = rand() % map->laneletLayer.size();
        auto start_lanelet_it = map->laneletLayer.begin();
        for (int i=0; i<random_lanelet; ++i) start_lanelet_it++;
        // now we have an iterator pointing to a random lanelet

        // find paths from the starting lanelet with at least 15 lanelets length
        routing::LaneletPaths paths = routing_graph->possiblePaths(*start_lanelet_it, 15, false);

        // Select one of the paths randomly
        int random_path = rand() % paths.size();
        auto path_part1 = paths.at(random_path);

        // In order to create a circle find the shortest path from the end of part1 to the starting lanelet
        Optional<routing::LaneletPath> path_part2_opt = routing_graph->shortestPath(path_part1.back(), path_part1.front(), 1, false);
        assert(path_part2_opt.has_value());
        routing::LaneletPath path_part2 = path_part2_opt.get();

        // Get lanelet IDs (matlab) of whole path
        matlab::data::TypedArray<int> path_ids = factory.createArray<int>({1,path_part1.size()+path_part2.size()-2}); // start and end of parts are the same
        int i = 0;
        for (auto lanelet_it = path_part1.begin(); lanelet_it!=--path_part1.end(); ++lanelet_it, ++i){
            path_ids[0][i] = id_to_index.find(lanelet_it->id())->second + 1; // Get index of current lanelet
        }
        for (auto lanelet_it = path_part2.begin(); lanelet_it!=--path_part2.end(); ++lanelet_it, ++i){
            path_ids[0][i] = id_to_index.find(lanelet_it->id())->second + 1; // Get index of current lanelet
        }
        outputs[0] = path_ids;
    }

private:
    // Create MATLAB data array factory
    matlab::data::ArrayFactory factory;

    // Maps from the lanelet ids to the matlab ids (indices of rows/columns)
    std::map<lanelet::Id, size_t> id_to_index;


};