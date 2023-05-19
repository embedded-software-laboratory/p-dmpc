#include "mex.hpp"
#include "mexAdapter.hpp"

#include <map>

#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_scaled_lab_projector/ScaledLab.h>


using matlab::mex::ArgumentList;

// helpful: https://de.mathworks.com/help/matlab/matlab_external/matlab-to-c-data-type-mapping.html
//          https://de.mathworks.com/help/matlab/matlab-data-array.html

// This function retrieves a sequence of segments and connects them by using the shortest path (without lane changes) between them
class MexFunction : public matlab::mex::Function {
public:
    inline void operator()(ArgumentList outputs, ArgumentList inputs) {
        const std::string filename = matlab::engine::convertUTF16StringToUTF8String(inputs[0][0]);
        matlab::data::TypedArray<int64_t> lanelet2_segment_ids = std::move(inputs[1]);

        using namespace lanelet;

        // Read lanelet2 map
        LaneletMapPtr map = load(filename, projection::ScaledLabProjector{Origin({0,0})});
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
        std::vector<int> path_ids;

        for (int i=0; i<lanelet2_segment_ids.getNumberOfElements()-1; ++i){
            Optional<routing::LaneletPath> path_part_opt = routing_graph->shortestPath(*map->laneletLayer.find(lanelet2_segment_ids[i]),
                                                                                       *map->laneletLayer.find(lanelet2_segment_ids[i+1]),
                                                                                       1, false);
            assert(path_part_opt.has_value());
            routing::LaneletPath path_part = path_part_opt.get();
            for (auto lanelet_it = path_part.begin(); lanelet_it!=--path_part.end(); ++lanelet_it){
                path_ids.push_back(id_to_index.find(lanelet_it->id())->second + 1); // Get index of current lanelet
            }
        }
        path_ids.push_back(id_to_index.find(lanelet2_segment_ids[lanelet2_segment_ids.getNumberOfElements()-1])->second + 1);

        matlab::data::TypedArray<int> matlab_path_ids = factory.createArray<int>({1,path_ids.size()});
        // Copy vector into typedarray
        for (int i=0; i<path_ids.size(); ++i){
            matlab_path_ids[0][i] = path_ids.at(i);
        }
        outputs[0] = matlab_path_ids;
    }

private:
    // Create MATLAB data array factory
    matlab::data::ArrayFactory factory;

    // Maps from the lanelet ids to the matlab ids (indices of rows/columns)
    std::map<lanelet::Id, size_t> id_to_index;


};