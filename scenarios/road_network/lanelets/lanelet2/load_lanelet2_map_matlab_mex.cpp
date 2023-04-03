#include "mex.hpp"
#include "mexAdapter.hpp"

#include <map>

#include <lanelet2_io/Io.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_scaled_lab_projector/ScaledLab.h>


using matlab::mex::ArgumentList;

// helpful: https://de.mathworks.com/help/matlab/matlab_external/matlab-to-c-data-type-mapping.html
//          https://de.mathworks.com/help/matlab/matlab-data-array.html

// Note: This algorithm probably doesn't work for maps with bidrectional lanelets (see notes below)
class MexFunction : public matlab::mex::Function {
public:
    inline void operator()(ArgumentList outputs, ArgumentList inputs) {
        // inputs[0]: filename as matlab::data::CharArray
        // const std::string filename = std::string(inputs[0][0]);
        // matlab::data::MATLABString filename_m = inputs[0][0];
        const std::string filename = matlab::engine::convertUTF16StringToUTF8String(inputs[0][0]);

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
        

////////////////////////////////////// Read Lanelets //////////////////////////////////////
        matlab::data::CellArray lanelets_mat = factory.createCellArray({1,map->laneletLayer.size()});

        // // auto lanelets_mat_it = lanelets_mat.begin();
        // {
        //     auto lanelet_it = map->laneletLayer.begin();
        //     for (size_t i=0; lanelet_it != map->laneletLayer.end(); ++lanelet_it, ++i){
        //         size_t n_rows = lanelet_it->leftBound2d().size();
        //         assert(n_rows == lanelet_it->rightBound2d().size());
        //         // We assume that leftBound, rightBound and centerline have the same number of points. Valid hypothesis?
        // 
        //         lanelets_mat[0][i] = factory.createArray<double>({n_rows,6});
        // 
        //         auto point_left_it  = lanelet_it->leftBound2d().begin();
        //         auto point_right_it = lanelet_it->rightBound2d().begin();
        //         auto point_center_it = lanelet_it->centerline().begin();
        //         auto ref_to_lanelet_mat = static_cast< matlab::data::Reference<matlab::data::TypedArray<double>> >(lanelets_mat[i]);
        //         auto mat_it = ref_to_lanelet_mat.begin();
        //         for (int j=0 ; point_left_it != lanelet_it->leftBound2d().end()
        //                         && point_right_it != lanelet_it->rightBound2d().end()
        //                         && mat_it != ref_to_lanelet_mat.end(); ++point_left_it, ++point_right_it, ++mat_it, ++point_center_it){
        //             *(mat_it + 0*n_rows) = point_left_it ->x();
        //             *(mat_it + 1*n_rows) = point_left_it ->y();
        //             *(mat_it + 2*n_rows) = point_right_it->x();
        //             *(mat_it + 3*n_rows) = point_right_it->y();
        //             *(mat_it + 4*n_rows) = point_center_it->x();
        //             *(mat_it + 5*n_rows) = point_center_it->y();
        //         }
        //         assert(point_left_it == lanelet_it->leftBound2d().end());
        //         assert(point_right_it == lanelet_it->rightBound2d().end());
        //         assert(point_center_it == lanelet_it->canterline().end());
        //         assert(mat_it == ref_to_lanelet_mat.end());
        //     }
        // }
        outputs[0] = lanelets_mat;


////////////////////////////////////// Find relation of lanelets //////////////////////////////////////
        size_t n_lanelets = map->laneletLayer.size();
        // Create adjacency matrix
        matlab::data::TypedArray<bool> adjacency_lanelets = factory.createArray<bool>({n_lanelets,n_lanelets});
        std::fill(adjacency_lanelets.begin(), adjacency_lanelets.end(), false); // Init all entries with false

        // Create relationships matrix
//         matlab::data::TypedArray<matlab::data::StructArray>
        auto lanelet_relationships = factory.createStructArray({n_lanelets,n_lanelets},{"type","point"});

        {
            // Iterate through all lanelets once
            for (auto lanelet_it = map->laneletLayer.begin(); lanelet_it!=map->laneletLayer.end(); ++lanelet_it){
                size_t i = id_to_index.find(lanelet_it->id())->second; // Get index of current lanelet

                // Diagonal
                adjacency_lanelets[i][i] = true;

                auto f_set_to_true = [&](ConstLanelet &lanelet){
                    size_t j = id_to_index.find(lanelet.id())->second;
                    adjacency_lanelets[i][j] = true;
                    adjacency_lanelets[j][i] = true;
                };

                auto f_set_relationship = [&](ConstLanelet &lanelet, std::string relation){
                    size_t j = id_to_index.find(lanelet.id())->second;
                    if (i==j) relation = "same";

                    lanelet_relationships[i][j]["type"] = factory.createCharArray(relation);
                    lanelet_relationships[j][i]["type"] = factory.createCharArray(relation);
                };

                // Set all lanelets besides to adjacent
                for (auto lanelet_besides : routing_graph->besides(*lanelet_it)){
                    f_set_to_true(lanelet_besides);
                    if (lanelet_besides != *lanelet_it){
                        f_set_relationship(lanelet_besides, "left/right");
                    } else {
                        f_set_relationship(lanelet_besides, "same");
                    }
                }

                for (auto lanelet_in_front : routing_graph->following(*lanelet_it)){
                    // Set all following lanelets as adjacent
                    f_set_to_true(lanelet_in_front);
                    f_set_relationship(lanelet_in_front, "longitudinal");

                    // Set all lanelets adjacent which are besides of the following
                    for (auto lanelet_besides : routing_graph->besides(lanelet_in_front)){
                        f_set_to_true(lanelet_besides);
                        f_set_relationship(lanelet_besides, "longitudinal");
                    }
                    // Set all lanelets adjacent which are predecessors of the following, i.e., merging
                    for (auto lanelet_behind : routing_graph->previous(lanelet_in_front)){
                        // Note: we assume here, that the lanelets are going only in one direction
                        f_set_to_true(lanelet_behind);
                        f_set_relationship(lanelet_behind, "merging");
                    }
                }

                for (auto lanelet_behind : routing_graph->previous(*lanelet_it)){
                    // Set all previous lanelets as adjacent
                    f_set_to_true(lanelet_behind);
                    f_set_relationship(lanelet_behind, "longitudinal");

                    // Set all lanelets adjacent which are besides of the previous
                    for (auto lanelet_besides : routing_graph->besides(lanelet_behind)){
                        f_set_to_true(lanelet_besides);
                        f_set_relationship(lanelet_besides, "longitudinal");
                    }
                    // Set all lanelets adjacent which are successors of the previous, i.e., forking
                    for (auto lanelet_in_front : routing_graph->following(lanelet_behind)){
                        // Note: we assume here, that the lanelets are going only in one direction
                        f_set_to_true(lanelet_in_front);
                        f_set_relationship(lanelet_in_front, "forking");
                    }
                }
            }
        }
        outputs[1] = adjacency_lanelets;
        // laneletrelationships are added to output later (see below)

////////////////////////////////////// Find intersecting lanelets //////////////////////////////////////
        matlab::data::TypedArray<double> intersection_lanelets = factory.createArray<double>({1,n_lanelets});
        std::fill(intersection_lanelets.begin(), intersection_lanelets.end(), 0);

        {
            size_t current_column_intersection = 0;

            // Find out all lanelets which are part of an intersection
            for (auto lanelet1_it = map->laneletLayer.begin(); lanelet1_it!=map->laneletLayer.end(); ++lanelet1_it){
                bool is_adjacency_set = false;
                for (auto lanelet2_it = map->laneletLayer.begin(); lanelet2_it!=map->laneletLayer.end(); ++lanelet2_it){
                    if (lanelet1_it == lanelet2_it) continue;

                    auto lanelet1_index = id_to_index.find(lanelet1_it->id())->second;
                    auto lanelet2_index = id_to_index.find(lanelet2_it->id())->second;

                    if (geometry::overlaps2d(*lanelet1_it, *lanelet2_it)
                        && !adjacency_lanelets[lanelet1_index][lanelet2_index])
                    {
                        // Add lanelets which are intersecting but not adjacent
                        if (!is_adjacency_set){
                            intersection_lanelets[0][current_column_intersection] = lanelet1_index+1; // Index in matlab starts with 1
                            ++current_column_intersection;
                            is_adjacency_set = true;
                        }

                        // Set relationship
                        lanelet_relationships[lanelet1_index][lanelet2_index]["type"] = factory.createCharArray("crossing");
                        lanelet_relationships[lanelet2_index][lanelet1_index]["type"] = factory.createCharArray("crossing");
                    }
                }
            }
        }
        outputs[2] = intersection_lanelets;
        outputs[4] = lanelet_relationships;

////////////////////////////////////// For Debugging //////////////////////////////////////
        matlab::data::TypedArray<int> ids = factory.createArray<int>({1,n_lanelets});

        {
            for (auto lanelet_it = map->laneletLayer.begin(); lanelet_it!=map->laneletLayer.end(); ++lanelet_it){
                auto lanelet_index = id_to_index.find(lanelet_it->id())->second;
                ids[0][lanelet_index] = lanelet_it->id();
            }
        }

        outputs[3] = ids;

////////////////////////////////////// Create raw data //////////////////////////////////////
        auto raw_data = factory.createStructArray({1,n_lanelets},{"idAttribute","leftBound","rightBound","predecessor",
                                                                  "successor","adjacentLeft","laneletType","adjacentRight"});

        {
            for (auto lanelet_it = map->laneletLayer.begin(); lanelet_it!=map->laneletLayer.end(); ++lanelet_it){
                size_t lanelet_index = id_to_index.find(lanelet_it->id())->second;
                raw_data[0][lanelet_index]["idAttribute"] = factory.createScalar<size_t>(lanelet_index+1);

                // Create left and right bound
                unsigned long fill_up_to = std::max(lanelet_it->leftBound().size(), lanelet_it->rightBound().size());
                raw_data[0][lanelet_index]["leftBound"] = create_raw_data_bound(lanelet_it->leftBound(), fill_up_to);
                raw_data[0][lanelet_index]["rightBound"] = create_raw_data_bound(lanelet_it->rightBound(), fill_up_to);

                // Write ids of to value "refAttribute"
                raw_data[0][lanelet_index]["predecessor"] = create_raw_data_index_list(routing_graph->previous(*lanelet_it));
                raw_data[0][lanelet_index]["successor"] = create_raw_data_index_list(routing_graph->following(*lanelet_it));

                // Get location as laneletType (typically "urban")
                raw_data[0][lanelet_index]["laneletType"] = factory.createCharArray(lanelet_it->attributes().at(AttributeName::Location).value());

                if (routing_graph->leftRelations(*lanelet_it).size()>0){
                    auto structArray = factory.createStructArray({1,routing_graph->leftRelations(*lanelet_it).size()},{"drivingDirAttribute","refAttribute"});
                    int i=0;
                    for (auto lanelet_relation : routing_graph->leftRelations(*lanelet_it)){
                        if (lanelet_relation.relationType == routing::RelationType::AdjacentLeft){
                            // Adjacent but not routable, i.e., other direction
                            structArray[0][i]["drivingDirAttribute"] = factory.createCharArray("opposite");
                        } else if (lanelet_relation.relationType == routing::RelationType::Left){
                            // Adjacent and routable, i.e., same direction
                            structArray[0][i]["drivingDirAttribute"] = factory.createCharArray("same");
                        } else {
                            structArray[0][i]["drivingDirAttribute"] = factory.createCharArray("unknown");
                        }
                        structArray[0][i]["refAttribute"] = factory.createScalar<int>(id_to_index.find(lanelet_relation.lanelet.id())->second + 1);
                        ++i;
                    }
                    raw_data[0][lanelet_index]["adjacentLeft"] = matlab::data::StructArray(structArray);
                }

                if (routing_graph->rightRelations(*lanelet_it).size()>0){
                    auto structArray = factory.createStructArray({1,routing_graph->rightRelations(*lanelet_it).size()},{"drivingDirAttribute","refAttribute"});
                    int i=0;
                    for (auto lanelet_relation : routing_graph->rightRelations(*lanelet_it)){
                        if (lanelet_relation.relationType == routing::RelationType::AdjacentRight){
                            // Adjacent but not routable, i.e., other direction
                            structArray[0][i]["drivingDirAttribute"] = factory.createCharArray("opposite");
                        } else if (lanelet_relation.relationType == routing::RelationType::Right){
                            // Adjacent and routable, i.e., same direction
                            structArray[0][i]["drivingDirAttribute"] = factory.createCharArray("same");
                        } else {
                            structArray[0][i]["drivingDirAttribute"] = factory.createCharArray("unknown");
                        }
                        structArray[0][i]["refAttribute"] = factory.createScalar<int>(id_to_index.find(lanelet_relation.lanelet.id())->second + 1);
                        ++i;
                    }
                    raw_data[0][lanelet_index]["adjacentRight"] = matlab::data::StructArray(structArray);
                }
            }
        }

        outputs[5] = raw_data;
    }

private:
    // Create MATLAB data array factory
    matlab::data::ArrayFactory factory;

    // Maps from the lanelet ids to the matlab ids (indices of rows/columns)
    std::map<lanelet::Id, size_t> id_to_index;

    matlab::data::StructArray create_raw_data_bound(const lanelet::LineString3d & lanelet_bound, unsigned long fill_up_to){
        using namespace lanelet;
        auto raw_data_bound = factory.createStructArray({1,1},{"point","lineMarking"});
        std::string line_marking;
        if (lanelet_bound.attributes().at(AttributeName::Type) == AttributeValueString::Virtual){
            line_marking = "dashed"; // Mark virtual ones as dashed since the matlab code only cannot handle virtual
        } else if (lanelet_bound.attributes().at(AttributeName::Type) == AttributeValueString::RoadBorder){
            line_marking = lanelet_bound.attributes().at(AttributeName::Type).value();
        } else if (lanelet_bound.attributes().find(AttributeName::Subtype) == lanelet_bound.attributes().end()){ // Not found
            line_marking = "unknown";
        } else {
            line_marking = lanelet_bound.attributes().at(AttributeName::Subtype).value();
        }
        raw_data_bound[0][0]["lineMarking"] = factory.createCharArray(line_marking);

        auto points = factory.createStructArray({1,fill_up_to},{"x","y"});
        size_t i = 0;
        for (auto point : lanelet_bound){
            points[0][i]["x"] = factory.createScalar<double>(point.x());
            points[0][i]["y"] = factory.createScalar<double>(point.y());
            ++i;
        }
        // If size< fill_up_to, i.e., the bound of one side has less elements than the one from the other side
        // append the last point multiple times
        for (int j=i; j<fill_up_to; ++j){
            points[0][j]["x"] = factory.createScalar<double>( (--lanelet_bound.end())->x() );
            points[0][j]["y"] = factory.createScalar<double>( (--lanelet_bound.end())->y() );
        }

        raw_data_bound[0][0]["point"] = matlab::data::StructArray(points);
        return raw_data_bound;
    }

    matlab::data::StructArray create_raw_data_index_list(const lanelet::ConstLanelets lanelets){
        // Used to create an index_list from a given set of lanelets (used for list of successors and predecessors)
        auto structArray = factory.createStructArray({1,lanelets.size()},{"refAttribute"});
        int i=0;
        for (auto lanelet : lanelets){
            structArray[0][i]["refAttribute"] = factory.createScalar<int>(id_to_index.find(lanelet.id())->second + 1);
            ++i;
        }
        return structArray;
    }
};