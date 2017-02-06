#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/datafacade/shared_memory_allocator.hpp"
#include "engine/edge_unpacker.hpp"
#include "engine/edge_unpacker.hpp"
#include "storage/shared_barrier.hpp"
#include "storage/shared_datatype.hpp"
#include "storage/shared_memory.hpp"
#include "util/log.hpp"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/interprocess/file_mapping.hpp>

namespace po = boost::program_options;

using namespace osrm;

namespace osrm {
namespace contractor {
std::ostream& operator<<(std::ostream& os, const QueryEdge::EdgeData& data)
{
    return os
        << "id: " << data.id
        << ", weight " << data.weight
        << (data.shortcut ? ", shortcut" : "")
        << (data.forward ? ", forward" : "")
        << (data.backward ? ", backward" : "");
}
}
}


std::string get_color()
{
    int red = std::rand() % 256, green = std::rand() % 256, blue = std::rand() % 256;
    std::stringstream sstr;
    sstr << std::hex << std::setw(2) << "#" << red << green << blue;
    return sstr.str();
}

using Facade = engine::datafacade::ContiguousInternalMemoryDataFacade;
std::string to_json(const Facade& facade,
                    const std::vector<NodeID>& ids, const std::vector<EdgeWeight>& weights,
                    const std::vector<DatasourceID>& datasources)
{
    std::stringstream sstr;
    BOOST_ASSERT(ids.size() - 1 == weights.size());
    BOOST_ASSERT(ids.size() - 1 == datasources.size());

    sstr << std::setprecision(8);
    char sep = ' ';
    std::string color = get_color();
    for (std::size_t index = 0; index < ids.size() - 1; ++index)
    {
        auto from = facade.GetCoordinateOfNode(ids[index]);
        auto to = facade.GetCoordinateOfNode(ids[index + 1]);
        auto weight = weights[index];
        auto datasource = datasources[index];

        sstr << sep
             << "{\"type\":\"Feature\","
             << "\"properties\": {\"stroke\": \"" << color << "\", \"stroke-width\": 2, \"stroke-opacity\": 1, \"weight\": "<<weight<<", \"datasource\":"<<(int)datasource<<"},"
             << "\"geometry\":{\"type\":\"LineString\",\"coordinates\":"
             << "["
             << "[" << util::toFloating(from.lon) << ","<<util::toFloating(from.lat) << "],"
             << "[" << util::toFloating(to.lon) << ","<<util::toFloating(to.lat) << "]"
             << "]"
             << "}}";
        sep = ',';
    }
    for (std::size_t index = 0; index < ids.size(); ++index)
    {
        auto osmid = facade.GetOSMNodeIDOfNode(ids[index]);
        auto coord = facade.GetCoordinateOfNode(ids[index]);

        sstr << sep
             << "{\"type\":\"Feature\","
             << "\"properties\": {\"stroke\": \"" << color << "\", \"stroke-width\": 2, \"stroke-opacity\": 1, \"osmid\": "<<osmid<<"},"
             << "\"geometry\":{\"type\":\"Point\",\"coordinates\":"
             << "[" << util::toFloating(coord.lon) << ","<<util::toFloating(coord.lat) << "]"
             << "}}";
        sep = ',';
    }

    return sstr.str();
}

const auto mmap_file = [](const std::string &filename) {
    using boost::interprocess::file_mapping;
    using boost::interprocess::mapped_region;
    using boost::interprocess::read_only;

    try {
        const file_mapping mapping{filename.c_str(), read_only};
        mapped_region region{mapping, read_only};
        region.advise(mapped_region::advice_sequential);
        return region;
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Error while trying to mmap " + filename + ": " + e.what();
        throw;
    }
};

// Set to 1 byte alignment
#pragma pack(push, 1)
struct SegmentHeaderBlock
{
    std::uint32_t num_osm_nodes;
    OSMNodeID previous_osm_node_id;
};
#pragma pack(pop)
static_assert(sizeof(SegmentHeaderBlock) == 12, "SegmentHeaderBlock is not packed correctly");

#pragma pack(push, 1)
struct SegmentBlock
{
    OSMNodeID this_osm_node_id;
    double segment_length;
    EdgeWeight segment_weight;
};
#pragma pack(pop)

int main(int argc, char *argv[])
{
    util::LogPolicy::GetInstance().Unmute();

    po::options_description opts("Options");

    opts.add_options()
        ("help,h", "Show this help message")
        ("edge-based-nodes,n", po::value<std::vector<NodeID>>()->multitoken(), "description")
        ("edge-segment-lookup", po::value<EdgeID>(), "description");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, opts), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << opts;
        return 0;
    }

    char sep = ' ';
    std::string json;

    if (vm.count("edge-based-nodes"))
    {
        auto barrier = std::make_shared<storage::SharedBarrier>(boost::interprocess::open_only_t{});

        util::Log() << "Current shared data region " << barrier->GetRegion()
                    << " timestamp " << barrier->GetTimestamp() << "\n";

        auto facade = std::make_shared<engine::datafacade::ContiguousInternalMemoryDataFacade>(
                        std::make_unique<engine::datafacade::SharedMemoryAllocator>(barrier->GetRegion()));

        for (auto edge_based_node : vm["edge-based-nodes"].as<std::vector<NodeID>>())
        {
            std::cout << "Edge-based node " << edge_based_node << "\n";
            for (auto edge_based_edge : facade->GetAdjacentEdgeRange(edge_based_node))
            {
                auto edge_based_target = facade->GetTarget(edge_based_edge);
                auto edge_data = facade->GetEdgeData(edge_based_edge);
                std::cout << "    "
                          << "edge-based edge " << edge_based_edge << " -> " << edge_based_target
                          << " {" << edge_data << "}"
                          << "\n";

                std::vector<GeometryID> geometry_ids;
                if (edge_data.shortcut)
                {
                    std::array<NodeID, 2> path{{edge_based_node, edge_based_target}};
                    if (edge_data.backward) path = std::array<NodeID, 2>{{edge_based_target, edge_based_node}};
                    engine::UnpackCHPath(*facade, path.begin(), path.end(),
                                         [&facade, &geometry_ids](const std::pair<NodeID, NodeID> &/*edge*/,
                                                                  const contractor::QueryEdge::EdgeData & edge_data)
                                         {
                                             geometry_ids.emplace_back(facade->GetGeometryIndexForEdgeID(edge_data.id));
                                         });
                }
                else
                {
                    geometry_ids = std::vector<GeometryID>(1, facade->GetGeometryIndexForEdgeID(edge_data.id));
                }


                // check edge_based_edge or edge_data.id
                NodeID last_node_id = SPECIAL_NODEID;
                std::vector<NodeID> id_vector;
                std::vector<EdgeWeight> weight_vector;
                std::vector<DatasourceID> datasource_vector;
                for (auto geometry_index : geometry_ids)
                {
                    std::vector<NodeID> ids;
                    std::vector<EdgeWeight> weights;
                    std::vector<DatasourceID> datasources;

                    if (geometry_index.forward)
                    {
                        ids = facade->GetUncompressedForwardGeometry(geometry_index.id);
                        weights = facade->GetUncompressedForwardWeights(geometry_index.id);
                        datasources = facade->GetUncompressedForwardDatasources(geometry_index.id);
                    }
                    else
                    {
                        ids = facade->GetUncompressedReverseGeometry(geometry_index.id);
                        weights = facade->GetUncompressedReverseWeights(geometry_index.id);
                        datasources = facade->GetUncompressedReverseDatasources(geometry_index.id);
                    }

                    std::copy(ids.begin(), ids.end() - 1, std::back_inserter(id_vector));
                    std::copy(weights.begin(), weights.end(), std::back_inserter(weight_vector));
                    std::copy(datasources.begin(), datasources.end(), std::back_inserter(datasource_vector));
                    last_node_id = ids.back();
                }
                id_vector.push_back(last_node_id);

                int total_weight = 0;
                std::cout << "        node-based nodes:"; for (auto id : id_vector) std::cout << " " << id; std::cout << "\n";
                std::cout << "        OSM ids:"; for (auto id : id_vector) std::cout << " " << facade->GetOSMNodeIDOfNode(id); std::cout << "\n";
                std::cout << "        weights:"; for (auto weight : weight_vector) std::cout << " " << weight, total_weight += weight;
                std::cout << " = " << total_weight << "\n";
                std::cout << "        datasource:"; for (auto source : datasource_vector) std::cout << " " << (int)source; std::cout << "\n";


                for (std::size_t index = 0; index < weight_vector.size(); ++index)
                {
                    auto from = facade->GetOSMNodeIDOfNode(id_vector[index]);
                    auto to = facade->GetOSMNodeIDOfNode(id_vector[index + 1]);
                    auto weight = weight_vector[index];
                    std::cerr << from << "," << to << ",1," << weight << "\n";
                }

                // std::cout << id_vector.size() << "\n";
                // std::cout << weight_vector.size() << "\n";
                // std::cout << datasource_vector.size() << "\n";

                json += "\n";
                json += sep; sep = ',';
                json += to_json(*facade, id_vector, weight_vector, datasource_vector);
            }
        }

        std::cout << "{\"type\": \"FeatureCollection\",  \"features\": [" << json << "\n]}\n";
    }

    if (vm.count("edge-segment-lookup"))
    {
        auto edgeId = vm["edge-based-nodes"].as<EdgeID>();
        auto edge_segment_region = mmap_file("latest.osrm.edge_segment_lookup");
        auto edge_segment_byte_ptr = reinterpret_cast<const char *>(edge_segment_region.get_address());
        for (EdgeID id = 0; id < edgeId; ++id )
        {
            auto header = reinterpret_cast<const SegmentHeaderBlock *>(edge_segment_byte_ptr);
            edge_segment_byte_ptr += sizeof(SegmentHeaderBlock);
            edge_segment_byte_ptr += sizeof(SegmentBlock) * (header->num_osm_nodes - 1);
        }

        auto header = reinterpret_cast<const SegmentHeaderBlock *>(edge_segment_byte_ptr);
        edge_segment_byte_ptr += sizeof(SegmentHeaderBlock);
        std::cout << header->previous_osm_node_id;
        for (std::uint32_t i=0; i<header->num_osm_nodes - 1; ++i)
        {
            auto segmentblocks = reinterpret_cast<const SegmentBlock *>(edge_segment_byte_ptr);
            edge_segment_byte_ptr += sizeof(SegmentBlock) * (header->num_osm_nodes - 1);
            std::cout << " -> (length: " << segmentblocks->segment_length << ", " << segmentblocks->segment_weight << ") " << segmentblocks->this_osm_node_id << "\n";
        }
        std::cout << "\n";
    }

}
