#include <boost/config.hpp>
#include <iostream>                      // for std::cout
#include <utility>                       // for std::pair
#include <algorithm>                     // for std::for_each
#include <boost/utility.hpp>             // for boost::tie

#include <boost/graph/graph_traits.hpp>  // for boost::graph_traits
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/properties.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <cstring> // std::memset()
#include "main.hpp"

#include <iostream>




int main(int ac, char** av)
{
    using namespace std;

    namespace po = boost::program_options;
    std::locale::global(std::locale(""));


    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input", po::value<std::string>()->default_value("arlington.osm"), "input file")
        ("port", po::value<int>()->default_value(1883), "mqtt port")
        ("pwm1topic", po::value<std::string>()->default_value("pwm1"), "mqtt publish topic for pwm1 output")
        ("velocity", po::value<double>()->default_value(.01), "rotary encoder velocity accelerator")
        ("wdt", po::value<double>()->default_value(10), "pwm to zero after this many seconds of no PWM messages")
        ("verbose", "report extra data to stdout")
        ("csv", "csv readout to stdout")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }
    bool verbose = false;
    if (vm.count("verbose")) {
        std::cout << "verbose set\n";
        verbose = true;
    }

    std::string input_filename = vm["input"].as<std::string>();

    // Parse the entire document in one go:
    osm_sax_parser parser;
    auto return_code = EXIT_SUCCESS;
    try
    {
        parser.set_substitute_entities(true);
        parser.parse_file(input_filename);
    }
    catch (const xmlpp::exception& ex)
    {
        std::cerr << "libxml++ exception: " << ex.what() << std::endl;
        return_code = EXIT_FAILURE;
    }

    std::cout << "ways: ";
    std::cout << parser.ways.size();
    //// mark nodes that are on roads
    for (auto& way : parser.ways)
    {
        if (way.second.tags.count("highway")) {
            if (way.second.tags["highway"] == "residential") {
                parser.roads.push_back(way.second);
                for (auto& node : way.second.nodes) {
                    parser.nodes[node.id].roads++;
                    parser.nodes[node.id].norm_num = 0;
                }
            }
        }
    }

     // loop through roads and simplify them into the least number of vertices 
    uint32_t node_id_counter = 0;

       
    for (auto way : parser.roads)
    {
        auto& road = way.get();
        //std::cout << "\nid: " << road.id;
        //std::cout << " name: " << road.get_tag_string_if_exists("name");

        node last_node{};
        long double meters = 0.0;
        long double distance = 0.0;
        int n = 0;
        std::vector<std::reference_wrapper<node>> simplified_nodes{};
        
        for (auto it = road.nodes.begin(); it != road.nodes.end(); it++) {
            auto& nd = *it;
            auto& parent_node = parser.nodes[nd.id];
            if (last_node.lat != 0.0) {
                auto meters = (parent_node - last_node) * 1000.0;
                distance += meters;
                //std::cout << "\n\t" << n << ", " << last_node.num;
                //std::cout << " : " << parent_node.num;
                //std::cout << ", " << std::setprecision(4) << distance;
            }
            if (parent_node.roads >= 2 || (road.nodes.end() == std::next(it)) || n==0) {
                //std::cout << ", keep edge extent, " << parent_node.num;
                parent_node.used_by_road = true;
                parent_node.distance = distance;
                simplified_nodes.push_back(parent_node);
                if(!parent_node.norm_num) 
                    parent_node.norm_num = node_id_counter++;
                distance = 0.0;
            }
            else {
                //std::cout << ", dump " << parent_node.num;
            }
            last_node = parent_node;
            n++;
        }
        road.v_node_refs = simplified_nodes;
    }

    std::vector<std::reference_wrapper<node>> nodes_normal_vec{};
    
    for (auto& node : parser.nodes) {
        if (node.second.used_by_road)
            nodes_normal_vec.push_back(std::ref(node.second));
    }
    
    std::sort(nodes_normal_vec.begin(), nodes_normal_vec.end(), [](node a, node b)
        {
            return a.norm_num < b.norm_num;
        }
    );

    //parser.print_road_list();
	// edge property storage
	//typedef boost::property<boost::vertex_distance_t, float,boost::property<boost::vertex_name_t, std::string> > VertexProperty;
	
    typedef boost::property<boost::edge_weight_t, long double> EdgeProperty;

    //https://www.boost.org/doc/libs/1_60_0/libs/graph/doc/using_adjacency_list.html#sec:custom-storage

    std::vector<std::pair<int, int>> edges{};
    boost::array<long double,1000> edge_lengths{};


    int n = 0;
    // map print list of roads
    for (auto way : parser.roads) {
        auto road = way.get();
        auto name = road.get_tag_string_if_exists("name");
        std::cout << std::endl << name;
        auto prev = road.v_node_refs[0];
        for (auto it = road.v_node_refs.begin(); it != road.v_node_refs.end(); it++) {
            
            if (it == road.v_node_refs.begin()) {
                prev = *it;
                continue;
            }
            auto curr = *it;
            std::cout << std::endl << prev.get().norm_num << ":" << curr.get().norm_num << ", " << curr.get().distance;
            edges.push_back(std::make_pair(prev.get().norm_num, curr.get().norm_num));
            edge_lengths[n++] = curr.get().distance;
            prev = curr;
        }
    }


   typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
		EdgeProperty> graph;
	graph g{ edges.begin(), edges.end(), edge_lengths.begin(), edges.size() };

	boost::array<int, 400> directions;
	boost::dijkstra_shortest_paths(g, 10,
		boost::predecessor_map(directions.begin()));


	//boost::array<edge_properties,7> props{ {4,3,4,4,5,1,3} };
    //std::vector<edge_properties> propsv{ {4},{3},{4},{4},{5},{1},{3} };
    /*
	graph g{ edges.begin(), edges.end(), edge_lengths.begin(), edges.size() };

	boost::array<int, 100> directions;
    boost::dijkstra_shortest_paths(g, 33,
        boost::predecessor_map(directions.begin()));
        */

	int p = 0;
    std::cout << std::endl;
	while (p != 10)
	{
		std::cout << p << " -> ";
		p = directions[p];
	}
	std::cout << p << '\n';
    
	std::cout << std::endl;
    p = 0;
	while (p != 10)
	{
		std::cout << nodes_normal_vec[p].get().print_trkpt() << " \n\t";
		p = directions[p];
	}
    std::cout << nodes_normal_vec[p].get().print_trkpt() << " \n\t";
    return 0;
}
