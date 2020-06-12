#include <boost/config.hpp>
#include <iostream>                      // for std::cout
#include <utility>                       // for std::pair
#include <algorithm>                     // for std::for_each
#include <boost/utility.hpp>             // for boost::tie
#include <boost/graph/graph_traits.hpp>  // for boost::graph_traits
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/program_options.hpp>
#include <iostream>
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
        ("input", po::value<std::string>()->default_value("map.osm"), "input file")
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
                }
            }
        }
    }

     // loop through roads and simplify them into the least number of vertices 
    for (auto way : parser.roads)
    {
        auto& road = way.get();
        std::cout << "\nid: " << road.id;
        std::cout << " name: " << road.get_tag_string_if_exists("name");

        node last_node{};
        long double meters = 0.0;
        long double distance = 0.0;
        int n = 0;
        std::vector<node> simplified_nodes{};
        
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
                parent_node.distance = distance;
                simplified_nodes.push_back(parent_node);
                distance = 0.0;
            }
            else {
                //std::cout << ", dump " << parent_node.num;
            }
            last_node = parent_node;
            n++;
        }
        road.nodes = simplified_nodes;
    }

    // print list of roads
    for (auto way : parser.roads) {
        auto road = way.get();
        std::cout << std::endl << road.get_tag_string_if_exists("name");
        int n = 0;
        for (auto& nd : road.nodes) {
            std::cout << "\n\t" << n++ << ", " << nd.num << ", " << nd.distance;
        }
    }

    // XML print list of roads
#if 0
    for (auto way : parser.roads) {
        auto road = way.get();
        auto name = road.get_tag_string_if_exists("name");
        int n = 0;
        for (auto& nd : road.nodes) {
            std::cout << nd.print_wpt(name);
        }
    }
#endif
    // map print list of roads
    for (auto way : parser.roads) {
        auto road = way.get();
        auto name = road.get_tag_string_if_exists("name");
        std::cout << std::endl << name;
        node& prev = road.nodes[0];
        for (auto it = road.nodes.begin(); it != road.nodes.end(); it++) {
            
            if (it == road.nodes.begin()) {
                prev = *it;
                continue;
            }
            node& curr = *it;
            std::cout << std::endl << prev.num << ":" << curr.num << ", " << curr.distance;
            prev = curr;
        }
    }


    return 0;
}
