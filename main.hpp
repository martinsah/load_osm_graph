#pragma once
#include <libxml++/libxml++.h>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include <glibmm/convert.h> //For Glib::ConvertError
#include <iostream>
#include <cmath>

const long double pi = 3.14159265359;

struct tag {
    std::string k;
    std::string v;
}; 

struct node {
    std::string id;
    uint64_t num;
    long double lat=0.0;
    long double lon=0.0;
    int roads = 0;
    long double distance = 0.0;
    long double operator-(node const& other) {
        long double dlat = (lat - other.lat)* pi / 180.0;
        long double dlon = (lon - other.lon)* pi / 180.0;
        const long double R = 6373; // km radius of earth

        long double a = pow((sin(dlat / 2.0)),2) + cos(lat*pi/180.0) * cos(other.lat*pi/180.0) * pow((sin(dlon / 2.0)), 2);
        long double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        long double d = R * c;
        return d;
    }
    void print() {
        std::cout << std::setprecision(10) << num << ": " << lat << ", " << lon;
    }
    std::string print_wpt(std::string name="noname") {
        std::stringstream ss;
        ss << "<wpt lat=\"" << std::setprecision(10) << lat << "\" lon=\"" << lon << "\"><name>" << name << "</name></wpt>";
        return ss.str();
    }
};

enum class way_types {
    road,
    building,
    parking
};

struct way {
    std::string id;
    std::string name;
    std::vector<node> nodes;
    std::unordered_map<std::string, std::string>tags;

    std::string get_tag_string_if_exists(std::string key, std::string noname = "<noname>")
    {
        if (tags.count(key))
            return(tags[key]);
        else
            return(noname);
    }
};

class osm_sax_parser : public xmlpp::SaxParser
{
public:
    osm_sax_parser();
    ~osm_sax_parser() override;

    bool way_tag;
    enum osm_states {
        s_top,
        s_node,
        s_way,
        s_tag
    };

    uint64_t node_count;
    std::unordered_map<std::string, node> nodes;
    std::unordered_map<std::string, way> ways;
    //std::unordered_map<std::string, way&> roads;
    std::vector<std::reference_wrapper<way>> roads;
    osm_states osm_state;
    
    way new_way{};

protected:
    //overrides:
    void on_start_document() override;
    void on_end_document() override;
    void on_start_element(const Glib::ustring& name, const AttributeList& properties) override;
    void on_end_element(const Glib::ustring& name) override;
    void on_characters(const Glib::ustring& characters) override;
    void on_comment(const Glib::ustring& text) override;
    void on_warning(const Glib::ustring& text) override;
    void on_error(const Glib::ustring& text) override;
    void on_fatal_error(const Glib::ustring& text) override;
};

void osm_sax_parser::on_characters(const Glib::ustring& text)
{/*
    try
    {
        std::cout << "on_characters(): " << text << std::endl;
    }
    catch (const Glib::ConvertError& ex)
    {
        std::cerr << "osm_sax_parser::on_characters(): Exception caught while converting text for std::cout: " << ex.what() << std::endl;
    }
    */
}

void osm_sax_parser::on_comment(const Glib::ustring& text)
{
    try
    {
        std::cout << "on_comment(): " << text << std::endl;
    }
    catch (const Glib::ConvertError& ex)
    {
        std::cerr << "osm_sax_parser::on_comment(): Exception caught while converting text for std::cout: " << ex.what() << std::endl;
    }
}

void osm_sax_parser::on_warning(const Glib::ustring& text)
{
    try
    {
        std::cout << "on_warning(): " << text << std::endl;
    }
    catch (const Glib::ConvertError& ex)
    {
        std::cerr << "osm_sax_parser::on_warning(): Exception caught while converting text for std::cout: " << ex.what() << std::endl;
    }
}

void osm_sax_parser::on_error(const Glib::ustring& text)
{
    try
    {
        std::cout << "on_error(): " << text << std::endl;
    }
    catch (const Glib::ConvertError& ex)
    {
        std::cerr << "osm_sax_parser::on_error(): Exception caught while converting text for std::cout: " << ex.what() << std::endl;
    }
}

void osm_sax_parser::on_fatal_error(const Glib::ustring& text)
{
    try
    {
        std::cout << "on_fatal_error(): " << text << std::endl;
    }
    catch (const Glib::ConvertError& ex)
    {
        std::cerr << "osm_sax_parser::on_characters(): Exception caught while converting value for std::cout: " << ex.what() << std::endl;
    }
}

osm_sax_parser::osm_sax_parser() : xmlpp::SaxParser()
{
    node_count = 0;
}

osm_sax_parser::~osm_sax_parser()
{
}

void osm_sax_parser::on_start_document()
{
    std::cout << "on_start_document()" << std::endl;
}

void osm_sax_parser::on_end_document()
{
    std::cout << "on_end_document()" << std::endl;
}

void osm_sax_parser::on_start_element(const Glib::ustring& name,
    const AttributeList& attributes)
{
    //std::cout << "node name=" << name << std::endl;
    std::string name_str{ name };
    // parser states
    if (name_str == "node") {
        osm_state = s_node;
        node newnode{};
        // Print attributes:
        for (const auto& attr_pair : attributes)
        {
            std::string name, value;
            try
            {
                //std::cout << "  Attribute name=" << attr_pair.name << std::endl;
                name = attr_pair.name;
            }
            catch (const Glib::ConvertError& ex)
            {
                std::cerr << "osm_sax_parser::on_start_element(): Exception caught while converting name for std::cout: " << ex.what() << std::endl;
            }

            try
            {
                //std::cout << "    , value= " << attr_pair.value << std::endl;
                value = attr_pair.value;
            }
            catch (const Glib::ConvertError& ex)
            {
                std::cerr << "osm_sax_parser::on_start_element(): Exception caught while converting value for std::cout: " << ex.what() << std::endl;
            }
            if (name == "id")
                newnode.id = value;
            if (name == "lat")
                newnode.lat = std::stod(value);
            if (name == "lon")
                newnode.lon = std::stod(value);

        }
        //std::cout << "node[" << node_count << "]: " << newnode.id << " " << newnode.lat << ", " << newnode.lon << std::endl;
        newnode.num = node_count++;
        nodes[newnode.id] = newnode;
        return;
    }
    if (name_str == "way") {
        osm_state = s_way;
        // Print attributes:
        for (const auto& attr_pair : attributes)
        {
            std::string name, value;
            try
            {
                //std::cout << "  Attribute name=" << attr_pair.name << std::endl;
                name = attr_pair.name;
				value = attr_pair.value;
            }
            catch (const Glib::ConvertError& ex)
            {
                std::cerr << "osm_sax_parser::on_start_element(): Exception caught while converting name for std::cout: " << ex.what() << std::endl;
            }

            if (name == "id") {
                new_way.id = value;
                break;
            }

        }
        return;
    }
    if (name == "nd") {
        node nd{};
        std::string name, value;
        for (const auto& attr_pair : attributes)
        {
            try {
                name = attr_pair.name;
                value = attr_pair.value;
            }
            catch (const Glib::ConvertError& ex)
            {
                std::cerr << "osm_sax_parser::on_start_element(): Exception caught while converting name for std::cout: " << ex.what() << std::endl;
            }
            if (name == "ref")
                nd.id = value;
        }

        new_way.nodes.push_back(nd);
        return;
    }
    if (name_str == "tag") {
        if (osm_state == s_way) {
            std::string name, value, k, v;
            for (const auto& attr_pair : attributes)
            {
                try {
                    name = attr_pair.name;
                    value = attr_pair.value;
                }
                catch (const Glib::ConvertError& ex)
                {
                    std::cerr << "osm_sax_parser::on_start_element(): Exception caught while converting name for std::cout: " << ex.what() << std::endl;
                }
                if (name == "k")
                    k = value;
                if (name == "v")
                    v = value;
            }
            new_way.tags[k] = v;
        }
        return;
    }
}

void osm_sax_parser::on_end_element(const Glib::ustring& name)
{
    if (name == "way") {
        osm_state = s_top;
        ways[new_way.id] = new_way;
        new_way = way{};
    }
}
