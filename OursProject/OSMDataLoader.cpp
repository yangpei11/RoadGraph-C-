#include "OSMDataLoader.h"
#include "RoadGraph.h"
#include "RoadEdge.h"
#include "RoadGraphInterface.h"
#include "VPMath.h"
#include "VUID.h"

#include "boost/asio.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include "boost/foreach.hpp"
#include "boost/lexical_cast.hpp"

#include <vector>
#include <cstdint>
#include <unordered_map>
#include <set>

using namespace std;
using namespace boost::property_tree;
using namespace VPE;

#include "RoadNode.h"

std::unordered_map<std::string, RoadType> OSMDataLoader::type_map = {
    {"motorway", MOTORWAY},
    {"trunk", TRUNK},
    {"primary", PRIMARY},
    {"secondary", SECONDARY},
    {"tertiary", TERTIARY},
    {"unclassified", QUATERNARY},
    {"residential", RESIDENTIAL},
    {"motorway_link", MOTORWAY_LINK},
    {"trunk_link", TRUNK_LINK},
    {"primary_link", PRIMARY_LINK},
    {"secondary_link", SECONDARY_LINK},
    {"tertiary_link", TERTIARY_LINK}};

bool OSMDataLoader::LoadOSM(const std::string& filename, RoadGraph & road_graph)
{
	ptree pt;
	try
	{
		read_xml(filename, pt);
		double minLon = DBL_MAX;
		double minLat = DBL_MAX;
		double maxLon = 0;
		double maxLat = 0;

		BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("osm"))
		{
			if (v.first == "node")
			{
                RoadNode *node = new RoadNode();
				node->id = v.second.get<int64_t>("<xmlattr>.id");
				node->lon = v.second.get<double>("<xmlattr>.lon");
				node->lat = v.second.get<double>("<xmlattr>.lat");
                node->alt      = 0;// 默认alt = 0

				minLon = std::min(node->lon, minLon);
                maxLon = std::max(node->lon, maxLon);
                minLat = std::min(node->lat, minLat);
                maxLat = std::max(node->lat, maxLat);
 
                road_graph.AddRoadNode(node);
			}
			else if (v.first == "way")
			{
                
                RoadEdge *edge = nullptr;
                int64_t   id   = v.second.get<int64_t>("<xmlattr>.id");
                for (auto iter2 = v.second.find("tag"); iter2 != v.second.not_found(); ++iter2) {
                    string key_name = iter2->second.get<string>("<xmlattr>.k");
                    if (key_name == "highway") {//目前暂时只支持highway字段                        
                        string key_value = iter2->second.get<string>("<xmlattr>.v");
						auto iter = type_map.find(key_value);
                        if (iter != type_map.end()) {                            
                            edge        = new RoadEdge(id, &road_graph);
                            edge->type_ = iter->second;
                        }
                        break;
                    } else if (key_name == "railway") {
                        edge = new RoadEdge(id, &road_graph);
                        edge->is_railway_ = true;
                        break;
                    }
                }

                if (edge) {
                    for (auto it = v.second.begin(); it != v.second.end(); ++it) {
                        if (it->first == "nd") {
                            int64_t   nd_id = it->second.get<int64_t>("<xmlattr>.ref");
                            RoadNode *node  = road_graph.GetRoadNode(nd_id);
                            if (node)
                                edge->AddNode(node);
                        } else if (it->first == "tag") {
                            string key_name, key_value;

                            key_name  = it->second.get<string>("<xmlattr>.k");
                            key_value = it->second.get<string>("<xmlattr>.v");

                            if (key_name == "max_speed") {
                                edge->max_speed_ = atoi(key_value.c_str());
                            } else if (key_name == "lanes") {
                                edge->lane_nums_ = atoi(key_value.c_str());
                            } else if (key_name == "oneway") {
                                edge->is_oneway_ = true;
                            }
                            // else if ((key_name == "junction" && key_value == "roundabout"))
                            //{
                            //	is_roundabout = true;
                            //}
                            // else if (key_name == "aeroway")
                            //{
                            //	is_aeroway = true;
                            //}

                            else if (key_name == "bridge" && key_value == "yes") { //桥
                                edge->is_bridge_ = true;
                            } else if (key_name == "tunnel" && key_value == "yes") { //隧道
                                edge->is_tunnel_ = true;
                            } else if (key_name == "layer") { //层级
                                edge->layer = atoi(key_value.c_str());
                                if (edge->layer != 0)
                                    edge->is_bridge_ = true;
                            }
                        }
                    }
                    if (edge->nodes_.size() > 1) //是有效道路
                    {
                        edge->SetupEdge(); //设置属性
                        road_graph.AddRoadEdge(edge);
                    } else {
                        for (auto node: edge->nodes_)
                        {
                            auto iter = std::find(node->edges.begin(), node->edges.end(), edge);
                            if (iter != node->edges.end())
                                node->edges.erase(iter);
                        }
                        delete edge;
                    }
                }                				
			}
		}
		road_graph.SetCenter((minLon + maxLon)/2.0, (minLat + maxLat)/2.0);
	}
	catch (...)
	{
		return false;
	}

	return true;
}