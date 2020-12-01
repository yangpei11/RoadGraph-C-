#include "RoadGraphInterface.h"
#include "OSMDataLoader.h"
#include "VehicleSchedule.h"
#include "RoadUtils.h"
#include "RenderTree.h"
#include "RoadTextureCollection.h"
#include "TerrainExport/TerrainSdkInterface.h"

#include "Console.h"
#include "CNetWorkCacheManager.h"

#include "boost/asio.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include "boost/foreach.hpp"
#include "boost/thread/thread.hpp"
#include "boost/filesystem.hpp"


#include "VUID.h"

#include "GL/glew.h"
#include <cstdint>
#include <fstream>
#include <sstream>
#include <unordered_set>
#include "EffectManagerInterface.h"
#include "ViWoRoot.h"
#include "GLShaderUtils.h"
#include "VehicleManager.h"
#include "RenderEngine/NewGPUCullingModel.h"
#include "RoadConstructor.h"
#include "RoadRender.h"

using namespace std;
using namespace boost::property_tree;

RoadGraphInterface g_roadgraph_interface;

RoadGraphInterface::RoadGraphInterface()
	:street_light_color_(1.0, 1.0, 1.0),
	street_light_space_(50)
{
    //road_graph_program_ = OpenGLUtils::CompileAndLinkShaderProgram(OpenGLUtils::ShaderConfig().Vertex("roadgraph.glsl").Fragment("roadgraph.glsl"));
    //road_graph_night_program_ = OpenGLUtils::CompileAndLinkShaderProgram(OpenGLUtils::ShaderConfig().Vertex("roadgraph_night.glsl").Fragment("roadgraph_night.glsl"));
	NewGPUCullingModel::initShader();
    ViWoROOT::GetMainEntry()->Add(this);
}

RoadGraphInterface::~RoadGraphInterface()
{
    for_each(road_graphs_.begin(), road_graphs_.end(), [](pair<RoadObjectId, RoadGraph*> r){ delete r.second;});
    road_graphs_.clear();
    db_handle_map_.clear();

    ViWoROOT::GetMainEntry()->Erase(this);
}

bool RoadGraphInterface::Load(const std::string & data_file, std::function<void(int percent)> callback)
{
	return LoadConfig(0, data_file);
}

bool RoadGraphInterface::Clear()
{
	ClearAllRoadGraph();
	return true;
}

bool RoadGraphInterface::Release(const std::string & data_file)
{
	return true;
}

RoadObjectId RoadGraphInterface::CreateRoadGraph()
{
    uint64_t id = VPE::VUID::AllocVUID();

    RoadGraph* graph = new RoadGraph(id);
    road_graphs_.insert(make_pair(id, graph));

    return id;
}

bool RoadGraphInterface::DeleteGraph(RoadObjectId graph_id)
{
    auto graph_it = road_graphs_.find(graph_id);

    if (graph_it != road_graphs_.end())
    {
        try
        {
            delete graph_it->second;
        }
        catch(...)
        {
            return false;
        }

        return true;
    }

    return false;
}

RoadObjectId RoadGraphInterface::ImportOSM(const std::string& filename)
{
	auto graph_id = g_roadgraph_interface.CreateRoadGraph();
	RoadGraph* road_graph = GetRoadGraph(graph_id);
	if (!road_graph)
		return 0;

	if (OSMDataLoader::LoadOSM(filename, *road_graph))
	{
		
		//road_graph->Construct(true);
		RoadConstructor constructor;
		constructor.Construct(*road_graph);
		road_graph->Init();
		return graph_id;
	}
	else
		return 0;
}

void RoadGraphInterface::HideGraph(RoadObjectId graph_id)
{

}

void RoadGraphInterface::ShowGraph(RoadObjectId graph_id)
{

}

RoadGraph* RoadGraphInterface::GetRoadGraph(RoadObjectId graph_id)
{
    auto graph_it = road_graphs_.find(graph_id);

    if (graph_it != road_graphs_.end())
    {
        return graph_it->second;
    }

    return nullptr;
}

void RoadGraphInterface::Render(void *pOptions /* = 0 */, double _time /* = 0.0 */, float _deltatime /* = 0.0f */)
{
    VPE::dvec3 camera_pos(0.0), camera_dir(0.0);

    RoadUtils::GetCameraParameter(camera_pos, camera_dir);

	double lon, lat, height;
	g_coord.GlobalCoord2LongLat(VPE::value_ptr<double>(camera_pos), lon, lat, height);
	if (height > g_roadgraph_interface.GetLODParameter().start_show_road_height)
	{
		return;
	}

    // retrieve visible terrain blocks
	unordered_set<std::uint64_t> visible_blocks;
    ComputeVisibleTerrainBlocks(visible_blocks);

    glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_TEXTURE_BIT |
                 GL_POLYGON_BIT | GL_COLOR_BUFFER_BIT);

    glDisable(GL_BLEND);
	for (auto it = road_graphs_.begin(); it != road_graphs_.end(); ++it)
	{
        it->second->GetRender()->Render(pOptions, _time, _deltatime, camera_pos, visible_blocks);
	}
    glPopAttrib();

    VehicleManager::GetInstancePtr()->Render();
}

void RoadGraphInterface::ClearAllRoadGraph()
{
    for_each(road_graphs_.begin(), road_graphs_.end(), [](pair<RoadObjectId, RoadGraph*> r){ delete r.second;});

    road_graphs_.clear();
}

void RoadGraphInterface::ConstructGraphNode(RoadObjectId node_id)
{

}

void RoadGraphInterface::ConstructAll()
{
}

void RoadGraphInterface::StartVehicleSchedule()
{
    StaticRenderer::RenderTree::GetInstance();

    VehicleManager::GetInstancePtr()->Start();
}

bool RoadGraphInterface::LoadConfig(RoadObjectId graph_id, const std::string& filename)
{
    StaticRenderer::RenderTree::GetInstance();

    vector<string> input_filenames;
    string prefix = filename.substr(0, filename.find_last_of("/")+1);

    // vehicle manager
    int vehicle_num = 0;
    vector<ModelType> vehicle_models;

    try
    {
        ptree pt;
        read_xml(filename, pt);

        BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("RoadParameter"))
        {
            if (v.first == "InputFiles")
            {
                for (auto it = v.second.begin(); it != v.second.end(); ++it)
                {
                    input_filenames.push_back( prefix + it->second.get<string>("<xmlattr>.v") );
                }
            }
			else if (v.first == "StreetLightModel")
			{
				for (auto it = v.second.begin(); it != v.second.end(); ++it)
				{
					ModelType model;
					model.name = it->second.get<string>("<xmlattr>.v");
					model.scale = it->second.get<double>("<xmlattr>.scale");
					model.distance = it->second.get<double>("<xmlattr>.dis");
					street_light_model_.push_back(model);
				}
			}
			else if (v.first == "StreetLightSpace")
			{
				street_light_space_ = v.second.get<float>("<xmlattr>.v");
			}
			else if (v.first == "StreetLightColor")
			{
				string value = v.second.get<string>("<xmlattr>.v");
				istringstream ss(value);

				ss >> street_light_color_.x >> street_light_color_.y >> street_light_color_.z;
			}
            else if (v.first == "VehicleNum")
            {
                vehicle_num = v.second.get<int>("<xmlattr>.v");
            }
            else if (v.first == "StartShowVehicleDistance")
            {
                start_show_vehicle_distance_ = v.second.get<double>("<xmlattr>.v");
            }
            else if (v.first == "VehicleView")
            {
                control_camera_vehicle_num_ = v.second.get<unsigned int>("<xmlattr>.v");
            }
            else if (v.first == "VehicleOneWayBaseSpeed")
            {
                vehicle_oneway_base_speed_ = v.second.get<double>("<xmlattr>.v");
            }
            else if (v.first == "VehicleNormalWayBaseSpeed")
            {
                vehicle_normalway_bse_speed_ = v.second.get<double>("<xmlattr>.v");
            }
            else if (v.first == "VehicleModel")
            {
                for (auto it = v.second.begin(); it != v.second.end(); ++it)
                {
                    ModelType model;
                    model.name = it->second.get<string>("<xmlattr>.v");
                    model.scale = it->second.get<double>("<xmlattr>.scale");
                    vehicle_models.push_back(model);
                }
            }
            else if (v.first == "RoadTextures")
            {
                for (auto it = v.second.begin(); it != v.second.end(); ++it)
                {
                    string texture_filename = it->second.get<string>("<xmlattr>.v");
					string name = it->second.get<string>("<xmlattr>.name");
                    g_road_texture_collection.AddTexture(name, texture_filename);
					boost::filesystem::path filepath(texture_filename);
					std::string night_tex = (filepath.parent_path() / filepath.stem()).string() + "_n" + filepath.extension().string();
					g_road_texture_collection.AddTexture(name+"_n", night_tex);
                }
            }
            else if (v.first == "RoadLOD")
            {
                auto splitlevel_node =  v.second.get_child("SplitLvel");
                load_param_.split_level = splitlevel_node.get<int>("<xmlattr>.v");

                auto start_show_road_height_node = v.second.get_child("StartShowRoadHeight");
                load_param_.start_show_road_height = start_show_road_height_node.get<double>("<xmlattr>.v");

                auto show_marker_block_radius_node = v.second.get_child("BlockHalfWidth");
                load_param_.block_half_width = show_marker_block_radius_node.get<int>("<xmlattr>.v");

                auto show_road_block_radius_node = v.second.get_child("PreLoadBlockHalfWidth");
                load_param_.preload_block_half_width = show_road_block_radius_node.get<int>("<xmlattr>.v");

                auto show_street_light_node = v.second.get_child("ShowStreetLight");
                load_param_.show_street_light = show_street_light_node.get<int>("<xmlattr>.v");

                auto show_light_flash_node = v.second.get_child("ShowLightFlash");
                load_param_.show_light_flash = show_light_flash_node.get<int>("<xmlattr>.v");

                auto show_road_mesh_node = v.second.get_child("ShowRoadMesh");
                load_param_.show_road_mesh = show_road_mesh_node.get<int>("<xmlattr>.v");

                assert(load_param_.start_show_road_height > 0.0);
                assert(load_param_.block_half_width > 0);
                assert(load_param_.preload_block_half_width > 0);
                assert(load_param_.show_street_light >= 0);
                assert(load_param_.show_light_flash >= 0);
                assert(load_param_.show_road_mesh >= 0);
            }
        }
    }
    catch(const std::exception &e)
    {
         return false;
    }

	std::vector<RoadObjectId> graphs;
	for (auto & file : input_filenames)
	{
		boost::filesystem::path file_path(file);
		if (!boost::filesystem::exists(file_path))
			continue;
		std::string ext_string = file_path.extension().string();

		if (ext_string == ".osm") 
		{
			auto graph_id = ImportOSM(file);
			if (graph_id == 0)
				return false;
			graphs.push_back(graph_id);
			std::string db_filename = file_path.replace_extension(".db").string();
			//SaveDB(db_filename);
			data_from_cache_ = false;
		}
		else if (ext_string == ".db")
		{
			std::vector<RoadObjectId> ids = LoadDB(file);
			graphs.insert(graphs.end(), ids.begin(), ids.end());
			data_from_cache_ = true;
		}
	}


	for (auto & road_id : graphs)
	{
		auto road_graph = GetRoadGraph(road_id);
		if (road_graph && vehicle_num)
		{
			VehicleSchedule* vehicle_scheduler = new VehicleSchedule(vehicle_num, vehicle_models);
			VehicleManager::GetInstancePtr()->AddScheduler(vehicle_scheduler, road_graph);
		}
	}
    StartVehicleSchedule();
    return true;
}

//RoadEdge* RoadGraphInterface::GetEdgeCacheFromID(RoadObjectId graph_id, RoadObjectId edge_id)
//{
//    RoadEdge* ret = nullptr;
//    auto iter = db_handle_map_.find(graph_id);
//    if (iter == db_handle_map_.end() || iter->second == nullptr) return ret;
//
//    string key = boost::lexical_cast<string>(edge_id);
//    vector<char> data;
//
//    if ( iter->second->GetCacheData(key, data) > 0)
//    {
//        boost::iostreams::basic_array_source<char> device(data.data(), data.size());
//        boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
//        boost::archive::binary_iarchive ia(s);
//
//        ret = new RoadEdge;
//
//        try
//        {
//            ia >> *ret;
//        }
//        catch(...)
//        {
//            delete ret;
//            ret = nullptr;
//        }	
//    }
//
//    return ret;
//}


////存储roadgraph到db文件中，db支持存储多个roadgraph
//void RoadGraphInterface::SaveDB(const std::string & db_filename)
//{
//	CNetWorkCacheManager* db_handle = new CNetWorkCacheManager(db_filename, true, false);
//	db_handle->InitDB();
//
//	//存储 RoadGraph ID
//	std::vector<RoadObjectId> graph_ids;
//    BOOST_FOREACH(auto it, road_graphs_)
//    {
//		graph_ids.push_back(it.first);
//    }
//
//	std::string serial_str;
//	boost::iostreams::back_insert_device<std::string> inserter(serial_str);
//	boost::iostreams::stream<boost::iostreams::back_insert_device<std::string> > s(inserter);
//	boost::archive::binary_oarchive oa(s);
//
//	oa << graph_ids;
//	s.flush();
//	vector<char> data;
//	data.resize(serial_str.size());
//	std::copy(serial_str.begin(), serial_str.end(), data.begin());
//	bool result = db_handle->AddCacheData("road_id", data.data(), data.size(), true);
//	if (!result)
//		return;
//
//	//分别存储 RoadGraph
//	BOOST_FOREACH(auto it, road_graphs_)
//	{
//		it.second->SaveDB(*db_handle);
//	}
//	db_handle->CloseDB();
//	//delete db_handle;
//	//db_handle = nullptr;
//}

void RoadGraphInterface::OpenAllStreetLight(bool is_open)
{
    for (auto & kv: road_graphs_)
    {
        kv.second->OpenStreetLight(is_open);
    }
}

CNetWorkCacheManager* RoadGraphInterface::GetDatabaseHandler( RoadObjectId graph_id )
{
    auto iter = db_handle_map_.find(graph_id);
    if (iter == db_handle_map_.end())
    {
        return nullptr;
    }
    else
    {
        return iter->second;
    }
}

bool RoadGraphInterface::LoadRoadResources(const std::string& filename)
{
    //try
    //{
    //    ptree pt;
    //    read_xml(filename+"/RoadGraphConfig.xml", pt);

    //    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("RoadGraph"))
    //    {
    //        if (v.first == "RoadConfigs")
    //        {
    //            for (auto it = v.second.begin(); it != v.second.end(); ++it)
    //            {
    //                const string xml_filename = it->second.get<string>("<xmlattr>.v");
    //                LoadConfig(0, filename + "/"+ xml_filename);
    //            }
    //        }
    //    }
    //}
    //catch(const std::exception &e)
    //{
    //    return false;
    //}

    LoadConfig(0, filename + "/" + "RoadConfig.xml");

    return true;
}

std::vector<RoadObjectId> RoadGraphInterface::LoadDB(const std::string& db_filename)
{
	std::vector<RoadObjectId> ret;

//	CNetWorkCacheManager* db_handle = new CNetWorkCacheManager(db_filename, false, false);
////	db_handle->InitDB();
//
//	//读取graph id
//	string key = "road_id";
//	vector<char> data;
//	std::vector<RoadObjectId> road_id;
//
//	RoadGraph * road_graph;
//	if (db_handle->GetCacheData(key, data) > 0)
//	{
//		boost::iostreams::basic_array_source<char> device(data.data(), data.size());
//		boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
//		boost::archive::binary_iarchive ia(s);
//
//		
//		try
//		{
//			ia >> road_id;
//		}
//		catch (...)
//		{
//			return ret;
//		}
//	}
//
//	for (auto road_graph_id : road_id)
//	{
//		db_handle_map_.insert(make_pair(road_graph_id, db_handle));
//
//		//读取graph内容
//		key = boost::lexical_cast<string>(road_graph_id);
//		if (db_handle->GetCacheData(key, data) > 0)
//		{
//			boost::iostreams::basic_array_source<char> device(data.data(), data.size());
//			boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
//			boost::archive::binary_iarchive ia(s);
//
//			road_graph = new RoadGraph(0);
//
//			try
//			{
//				ia >> *road_graph;
//			}
//			catch (...)
//			{
//				delete road_graph;
//				road_graph = nullptr;
//			}
//
//			if (road_graph)
//			{
//				road_graphs_.insert(make_pair(road_graph_id, road_graph));
//				road_graph->LoadDB(*db_handle);
//				//RoadConstructor constructor;
//				//constructor.Construct(road_graph);
//				road_graph->Init();
//				ret.push_back(road_graph->GetID());
//			}
//		}
//	}

	return ret;


}

void RoadGraphInterface::ComputeVisibleTerrainBlocks(unordered_set<std::uint64_t>& block_index)
{
    CTerrainSdkInterface::GetInstance()->GetRenderInfo(renderInfoVec);

    // renderInfoVec 是按照 level x y 信息排列的
    BOOST_FOREACH(auto& idx, renderInfoVec)
    {
        if (idx.x >= load_param_.split_level)
        {
            uint64_t x = idx.y;
            uint64_t y = idx.z;

            x = x / (1LL << (idx.x - load_param_.split_level));
            y = y / (1LL << (idx.x - load_param_.split_level));

            uint64_t ret_block_idx = RoadUtils::GenerateKeyFromIndex(x, y);
            block_index.insert(ret_block_idx);
        }
    }
}

bool RoadGraphInterface::Load(std::vector<std::string> &filenames)
{
    ClearAllRoadGraph();
    for (auto& scene : filenames)
    {
        LoadRoadResources(scene);
    }

    return true;
}

bool RoadGraphInterface::Unload()
{
    ClearAllRoadGraph();

    return true;
}

int RoadGraphInterface::OnEvent(double _curtime, double _deltatime, void *_userdata)
{
    VPE::dvec3 camera_pos(0.0), camera_dir(0.0);

    RoadUtils::GetCameraParameter(camera_pos, camera_dir);

    VehicleManager::GetInstancePtr()->UpdateTime(camera_pos, _curtime);

    return 0;
}

const VPE::vec3 & RoadGraphInterface::GetStreetLightColor() const
{
	return street_light_color_;
}

float RoadGraphInterface::GetStreetLightSpace() const
{
	return street_light_space_;
}

void RoadGraphInterface::SetVisible(bool visible)
{
	SetEnable(visible);
}