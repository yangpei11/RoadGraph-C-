
#include "RoadMarkerLine.h"
#include "CommonRoadType.h"
#include "RoadUtils.h"

#include <algorithm>

using namespace glm;
using namespace std;

RoadMarkerLine::RoadMarkerLine( const glm::dvec3& start, const glm::dvec3& end)
	: start_(start),
		end_(end)
{
	SetAttribute("road_marker_line_type", "white_dash");
	SetAttribute("road_marker_line_width", "0.15");
	SetAttribute("road_marker_line_length", "4.0");
	SetAttribute("road_marker_line_gap_length", "6.0");
	SetAttribute("road_marker_line_offset_to_road", "0.1");
	SetAttribute("road_marker_line_double_gap", "0.15");
}

RoadMarkerLine::~RoadMarkerLine()
{
	ClearMesh();
}

void RoadMarkerLine::SetAttribute(const std::string& key, const std::string& value)
{
	auto it = attributes_.find(key);
	if (it != attributes_.end())
	{
		it->second = value;
	}
	else
	{
		attributes_.insert(make_pair(key, value));
	}
}

bool RoadMarkerLine::WhiteOrYellow() const
{
	string marker_type = GetAttribute<string>("road_marker_line_type");

	bool ret = true;

	if ( marker_type == "white_dash")
	{
		ret = true;
	}
	else if (marker_type == "white_solid")
	{
		ret = true;
	}
	else if (marker_type == "yellow_dash")
	{
		ret = false;
	}
	else if (marker_type == "yellow_solid")
	{
		ret = false;
	}
	else if (marker_type == "double_white_dash")
	{
		ret = true;
	}
	else if (marker_type == "double_yellow_solid")
	{
		ret = false;
	}
	else if (marker_type == "yellow_dash_solid")
	{
		ret = false;
	}

	return ret;
}

void RoadMarkerLine::Construct(const glm::dvec3& graph_center)
{

	ClearMesh();

	dvec3 road_dir = end_ - start_;
	dvec3 up = start_;

	road_dir = glm::normalize(road_dir);
	up = glm::normalize(up);

	dvec3 vertical = glm::cross(road_dir, up);
	vertical = glm::normalize(vertical);

	string marker_type = GetAttribute<string>("road_marker_line_type");

	if ( marker_type == "white_dash")
	{
		WhiteDashConstruct(road_dir, vertical, up, graph_center);
	}
	else if (marker_type == "white_solid")
	{
		WhiteSolidConstruct(road_dir, vertical, up, graph_center);
	}
	else if (marker_type == "yellow_dash")
	{
		YellowDashConstruct(road_dir, vertical, up, graph_center);
	}
	else if (marker_type == "yellow_solid")
	{
		YellowSolidConstruct(road_dir, vertical, up, graph_center);
	}
	else if (marker_type == "double_white_dash")
	{
		DoubleWhiteDashConstruct(road_dir, vertical, up, graph_center);
	}
	else if (marker_type == "double_yellow_solid")
	{
		DoubleYellowSolidConstruct(road_dir, vertical, up, graph_center);
	}
	else if (marker_type == "yellow_dash_solid")
	{
		YellowDashSolidConstruct(road_dir, vertical, up, graph_center);
	}

}

std::vector<RoadMesh*> RoadMarkerLine::GetMesh() const
{
	return meshes_;
}

void RoadMarkerLine::WhiteDashConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	vector<RoadMesh*> meshes = ConstructDashMarker(start_, road_dir, vertical_dir, up_dir, graph_center);

	meshes_.insert(meshes_.end(), meshes.begin(), meshes.end());
}

void RoadMarkerLine::WhiteSolidConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	RoadMesh* mesh = ConstructSolidMarker(start_, road_dir, vertical_dir, up_dir, graph_center);

	meshes_.push_back(mesh);
}

void RoadMarkerLine::YellowDashConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	vector<RoadMesh*> meshes = ConstructDashMarker(start_, road_dir, vertical_dir, up_dir, graph_center);

	meshes_.insert(meshes_.end(), meshes.begin(), meshes.end());
}

void RoadMarkerLine::YellowSolidConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	RoadMesh* mesh = ConstructSolidMarker(start_, road_dir, vertical_dir, up_dir, graph_center);

	meshes_.push_back(mesh);
}

void RoadMarkerLine::DoubleWhiteDashConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	double width = GetAttribute<double>("road_marker_line_width");
	double gap_width = GetAttribute<double>("road_marker_line_double_gap");

	double offset = width / 2 + gap_width / 2;

	dvec3 start_point1 = start_ + vertical_dir * offset;
	dvec3 start_point2 = start_ - vertical_dir * offset;

	vector<RoadMesh*> meshes1, meshes2;

	meshes1 = ConstructDashMarker(start_point1, road_dir, vertical_dir, up_dir, graph_center);
	meshes2 = ConstructDashMarker(start_point2, road_dir, vertical_dir, up_dir, graph_center);

	meshes_.insert(meshes_.end(), meshes1.begin(), meshes1.end());
	meshes_.insert(meshes_.end(), meshes2.begin(), meshes2.end());

}

void RoadMarkerLine::DoubleYellowSolidConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	double width = GetAttribute<double>("road_marker_line_width");
	double gap_width = GetAttribute<double>("road_marker_line_double_gap");

	double offset = width / 2 + gap_width / 2;

	dvec3 start_point1 = start_ + vertical_dir * offset;
	dvec3 start_point2 = start_ - vertical_dir * offset;

	RoadMesh *meshes1, *meshes2;

	meshes1 = ConstructSolidMarker(start_point1, road_dir, vertical_dir, up_dir, graph_center);
	meshes2 = ConstructSolidMarker(start_point2, road_dir, vertical_dir, up_dir, graph_center);

	meshes_.push_back(meshes1);
	meshes_.push_back(meshes2);
}

void RoadMarkerLine::YellowDashSolidConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	double width = GetAttribute<double>("road_marker_line_width");
	double gap_width = GetAttribute<double>("road_marker_line_double_gap");

	double offset = width / 2 + gap_width / 2;

	dvec3 start_point1 = start_ + vertical_dir * offset;
	dvec3 start_point2 = start_ - vertical_dir * offset;

	RoadMesh* meshes1;
	vector<RoadMesh*> meshes2;

	meshes1 = ConstructSolidMarker(start_point1, road_dir, vertical_dir, up_dir, graph_center);
	meshes2 = ConstructDashMarker(start_point2, road_dir, vertical_dir, up_dir, graph_center);

	meshes_.push_back(meshes1);
	meshes_.insert(meshes_.end(), meshes2.begin(), meshes2.end());
}

RoadMesh* RoadMarkerLine::ConstructSolidMarker(const glm::dvec3& start_point, const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	double dist = glm::distance(start_, end_);
	double width = GetAttribute<double>("road_marker_line_width");
	double offset2road = GetAttribute<double>("road_marker_line_offset_to_road");

	RoadMesh* ret = RoadUtils::BuildRectangleDecal(start_point, vertical_dir, road_dir, up_dir, width, dist, offset2road);	

	RoadUtils::TransPosition2Local(ret, graph_center);

	return ret;
}

std::vector<RoadMesh*> RoadMarkerLine::ConstructDashMarker(const glm::dvec3& start_point, const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center)
{
	vector<RoadMesh*> ret;

	double dist = glm::distance(start_, end_);
	double width = GetAttribute<double>("road_marker_line_width");
	double offset2road = GetAttribute<double>("road_marker_line_offset_to_road");

	double markerlength = GetAttribute<double>("road_marker_line_length");
	double marker_gap_length = GetAttribute<double>("road_marker_line_gap_length");


	for ( double road_length = 0.0; road_length + markerlength < dist; road_length += markerlength + marker_gap_length)
	{
		RoadMesh* mesh = RoadUtils::BuildRectangleDecal(start_point + road_dir * road_length, vertical_dir, road_dir, up_dir, width, markerlength, offset2road);
		
		RoadUtils::TransPosition2Local(mesh, graph_center);
			
		ret.push_back(mesh);
	}

	return ret;
}

void RoadMarkerLine::ClearMesh()
{
	for_each(meshes_.begin(),  meshes_.end(), [](RoadMesh* r){ delete r;});
	meshes_.clear();
}