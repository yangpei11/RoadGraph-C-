
#pragma once

#include "glm/glm.hpp"

#include "BoostCommon.h"
#include "boost/lexical_cast.hpp"
#include "boost/serialization/list.hpp"
#include "boost/serialization/string.hpp"
#include "boost/serialization/vector.hpp"
#include "boost/serialization/unordered_map.hpp"
#include "boost/serialization/utility.hpp"
#include "boost/serialization/split_member.hpp"

#include <vector>
#include <string>
#include <unordered_map>

struct RoadMesh;
class RoadGraph;

class RoadMarkerLine
{

public:

	RoadMarkerLine(const glm::dvec3& start, const glm::dvec3& end);

	~RoadMarkerLine();

	void SetAttribute(const std::string& key, const std::string& value);

	/// 函数不对异常检查
	template<typename T>
	T GetAttribute(const std::string& key) const;

	// true: white   false: yellow
	bool WhiteOrYellow() const;

	void Construct(const glm::dvec3& graph_center);

	std::vector<RoadMesh*> GetMesh() const;

private:

	void WhiteDashConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);
	void WhiteSolidConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);
	void YellowDashConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);
	void YellowSolidConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);
	void DoubleWhiteDashConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);
	void DoubleYellowSolidConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);
	void YellowDashSolidConstruct(const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);

	RoadMesh* ConstructSolidMarker(const glm::dvec3& start_point, const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);
	std::vector<RoadMesh*> ConstructDashMarker(const glm::dvec3& start_point, const glm::dvec3& road_dir, const glm::dvec3& vertical_dir, const glm::dvec3& up_dir, const glm::dvec3& graph_center);

	void ClearMesh();

	friend class boost::serialization::access;

	template<class Archive>  
	void save(Archive & ar, const unsigned int version) const
	{
		ar & start_.x;
		ar & start_.y;
		ar & start_.z;

		ar & end_.x;
		ar & end_.y;
		ar & end_.z;

		ar & white_color_.x;
		ar & white_color_.y;
		ar & white_color_.z;

		ar & yellow_color_.x;
		ar & yellow_color_.y;
		ar & yellow_color_.z;

		ar & meshes_;
		ar & attributes_;
	}

	template<class Archive>  
	void load(Archive & ar, const unsigned int version)
	{
		ar & start_.x;
		ar & start_.y;
		ar & start_.z;

		ar & end_.x;
		ar & end_.y;
		ar & end_.z;

		ar & white_color_.x;
		ar & white_color_.y;
		ar & white_color_.z;

		ar & yellow_color_.x;
		ar & yellow_color_.y;
		ar & yellow_color_.z;

		ar & meshes_;
		ar & attributes_;
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER()  

	glm::dvec3 start_;
	glm::dvec3 end_;

	std::vector<RoadMesh*>	meshes_;

	std::unordered_map<std::string, std::string>  attributes_;
};

template<typename T>
T RoadMarkerLine::GetAttribute(const std::string& key) const
{
	auto it = attributes_.find(key);
	if (it != attributes_.end())
	{
		return boost::lexical_cast<T>(it->second);
	}
	else
	{
		return T();
	}
}
