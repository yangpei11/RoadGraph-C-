#include "RoadUtils.h"
#include "CommonRoadType.h"

#include "ViWoRoot.h"
#include "Camera.h"
#include "CameraManager.h"
#include "ITerrain.h"
#include "Coordination.h"
#include "TerrainExport/TerrainCacheIndex.h"

#include "VPMath.h"

#include "boost/asio.hpp"
#include "boost/functional/hash.hpp"


#include <cmath>
#include "GLProgramObject.h"
#include "RenderEngine/RenderEngine.h"
#include "GLShaderUtils.h"
#include "RoadGraphInterface.h"

using namespace std;
using namespace glm;

double RoadUtils::GetVectorAngle(const VPE::dvec2& v)
{
	
	double ret = atan2(v.y, v.x);// -pi ~ pi

	if(v.y < 0)
	{
		ret = VPE::two_pi<double>() + ret;
	}

	return ret;
}

double RoadUtils::GetVectorRelativeAngle(const VPE::dvec2& v0, const VPE::dvec2& v1)
{
	double angle0 = GetVectorAngle(v0);
	double angle1 = GetVectorAngle(v1);

	return angle1 - angle0;
}

double RoadUtils::Get3DVectorAngle(const VPE::dvec3& v0, const VPE::dvec3& v1, const VPE::dvec3& plane_normal)
{
	double angle = std::acos(VPE::dot(v0, v1));
	auto v_cross = VPE::cross(v0, v1);
	if (VPE::dot(plane_normal, v_cross) < 0)
	{
		angle = 2 * VPE::PI - angle;
	}

	return VPE::degrees(angle);
}

VPE::dmat4 RoadUtils::BuildLocalTransform(const VPE::dvec3 &axis_x, const VPE::dvec3 &axis_y, const VPE::dvec3 &axis_z, const VPE::dvec3 &axis_center)
{
	return VPE::dmat4(
		axis_x.x, axis_x.y, axis_x.z, 0.0,
		axis_y.x, axis_y.y, axis_y.z, 0.0,
		axis_z.x, axis_z.y, axis_z.z, 0.0,
		axis_center.x, axis_center.y, axis_center.z, 1.0);

}

RoadMesh* RoadUtils::BuildRectangleDecal(const VPE::dvec3& start_point, const VPE::dvec3& vertical_dir, const VPE::dvec3& forward_dir, const VPE::dvec3& up_dir, double width, double length, double offset2road)
{
	RoadMesh* ret = new RoadMesh;

	VPE::dvec3 temp_pos;
	RoadVertex temp_vertex;

	temp_pos = start_point + vertical_dir * (width / 2) + up_dir * offset2road;
	temp_vertex.pos = temp_pos;
	ret->vertices.push_back(temp_vertex);

	temp_pos = start_point - vertical_dir * (width / 2) + up_dir * offset2road;
	temp_vertex.pos = temp_pos;
	ret->vertices.push_back(temp_vertex);

	temp_pos = start_point + forward_dir * length + vertical_dir * (width / 2) + up_dir * offset2road;
	temp_vertex.pos = temp_pos;
	ret->vertices.push_back(temp_vertex);

	temp_pos = start_point + forward_dir * length - vertical_dir * (width / 2) + up_dir * offset2road;
	temp_vertex.pos = temp_pos;
	ret->vertices.push_back(temp_vertex);

	ret->indexes.push_back(0);
	ret->indexes.push_back(3);
	ret->indexes.push_back(1);

	ret->indexes.push_back(0);
	ret->indexes.push_back(2);
	ret->indexes.push_back(3);

	return ret;
}

int RoadUtils::DrawMeshOnLocal(const RoadMesh* mesh)
{

	//dvec3 center_glm = mesh->vertexes[mesh->vertexes.size() / 2];

	//std::vector<VPE::dvec3>	road_mesh_vertexes;

	//road_mesh_vertexes.resize(mesh->vertexes.size());

	//for (size_t i = 0; i < mesh->vertexes.size(); ++i)
	//{
	//	road_mesh_vertexes[i] = mesh->vertexes[i] - center_glm;
	//}

	return 0;

}

void RoadUtils::MergeMesh(const std::vector<const RoadMesh*> meshes, RoadMesh* ret_mesh)
{
	if (meshes.empty() || ret_mesh == nullptr)	return;

	int start_index = 0;
	for (auto it = meshes.begin(); it != meshes.end(); ++it)
	{
		start_index = ret_mesh->vertices.size();
		ret_mesh->vertices.insert(ret_mesh->vertices.end(), (*it)->vertices.begin(), (*it)->vertices.end());

		for (size_t i = 0; i < (*it)->indexes.size(); ++i)
		{
			ret_mesh->indexes.push_back(start_index + (*it)->indexes[i]);
		}
	}
}

void RoadUtils::GetCameraParameter(VPE::dvec3& pos, VPE::dvec3& dir)
{
	auto camera_p = ViWoROOT::GetCameraManager()->GetCurrCamera();

	auto pos_viwo = camera_p->getPosition();
	auto dir_viwo = camera_p->getDirection();

	pos.x = pos_viwo.x;
	pos.y = pos_viwo.y;
	pos.z = pos_viwo.z;

	dir.x = dir_viwo.x;
	dir.y = dir_viwo.y;
	dir.z = dir_viwo.z;
}

void RoadUtils::MakeVBO(const RoadMesh* mesh, RoadVBOCollection* vbos)
{
	MakeBuffer(GL_ARRAY_BUFFER, vbos->vertex_vbo, &(mesh->vertices[0]), mesh->vertices.size() * sizeof(RoadVertex));
	MakeBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos->index_vbo, &(mesh->indexes[0]), mesh->indexes.size() * sizeof(GLuint));
}

void RoadUtils::MakeVAO(const RoadMesh* mesh, RoadVAOCollection* vaos)
{

	MakeBuffer(GL_ARRAY_BUFFER, vaos->vertex_vbo, &(mesh->vertices[0]), mesh->vertices.size() * sizeof(RoadVertex));
	MakeBuffer(GL_ELEMENT_ARRAY_BUFFER, vaos->index_vbo, &(mesh->indexes[0]), mesh->indexes.size() * sizeof(GLuint));
}

void RoadUtils::ClearVBO(RoadVBOCollection* vbos)
{
	glDeleteBuffers(1, &(vbos->vertex_vbo));
	glDeleteBuffers(1, &(vbos->index_vbo));

	vbos->vertex_vbo = 0;
	vbos->index_vbo = 0;
}

std::size_t RoadUtils::HashMultiValue(const std::vector<std::int64_t>& values)
{
	std::size_t seed = 0;

	for (auto it = values.begin(); it != values.end(); ++it)
	{
		boost::hash_combine(seed, *it);
	}

	return seed;
}

std::size_t RoadUtils::HashSpecifiedValue(RoadObjectId v0, RoadObjectId v1, int v2, int v3)
{
	std::vector<std::int64_t> values;
	values.push_back((std::int64_t)(v0));
	values.push_back((std::int64_t)(v1));
	values.push_back((std::int64_t)(v2));
	values.push_back((std::int64_t)(v3));

	return HashMultiValue(values);
}

std::uint64_t RoadUtils::GenerateKeyFromLonLat(double lon, double lat, int level)
{
	CTerrainIndex index(lon, lat, level);
	std::uint64_t x = index.GetX();
	std::uint64_t y = index.GetY();
	std::uint64_t ret = (x << 32) + y;

	return ret;
}

std::uint64_t RoadUtils::GenerateKeyFromIndex(int x, int y)
{
	std::uint64_t temp_x = x;
	std::uint64_t temp_y = y;
	std::uint64_t ret = (temp_x << 32) + temp_y;

	return ret;
}

void RoadUtils::GenerateIndexFromIndex(double lon, double lat, int level, int& x, int& y)
{
	CTerrainIndex index(lon, lat, level);
	x = index.GetX();
	y = index.GetY();
}

void RoadUtils::MakeBuffer(GLenum target, GLuint& buffer, const void *buffer_data, GLsizei buffer_size)
{
	glGenBuffers(1, &buffer);
	glBindBuffer(target, buffer);
	glBufferData(target, buffer_size, buffer_data, GL_STATIC_DRAW);
}
VPE::dvec2 RoadUtils::GetIntersectPoint(const VPE::dvec2 & p1, const VPE::dvec2 & v1, const VPE::dvec2 & p2, const VPE::dvec2 & v2)
{
	double t2 = (v1.y*(p1.x - p2.x) + v1.x * (p2.y - p1.y)) / (v2.x * v1.y - v1.x * v2.y);
	return p2 + v2 * t2;
}

bool RoadUtils::IsNight() {
    bool                                       ret             = false;
    VPE::RenderSystem::EffectManagerInterface *effect_iterface = ViWoROOT::GetEffectManager();
    if (effect_iterface != nullptr) {
        double hour = effect_iterface->GetLocalHourOfDay();
        if (hour > 17.0 || hour < 7.0) {
            ret = true;
        }
    }

    return ret;
}