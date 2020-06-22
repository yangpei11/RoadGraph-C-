#pragma once

#include "VPMath.h"
#include "GL/glew.h"

#include <cassert>
#include <list>
#include <vector>
#include <cstdint>
#include "CommonRoadType.h"

struct RoadMesh;
struct RoadVBOCollection;
class RoadIntersectionPoint;

namespace RoadUtils
{

    /// 假设 v 已经单位化了
    /// 相对于x轴返回角度
    double GetVectorAngle(const VPE::dvec2& v);

    /// return v1相对v0的角度， 可能为负数
    double GetVectorRelativeAngle(const VPE::dvec2& v0, const VPE::dvec2& v1);

    /// \caution 默认已经单位化
    /// 只返回正数, [0, 360]
    double Get3DVectorAngle(const VPE::dvec3& v0, const VPE::dvec3& v1, const VPE::dvec3& plane_normal);

    VPE::dmat4x4 BuildLocalTransform(const VPE::dvec3& axis_x, const VPE::dvec3& axis_y, const VPE::dvec3& axis_z, const VPE::dvec3& axis_center);

    RoadMesh* BuildRectangleDecal(
        const VPE::dvec3& start_point, 
        const VPE::dvec3& vertical_dir, 
        const VPE::dvec3& horizon_dir,
        const VPE::dvec3& up_dir,
        double width, 
        double length,
        double offset2road
        );


    int DrawMeshOnLocal(const RoadMesh* mesh);

    void MergeMesh(const std::vector<const RoadMesh*> meshes, RoadMesh* ret_mesh);

    void GetCameraParameter(VPE::dvec3& pos, VPE::dvec3& dir);

    void MakeVBO(const RoadMesh* mesh, RoadVBOCollection* vbos);

    void MakeVAO(const RoadMesh* mesh, RoadVAOCollection* vaos);

    void ClearVBO( RoadVBOCollection* vbos );


    std::size_t	HashMultiValue( const std::vector<std::int64_t>& values);

    std::size_t  HashSpecifiedValue( RoadObjectId v0, RoadObjectId v1, int v2, int v3 );

    std::uint64_t GenerateKeyFromLonLat(double lon, double lat, int level);
    std::uint64_t GenerateKeyFromIndex(int x, int y);

    void GenerateIndexFromIndex(double lon, double lat, int level, int& x, int& y);

    void MakeBuffer(GLenum target, GLuint& buffer, const void *buffer_data, GLsizei buffer_size);

	/// \brief 计算二维平面内两直线交点
	/// p为直线经过的点，v为直线方向向量
	VPE::dvec2 GetIntersectPoint(const VPE::dvec2 & p1, const VPE::dvec2 & v1, const VPE::dvec2 & p2, const VPE::dvec2 & v2);

    bool IsNight();
   
    static int WAYID = 0;
};