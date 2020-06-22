#pragma once

#include "VPMath.h"

#include "boost/asio.hpp"
#include "boost/serialization/vector.hpp"
#include "boost/serialization/utility.hpp"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"

#include <vector>
#include <string>
#include <cstdint>

#include "GL/glew.h"

using  RoadObjectId = std::uint64_t;
using TerrainBlockID = std::uint64_t;

struct RoadVertex
{
    VPE::fvec3 pos;
    VPE::fvec2 tex_coords;
    VPE::fvec3 normal;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & pos;
        ar & tex_coords;
        ar & normal;
    }
};

struct  RoadVBOCollection
{
    GLuint vertex_vbo;
    GLuint index_vbo;
    RoadVBOCollection() : vertex_vbo(0), index_vbo(0) {}
};

struct  RoadVAOCollection
{
    GLuint vertex_vao;
    GLuint vertex_vbo;
    GLuint index_vbo;
    RoadVAOCollection() : vertex_vao(0), vertex_vbo(0), index_vbo(0) {}
};

struct RoadMesh
{
    std::vector<RoadVertex> vertices;
    std::vector<unsigned int> indexes;
	std::string	tex_name; //一个mesh只包含一张纹理

    void clear()
    {
        vertices.swap(std::vector<RoadVertex>());
        indexes.swap(std::vector<unsigned int>());
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & vertices;
        ar & indexes;
    }
};

struct RoadLoadParameter
{
    int split_level;

    double start_show_road_height;

    // 块数参数
    int block_half_width;
    int preload_block_half_width;

    // 是否显示路灯
    int show_street_light;

    // 是否开启道路灯光
    int show_light_flash;

    // 是否显示路网
    int show_road_mesh;
};

struct ModelType
{
    std::string name;
    double scale;
	double distance;
};

//道路类别
enum RoadType {
    UNKOWN,
    MOTORWAY,
    TRUNK,
    PRIMARY,
    SECONDARY,
    TERTIARY,
    QUATERNARY,
    RESIDENTIAL,
    MOTORWAY_LINK,
    TRUNK_LINK,
    PRIMARY_LINK,
    SECONDARY_LINK,
    TERTIARY_LINK
};

namespace boost {
    namespace serialization {
        template<class Archive, class T, VPE::precision P>
        void serialize(Archive & ar, VPE::tvec4<T, P> & p, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(p.x);
            ar & BOOST_SERIALIZATION_NVP(p.y);
            ar & BOOST_SERIALIZATION_NVP(p.z);
            ar & BOOST_SERIALIZATION_NVP(p.w);
        }

        template<class Archive, class T, VPE::precision P>
        void serialize(Archive & ar, VPE::tvec3<T, P> & p, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(p.x);
            ar & BOOST_SERIALIZATION_NVP(p.y);
            ar & BOOST_SERIALIZATION_NVP(p.z);
        }

        template<class Archive, class T, VPE::precision P>
        void serialize(Archive & ar, VPE::tvec2<T, P> & p, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(p.x);
            ar & BOOST_SERIALIZATION_NVP(p.y);
        }

		template<class Archive, class T, VPE::precision P>
		void serialize(Archive & ar, VPE::tmat4x4<T, P> & p, const unsigned int version)
		{
			for (int i = 0; i < 4 ; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					ar & p[i][j];
				}
			}
		}
    } // namespace serialization
} // namespace boost