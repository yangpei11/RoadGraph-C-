#pragma once

#include "GLFreeImageTexture.h"
#include "GL/glew.h"
#include <unordered_map>
#include <string>
#include <vector>

class RoadTextureCollection
{
public:

	RoadTextureCollection();

	void AddTexture(const std::string & name, const std::string& filename);

    //GLuint GetTexture( const std::string& filename);

	uint16_t GetTextureIndex(const std::string& filename);

	void GetTextureHandles(std::vector<GLuint64> & texHandles);

    void ReleaseTexture();

private:

    std::unordered_map<std::string, std::pair<uint16_t, GLuint64>> texture_maps;
	uint16_t index_;
};

extern RoadTextureCollection g_road_texture_collection;