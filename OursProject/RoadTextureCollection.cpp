#include "RoadTextureCollection.h"
#include "TextureCollection.h"

RoadTextureCollection g_road_texture_collection;

RoadTextureCollection::RoadTextureCollection()
	:index_(0)
{
}

void RoadTextureCollection::AddTexture(const std::string & name, const std::string& filename)
{
    auto it = texture_maps.find(name);
    if (it != texture_maps.end())	
		return;

 //   auto tex = GLFreeImageTexture::LoadTexture2D(filename.c_str(), GL_COMPRESSED_RGBA_BPTC_UNORM);

	//GLfloat max_TexAni;    //查询允许的各向异性数量  
	//glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_TexAni);

	//glBindTexture(GL_TEXTURE_2D, tex.get());
	//glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, max_TexAni);
	//glBindTexture(GL_TEXTURE_2D, 0);

	//GLuint64 texHandle = glGetTextureHandleNV(tex.get());
	//glMakeTextureHandleResidentNV(texHandle);

	GLuint64 texHandle = TextureCollection::GetInstance()->GetTextureHandle(filename);
    texture_maps.insert(make_pair(name, std::make_pair(index_, texHandle)));
	index_++;
}

//GLuint RoadTextureCollection::GetTexture(const std::string& filename)
//{
//    auto it = texture_maps.find(filename);
//    if (it == texture_maps.end())	return 0;
//
//    return it->second.first;
//}

uint16_t RoadTextureCollection::GetTextureIndex(const std::string& filename)
{
	auto it = texture_maps.find(filename);
	if (it == texture_maps.end())	
		return 0;
	return it->second.first;
}

void RoadTextureCollection::GetTextureHandles(std::vector<GLuint64> & texHandles)
{
	texHandles.resize(texture_maps.size());
	for (auto & item : texture_maps)
	{
		texHandles[item.second.first] = item.second.second;
	}
}

void RoadTextureCollection::ReleaseTexture()
{
    texture_maps.clear();
}
