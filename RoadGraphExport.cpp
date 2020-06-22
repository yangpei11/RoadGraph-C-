#include "RoadGraphLib.h"
#include "RoadGraphInterface.h"
#include "RoadTextureCollection.h"

#include "ViWoRoot.h"
#include "PluginManager.h"
#include "RenderInterface.h"


extern "C" BOOST_SYMBOL_EXPORT RoadGraphicsPlugin VPEPlugin;
RoadGraphicsPlugin                                VPEPlugin;

bool RoadGraphicsPlugin::Init(void *param) {
    ViWoROOT::GetModuleManager()->RegistInternalModule("RoadGraphInterface", &g_roadgraph_interface);
    ViWoROOT::GetRenderSystem()->GetCurrentRender()->AddRenderable(&g_roadgraph_interface, "RoadGraphRender");

    return true;
}

bool RoadGraphicsPlugin::Clean() {
    ViWoROOT::GetModuleManager()->RemoveInternalModule("GISInterface");

    ViWoROOT::GetRenderSystem()->GetCurrentRender()->RemoveRenderable(&g_roadgraph_interface);

    try
    {
        g_roadgraph_interface.ClearAllRoadGraph();
    }
    catch(...)
    {
        return false;
    }

    g_road_texture_collection.ReleaseTexture();

    return true;
}
