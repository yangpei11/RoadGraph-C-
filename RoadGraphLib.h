#pragma once

#ifdef ROADGRAPH_EXPORTS
#define _ROADGRAPH_EXP __declspec(dllexport)
#define _ROADGRAPH_API extern "C" __declspec(dllexport)
#else
#define _ROADGRAPH_EXP __declspec(dllimport)
#define _ROADGRAPH_API extern "C" __declspec(dllimport)
#endif

#include "IPlugin.h"
class _ROADGRAPH_EXP RoadGraphicsPlugin : public VPE::IPlugin {
public:
    RoadGraphicsPlugin(){};
    virtual bool Init(void *lpvoid) override;
    virtual bool Clean() override;
};