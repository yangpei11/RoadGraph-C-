#include "RoadEdge.h"
#include "RoadNode.h"
#include "CommonRoadType.h"
#include "RoadUtils.h"
#include "RoadGraphInterface.h"

#include "ViWoRoot.h"
#include "ITerrain.h"
#include "Coordination.h"
#include "RenderEngine/IVectorGraphicsRender.h"
#include "VPMath.h"

#include "boost/asio.hpp"
#include "boost/foreach.hpp"
#define GLM_ENABLE_EXPERIMENTAL

#include "EffectManagerInterface.h"
#include "RenderTree.h"

using namespace std;

RoadEdge::RoadEdge(RoadObjectId id, RoadGraph *road_graph)
    : RoadFeature(id, road_graph)
    , has_light_(false){
    light_space_ = g_roadgraph_interface.GetStreetLightSpace();
    column_space_ = 80.0f;
}

RoadEdge::~RoadEdge() {
}

void RoadEdge::AddNode(RoadNode *node) {
    
    node->edges.push_back(this);
    if (nodes_.size() > 0) {
        RoadNode *last_node = nodes_.back();
        last_node->connectNodes.push_back(node);
        node->connectNodes.push_back(last_node);
    }
	nodes_.push_back(node);
}

RoadNode *RoadEdge::GetOtherNode(const RoadNode *const node) const {
    return (node == nodes_.front()) ? nodes_.back() : nodes_.front();
}

void RoadEdge::SetupLanes() {
    if (lane_nums_ == 0) {
        switch (type_) {
            case MOTORWAY:
            case TRUNK:
            case PRIMARY:
            case SECONDARY:
            case TERTIARY:
            case MOTORWAY_LINK:
            case TRUNK_LINK:
                lane_nums_ = 2;
                break;
            default:
                lane_nums_ = 1;
                break;
        }
    }
}

void RoadEdge::SetupSpeed() {
    if (max_speed_ == 0.0f) {
        switch (type_) {
            case MOTORWAY:
            case TRUNK:
                max_speed_ = 120;
                break;
            case PRIMARY:
                max_speed_ = 100;
                break;
            case SECONDARY:
                max_speed_ = 80;
                break;
            case TERTIARY:
                max_speed_ = 40;
                break;
            case QUATERNARY:
                max_speed_ = 30;
                break;
            case RESIDENTIAL:
                max_speed_ = 20;
            default:
                max_speed_ = 40;
                break;
        }
    }
}

void RoadEdge::SetupWidth() {
    switch (type_) {
        case MOTORWAY:
        case TRUNK:
        case PRIMARY:
            lane_width_ = 3.75f;
            break;
        case SECONDARY:
        case TERTIARY:
            lane_width_ = 3.5f;
            break;
        case QUATERNARY:
            lane_width_ = 3.25f;
            break;
        case RESIDENTIAL:
            lane_width_ = 3.0f;
        default:
            lane_width_ = 3.5f;
            break;
    }
    width_ = lane_width_ * lane_nums_;
}

void RoadEdge::SetupLights() {
    switch (type_) {
        case TRUNK:
        case PRIMARY:
        case SECONDARY:
            has_light_ = true;
            break;
        default:
            has_light_ = false;
            break;
    }
}