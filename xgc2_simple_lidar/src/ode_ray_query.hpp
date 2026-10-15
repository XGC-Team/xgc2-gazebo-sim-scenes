#pragma once

#include <algorithm>
#include <ode/ode.h>
#include <unordered_set>

namespace xgc2_simple_lidar {

// The native ODE near callback, shared by the sensor and its geometry regression.
// Ray userdata points to its nearest range; exclusions are exact model geometry IDs.
struct OdeRayQuery {
    dSpaceID space = nullptr;
    double minimum_range = 0;
    std::unordered_set<dGeomID> emitter_collisions;

    static void Intersect(void* context, dGeomID first, dGeomID second) {
        auto* query = static_cast<OdeRayQuery*>(context);
        if (dGeomIsSpace(first) || dGeomIsSpace(second)) {
            dSpaceCollide2(first, second, context, &Intersect);
            return;
        }
        dGeomID ray = nullptr;
        if (dGeomGetSpace(first) == query->space)
            ray = first;
        else if (dGeomGetSpace(second) == query->space)
            ray = second;
        if (!ray || (dGeomGetClass(first) == dRayClass && dGeomGetClass(second) == dRayClass))
            return;
        const auto target = first == ray ? second : first;
        if (query->emitter_collisions.count(target) != 0)
            return;
        dContactGeom contact;
        if (dCollide(first, second, 1, &contact, sizeof(contact)) > 0) {
            auto* range = static_cast<float*>(dGeomGetData(ray));
            *range = std::min(*range, static_cast<float>(contact.depth + query->minimum_range));
        }
    }
};

} // namespace xgc2_simple_lidar
