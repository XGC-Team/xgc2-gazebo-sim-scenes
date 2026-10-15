#include "ode_ray_query.hpp"
#include <array>
#include <gtest/gtest.h>

namespace xgc2_simple_lidar {
namespace {
class NativeOdeRayTest : public ::testing::Test {
  protected:
    void SetUp() override {
        ASSERT_NE(dInitODE2(0), 0);
        scene_ = dSimpleSpaceCreate(nullptr);
        query_.space = dSimpleSpaceCreate(nullptr);
        dSpaceSetCleanup(scene_, 1);
        dSpaceSetCleanup(query_.space, 1);
    }
    void TearDown() override {
        dSpaceDestroy(query_.space);
        dSpaceDestroy(scene_);
        dCloseODE();
    }
    dGeomID Sphere(double x, double y, double z, double radius, bool emitter) {
        auto geometry = dCreateSphere(scene_, radius);
        dGeomSetPosition(geometry, x, y, z);
        if (emitter)
            query_.emitter_collisions.insert(geometry);
        return geometry;
    }
    float Scan(double x, double y, double z) {
        float range = 3;
        auto ray = dCreateRay(query_.space, 3);
        dGeomSetData(ray, &range);
        dGeomRaySet(ray, 0, 0, 0, x, y, z);
        dGeomRaySetFirstContact(ray, 0);
        dGeomRaySetBackfaceCull(ray, 0);
        dGeomRaySetClosestHit(ray, 1);
        dSpaceCollide2(reinterpret_cast<dGeomID>(query_.space), reinterpret_cast<dGeomID>(scene_), &query_,
                       &OdeRayQuery::Intersect);
        dGeomDestroy(ray);
        return range;
    }
    dSpaceID scene_ = nullptr;
    OdeRayQuery query_;
};

TEST_F(NativeOdeRayTest, ExcludesEmitterBodyAndOtherEmitterLinksButRetainsNeighborAndScene) {
    Sphere(.1, 0, 0, .05, true);  // emitting model body
    Sphere(0, .08, 0, .02, true); // another link/rotor of the same model
    Sphere(1, 0, 0, .1, false);   // neighboring model, behind the emitter body
    Sphere(0, 2, 0, .1, false);   // scene object, behind the emitter rotor
    EXPECT_NEAR(Scan(1, 0, 0), .9, 1e-6);
    EXPECT_NEAR(Scan(0, 1, 0), 1.9, 1e-6);
}

TEST_F(NativeOdeRayTest, CloseForeignGeometryIsNotClippedByAnEgoRadius) {
    Sphere(0, -.12, 0, .02, false); // neighboring model inside a hypothetical .25 m cutoff
    Sphere(-.35, 0, 0, .05, false); // static scene geometry
    EXPECT_NEAR(Scan(0, -1, 0), .1, 1e-6);
    EXPECT_NEAR(Scan(-1, 0, 0), .3, 1e-6);
}

TEST_F(NativeOdeRayTest, ModelReplacementRefreshesExclusionsWithoutHidingOldNeighbors) {
    auto old = Sphere(.3, 0, 0, .05, true);
    EXPECT_FLOAT_EQ(Scan(1, 0, 0), 3);
    query_.emitter_collisions.clear();
    Sphere(.1, 0, 0, .02, true);
    EXPECT_NEAR(Scan(1, 0, 0), .25, 1e-6);
    EXPECT_EQ(query_.emitter_collisions.count(old), 0u);
}
} // namespace
} // namespace xgc2_simple_lidar
