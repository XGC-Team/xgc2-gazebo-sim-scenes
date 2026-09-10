#include "xgc2_gazebo_scene/scene_model.hpp"
#include "xgc2_gazebo_scene/convex_mesh_geometry.hpp"
#include "xgc2_gazebo_scene/scene_ownership.hpp"

#include <boost/uuid/detail/sha1.hpp>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/MeshShape.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace xgc2_gazebo_scene {
namespace {

bool Positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

std::string Encode(const std::string& value) {
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (unsigned char byte : value)
        out << std::setw(2) << static_cast<unsigned>(byte);
    return out.str();
}

std::string PoseXml(const ignition::math::Pose3d& pose) {
    std::ostringstream out;
    const auto euler = pose.Rot().Euler();
    // Ignition's vector stream operator rounds coordinates to six decimal
    // places regardless of stream precision; authoring must retain doubles.
    out << std::setprecision(17) << "<pose>" << pose.Pos().X() << " " << pose.Pos().Y() << " " << pose.Pos().Z() << " "
        << euler.X() << " " << euler.Y() << " " << euler.Z() << "</pose>";
    return out.str();
}

std::string MeshFile(const xgc2_geometry_msgs::SceneGeometry& geometry, const std::string& directory) {
    ConvexMeshGeometry mesh;
    for (const auto& vertex : geometry.vertices)
        mesh.vertices.emplace_back(vertex.x, vertex.y, vertex.z);
    if (geometry.triangles.size() % 3 != 0)
        throw std::runtime_error("convex triangle index count must be divisible by 3");
    for (std::size_t i = 0; i < geometry.triangles.size(); i += 3) {
        mesh.triangles.push_back({geometry.triangles[i], geometry.triangles[i + 1], geometry.triangles[i + 2]});
    }
    std::string error;
    if (!ValidateClosedConvexMesh(mesh, &error))
        throw std::runtime_error(error);
    ignition::math::Vector3d center = ignition::math::Vector3d::Zero;
    for (const auto& vertex : mesh.vertices)
        center += vertex;
    center /= mesh.vertices.size();
    std::ostringstream contents;
    contents << std::setprecision(17);
    for (const auto& vertex : mesh.vertices)
        contents << "v " << vertex.X() << " " << vertex.Y() << " " << vertex.Z() << "\n";
    for (auto triangle : mesh.triangles) {
        const auto& a = mesh.vertices[triangle[0]];
        const auto normal = (mesh.vertices[triangle[1]] - a).Cross(mesh.vertices[triangle[2]] - a);
        if (normal.Dot(center - a) > 0.0)
            std::swap(triangle[1], triangle[2]);
        contents << "f " << triangle[0] + 1 << " " << triangle[1] + 1 << " " << triangle[2] + 1 << "\n";
    }
    const std::string bytes = contents.str();
    boost::uuids::detail::sha1 hash;
    hash.process_bytes(bytes.data(), bytes.size());
    unsigned int digest[5];
    hash.get_digest(digest);
    std::ostringstream name;
    name << std::hex << std::setfill('0');
    for (unsigned int word : digest)
        name << std::setw(8) << word;
    std::string path = directory + "/" + name.str() + ".obj";
    std::ifstream existing(path);
    if (existing.good()) {
        const std::string old((std::istreambuf_iterator<char>(existing)), std::istreambuf_iterator<char>());
        if (old != bytes)
            throw std::runtime_error("convex mesh cache content mismatch");
    } else {
        std::ofstream file(path, std::ios::binary);
        file << bytes;
        file.close();
        if (!file)
            throw std::runtime_error("cannot write convex mesh cache");
    }
    // Check Gazebo's actual mesh loader before removing anything from the world.
    if (!LoadConvexMeshGeometry(path, {1, 1, 1}, "", false, &mesh, &error))
        throw std::runtime_error(error);
    return path;
}

std::string GeometryXml(const SceneCollision& collision) {
    const auto& g = collision.geometry;
    std::ostringstream out;
    out << std::setprecision(17) << "<geometry>";
    if (g.type == "box") {
        out << "<box><size>" << g.size.x << " " << g.size.y << " " << g.size.z << "</size></box>";
    } else if (g.type == "sphere") {
        out << "<sphere><radius>" << g.radius << "</radius></sphere>";
    } else if (g.type == "cylinder") {
        out << "<cylinder><radius>" << g.radius << "</radius><length>" << g.height << "</length></cylinder>";
    } else if (g.type == "convex") {
        out << "<mesh><uri>" << collision.mesh_uri << "</uri><scale>1 1 1</scale></mesh>";
    } else {
        throw std::runtime_error("unsupported collision geometry: " + g.type);
    }
    out << "</geometry>";
    return out.str();
}

void AppendCollision(SceneCollision collision, const std_msgs::ColorRGBA& color, SceneModel* model,
                     std::ostringstream* body) {
    const auto geometry = GeometryXml(collision);
    *body << "<collision name='" << collision.name << "'>" << PoseXml(collision.pose) << geometry << "</collision>";
    *body << "<visual name='" << collision.name << "'>" << PoseXml(collision.pose) << geometry << "<material><ambient>"
          << color.r << " " << color.g << " " << color.b << " " << color.a << "</ambient><diffuse>" << color.r << " "
          << color.g << " " << color.b << " " << color.a << "</diffuse></material><transparency>" << 1.0 - color.a
          << "</transparency></visual>";
    model->collisions.push_back(std::move(collision));
}

bool Near(double a, double b) {
    return std::abs(a - b) <= 1.0e-6 * std::max({1.0, std::abs(a), std::abs(b)});
}

bool SamePose(const ignition::math::Pose3d& a, const ignition::math::Pose3d& b) {
    // q and -q describe the same orientation.
    return a.Pos().Equal(b.Pos(), 1.0e-6) && std::abs(std::abs(a.Rot().Dot(b.Rot())) - 1.0) < 1.0e-8;
}

} // namespace

std::string SceneModelName(const std::string& id) {
    return std::string(kSceneRuntimeModelPrefix) + Encode(id);
}

bool ValidateScenePose(const geometry_msgs::Pose& pose) {
    const auto& p = pose.position;
    const auto& q = pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) &&
           std::abs(norm - 1.0) <= 1.0e-6;
}

ignition::math::Pose3d ScenePose(const geometry_msgs::Pose& pose) {
    return {{pose.position.x, pose.position.y, pose.position.z},
            {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z}};
}

bool CompileScene(const xgc2_geometry_msgs::SceneSnapshot& scene, const std::string& mesh_directory,
                  std::map<std::string, SceneModel>* models, std::string* error) {
    models->clear();
    error->clear();
    try {
        if (scene.epoch.empty() || scene.scene_id.empty())
            throw std::runtime_error("scene ID and epoch are required");
        if (scene.header.frame_id != "world")
            throw std::runtime_error("Gazebo scene frame must be world");
        for (const auto& obstacle : scene.obstacles) {
            if (obstacle.id.empty() || obstacle.id.size() > 256 || models->count(obstacle.id)) {
                throw std::runtime_error("obstacle ID must be unique, nonempty and at most 256 bytes: " + obstacle.id);
            }
            if (!ValidateScenePose(obstacle.pose))
                throw std::runtime_error("invalid obstacle pose: " + obstacle.id);
            if (obstacle.parts.empty())
                throw std::runtime_error("obstacle has no parts: " + obstacle.id);
            SceneModel model;
            model.id = obstacle.id;
            model.name = SceneModelName(obstacle.id);
            model.pose = ScenePose(obstacle.pose);
            std::ostringstream body;
            body << std::setprecision(17) << "<static>true</static><link name='body'><gravity>false</gravity>";
            std::set<std::string> parts;
            for (const auto& part : obstacle.parts) {
                if (part.id.empty() || part.id.size() > 256 || !parts.insert(part.id).second) {
                    throw std::runtime_error("part ID must be unique, nonempty and at most 256 bytes: " + obstacle.id);
                }
                if (!ValidateScenePose(part.pose))
                    throw std::runtime_error("invalid part pose: " + part.id);
                for (float channel : {part.color.r, part.color.g, part.color.b, part.color.a}) {
                    if (!std::isfinite(channel) || channel < 0.0 || channel > 1.0)
                        throw std::runtime_error("invalid part color");
                }
                SceneCollision collision;
                collision.name = "part_" + Encode(part.id);
                collision.pose = ScenePose(part.pose);
                collision.geometry = part.geometry;
                const auto& g = part.geometry;
                if (g.type == "box") {
                    if (!Positive(g.size.x) || !Positive(g.size.y) || !Positive(g.size.z))
                        throw std::runtime_error("box requires positive side lengths");
                } else if (g.type == "sphere") {
                    if (!Positive(g.radius))
                        throw std::runtime_error("sphere requires positive radius");
                } else if (g.type == "cylinder" || g.type == "capsule") {
                    if (!Positive(g.radius) || !std::isfinite(g.height) || g.height < 0.0 ||
                        (g.type == "cylinder" && g.height == 0.0))
                        throw std::runtime_error(g.type + " requires positive radius and valid height");
                } else if (g.type == "convex") {
                    collision.mesh_uri = MeshFile(g, mesh_directory);
                } else {
                    throw std::runtime_error("unsupported scene geometry: " + g.type);
                }
                if (g.type == "capsule") {
                    if (g.height == 0.0) {
                        // A zero straight section is exactly one sphere.
                        collision.geometry.type = "sphere";
                        AppendCollision(collision, part.color, &model, &body);
                        continue;
                    }
                    // Exact set union, with the declared height referring only to
                    // the straight section. Keep all three collisions in one part.
                    collision.geometry.type = "cylinder";
                    collision.name += "_shaft";
                    AppendCollision(collision, part.color, &model, &body);
                    for (const int sign : {-1, 1}) {
                        collision.geometry.type = "sphere";
                        collision.name = "part_" + Encode(part.id) + (sign < 0 ? "_lower" : "_upper");
                        collision.pose =
                            ScenePose(part.pose) * ignition::math::Pose3d(0, 0, sign * g.height * 0.5, 0, 0, 0);
                        AppendCollision(collision, part.color, &model, &body);
                    }
                } else {
                    AppendCollision(collision, part.color, &model, &body);
                }
            }
            body << "</link>";
            model.body = body.str();
            model.sdf = "<sdf version='1.6'><model name='" + model.name + "'>" + PoseXml(model.pose) + model.body +
                        "</model></sdf>";
            sdf::SDFPtr parsed(new sdf::SDF());
            sdf::init(parsed);
            if (!sdf::readString(model.sdf, parsed))
                throw std::runtime_error("Gazebo rejected compiled scene SDF");
            models->emplace(model.id, std::move(model));
        }
        return true;
    } catch (const std::exception& exception) {
        *error = exception.what();
        models->clear();
        return false;
    }
}

bool ApplySceneModelParameters(const gazebo::physics::ModelPtr& model, const SceneModel& expected,
                               const ignition::math::Pose3d& current_pose) {
    if (!model)
        return false;
    boost::recursive_mutex::scoped_lock lock(*model->GetWorld()->Physics()->GetPhysicsUpdateMutex());
    const auto link = model->GetLink("body");
    if (!link || link->GetCollisions().size() != expected.collisions.size())
        return false;
    sdf::SDFPtr parsed(new sdf::SDF());
    sdf::init(parsed);
    if (!sdf::readString(expected.sdf, parsed))
        return false;
    // UpdateParameters updates visual definitions and their Gazebo messages too.
    model->UpdateParameters(parsed->Root()->GetElement("model"));
    model->SetInitialRelativePose(expected.pose);
    model->SetWorldPose(current_pose);
    for (const auto& part : expected.collisions) {
        const auto collision = link->GetCollision(part.name);
        if (!collision)
            return false;
        collision->SetInitialRelativePose(part.pose);
        collision->SetRelativePose(part.pose);
        const auto& g = part.geometry;
        const auto shape = collision->GetShape();
        const auto set_value = [&](const std::string& key, const std::string& value) {
            shape->GetSDF()->GetElement(key)->GetValue()->SetFromString(value);
        };
        std::ostringstream dimensions;
        dimensions << std::setprecision(17);
        if (g.type == "box") {
            const auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape);
            if (!box)
                return false;
            box->SetSize({g.size.x, g.size.y, g.size.z});
            dimensions << g.size.x << " " << g.size.y << " " << g.size.z;
            set_value("size", dimensions.str());
        } else if (g.type == "sphere") {
            const auto sphere = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(shape);
            if (!sphere)
                return false;
            sphere->SetRadius(g.radius);
            dimensions << g.radius;
            set_value("radius", dimensions.str());
        } else if (g.type == "cylinder") {
            const auto cylinder = boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(shape);
            if (!cylinder)
                return false;
            cylinder->SetSize(g.radius, g.height);
            dimensions << g.radius;
            set_value("radius", dimensions.str());
            dimensions.str("");
            dimensions << g.height;
            set_value("length", dimensions.str());
        }
    }
    return true;
}

bool VerifySceneModel(const gazebo::physics::ModelPtr& model, const SceneModel& expected, bool verify_pose,
                      std::string* error) {
    const auto fail = [&](const std::string& reason) {
        *error = expected.id + ": " + reason;
        return false;
    };
    if (!model)
        return fail("model is absent");
    if (!model->IsStatic())
        return fail("scene model is not anchored");
    if (verify_pose && !SamePose(model->WorldPose(), expected.pose)) {
        std::ostringstream details;
        details << std::setprecision(17) << "model pose differs: actual [" << model->WorldPose().Pos().X() << ", "
                << model->WorldPose().Pos().Y() << ", " << model->WorldPose().Pos().Z() << "] expected ["
                << expected.pose.Pos().X() << ", " << expected.pose.Pos().Y() << ", " << expected.pose.Pos().Z()
                << "], quaternion dot " << model->WorldPose().Rot().Dot(expected.pose.Rot());
        return fail(details.str());
    }
    const auto link = model->GetLink("body");
    if (!link || model->GetLinks().size() != 1 || link->GetCollisions().size() != expected.collisions.size()) {
        return fail("collision part count differs");
    }
    std::set<std::string> visual_names;
    auto link_sdf = link->GetSDF();
    if (link_sdf->HasElement("visual")) {
        for (auto visual = link_sdf->GetElement("visual"); visual; visual = visual->GetNextElement("visual")) {
            visual_names.insert(visual->Get<std::string>("name"));
        }
    }
    if (visual_names.size() != expected.collisions.size())
        return fail("visual part count differs");
    for (const auto& part : expected.collisions) {
        const auto collision = link->GetCollision(part.name);
        if (!collision || !visual_names.count(part.name))
            return fail("missing collision or visual " + part.name);
        if (!SamePose(collision->RelativePose(), part.pose))
            return fail("part pose differs: " + part.name);
        const auto shape = collision->GetShape();
        const auto& geometry = part.geometry;
        bool matches = false;
        if (geometry.type == "box") {
            const auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape);
            matches = box && box->Size().Equal({geometry.size.x, geometry.size.y, geometry.size.z}, 1.0e-6);
        } else if (geometry.type == "sphere") {
            const auto sphere = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(shape);
            matches = sphere && Near(sphere->GetRadius(), geometry.radius);
        } else if (geometry.type == "cylinder") {
            const auto cylinder = boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(shape);
            matches = cylinder && Near(cylinder->GetRadius(), geometry.radius) &&
                      Near(cylinder->GetLength(), geometry.height);
        } else if (geometry.type == "convex") {
            const auto mesh = boost::dynamic_pointer_cast<gazebo::physics::MeshShape>(shape);
            matches = mesh && mesh->GetMeshURI() == part.mesh_uri && mesh->Scale().Equal({1, 1, 1}, 1.0e-6);
            if (matches) {
                const auto bounds = collision->BoundingBox().Size();
                matches = Positive(bounds.X()) && Positive(bounds.Y()) && Positive(bounds.Z());
            }
        }
        if (!matches)
            return fail("runtime collision geometry differs: " + part.name);
    }
    error->clear();
    return true;
}

} // namespace xgc2_gazebo_scene
