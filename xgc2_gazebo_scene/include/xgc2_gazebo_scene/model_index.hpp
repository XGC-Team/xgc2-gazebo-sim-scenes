#pragma once

#include "xgc2_gazebo_scene/model_snapshot.hpp"

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace xgc2_gazebo_scene {

/// The models of a world by name, rebuilt only when the world's model list
/// changes.
///
/// World::ModelByName holds the world's model-loading mutex while it walks the
/// whole entity tree, copying every scoped name and name on the way, and a name
/// that is not there costs the whole walk. The scene adapter asks for each of
/// its obstacles by name in every pass of its 10 ms wait loops, which also
/// keeps the factory that is loading those obstacles waiting for the mutex, and
/// in every state message and heartbeat. One Refresh() per pass takes the
/// world's model list (World::Models(), the models at the root of the world)
/// and rebuilds the index only when the list differs from the one it was built
/// from: a model added, removed, reordered, renamed (a retired scene body is
/// renamed before it is removed) or made static or dynamic. Find() is then a
/// hash lookup.
///
/// Find() answers like ModelByName for every name that is not also the name of
/// another kind of entity (a link, joint or collision) or of a model nested in
/// another model. The names the scene adapter looks up carry its reserved
/// prefixes (kSceneRuntimeModelPrefix, kSceneRetiredModelPrefix), which no
/// other entity uses. A name listed twice answers with the first model, as the
/// walk does. Models are held weakly, so a removed model is not kept alive.
///
/// `ModelPtr` is a shared pointer to a type with GetName() and IsStatic();
/// `WeakPtr` is its weak pointer.
template <class ModelPtr, class WeakPtr> class ModelIndex {
  public:
    /// Rebuilds the index if `models` is not the list it was built from.
    /// Returns whether it was rebuilt.
    bool Refresh(const std::vector<ModelPtr>& models) {
        if (snapshot_.Matches(models))
            return false;
        snapshot_.Update(models);
        by_name_.clear();
        by_name_.reserve(models.size());
        for (const auto& model : models)
            by_name_.emplace(model->GetName(), WeakPtr(model));
        return true;
    }

    /// The model called `name` in the list given to the last Refresh(), or an
    /// empty pointer.
    ModelPtr Find(const std::string& name) const {
        const auto found = by_name_.find(name);
        return found == by_name_.end() ? ModelPtr() : found->second.lock();
    }

  private:
    ModelSnapshot<ModelPtr, WeakPtr> snapshot_;
    std::unordered_map<std::string, WeakPtr> by_name_;
};

/// Whether none of `names` is in the world: not in `index` (as last refreshed)
/// and, for a name the index no longer lists, not found by `walk` either.
/// `walk(name)` is World::ModelByName; it is asked once per name, and `confirmed`
/// remembers the names it has said are gone.
///
/// Gazebo's World::RemoveModel takes a model off the model list first, then
/// destroys its entities, collisions in the physics engine included, under the
/// physics update mutex. ModelByName finds the model until that is done. A
/// retired scene body counts as removed only then, so the index alone, which
/// forgets it a moment earlier, is not enough for it.
template <class ModelPtr, class WeakPtr, class Names, class Walk>
bool AllGone(const ModelIndex<ModelPtr, WeakPtr>& index, const Names& names, Walk&& walk,
             std::set<std::string>* confirmed) {
    for (const auto& name : names) {
        if (index.Find(name))
            return false;
        if (confirmed->count(name) != 0)
            continue;
        if (walk(name))
            return false;
        confirmed->insert(name);
    }
    return true;
}

} // namespace xgc2_gazebo_scene
