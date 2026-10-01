#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {

/// The part of the world's model list that the scene plugin derives state
/// from: each model object, its name and its static flag, in world order.
///
/// Rebuilding the contact table and rediscovering managed obstacles costs a
/// map insertion and several string copies per model. Matches() costs one
/// name copy and a few comparisons per model, so the plugin rebuilds only
/// when the list actually changed: a model added, removed, reordered,
/// renamed (a retired scene body is renamed before it is removed) or made
/// static or dynamic.
///
/// `ModelPtr` is a shared pointer to a type with GetName() and IsStatic();
/// `WeakPtr` is its weak pointer. The weak reference makes an equal address
/// conclusive: a model freed and replaced at the same address has expired.
template <class ModelPtr, class WeakPtr> class ModelSnapshot {
  public:
    bool Matches(const std::vector<ModelPtr>& models) const {
        if (models.size() != entries_.size())
            return false;
        for (std::size_t index = 0; index < models.size(); ++index) {
            const Entry& entry = entries_[index];
            const ModelPtr& model = models[index];
            if (model.get() != entry.address || entry.model.expired() || model->IsStatic() != entry.is_static ||
                model->GetName() != entry.name)
                return false;
        }
        return true;
    }

    void Update(const std::vector<ModelPtr>& models) {
        entries_.clear();
        entries_.reserve(models.size());
        for (const auto& model : models)
            entries_.push_back(Entry{model.get(), WeakPtr(model), model->GetName(), model->IsStatic()});
    }

  private:
    struct Entry {
        const void* address;
        WeakPtr model;
        std::string name;
        bool is_static;
    };
    std::vector<Entry> entries_;
};

} // namespace xgc2_gazebo_scene
