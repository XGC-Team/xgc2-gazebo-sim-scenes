#pragma once

#include <sdf/sdf.hh>

#include <string>

namespace xgc2_gazebo_scene {

/// Parses model SDF strings with libsdformat.
///
/// A new sdf::SDF must be sdf::init()ed before it can read a string, and
/// sdf::init() parses the whole SDF specification: about 25 ms, against about
/// 2 ms for the string. The scene adapter parsed every obstacle twice per
/// application, once to validate it and once to restore its parameters with
/// the physics update mutex held, each time in a new sdf::SDF, so 500 obstacles
/// meant 25 seconds of specification parsing. The parser reads the
/// specification once, on the first Parse(), and clears the previous elements
/// before each string, as Gazebo's own factory does with its persistent SDF.
/// The parse is the same: libsdformat describes each element from the
/// specification, which parsing a string leaves alone.
///
/// Not thread safe; one parser per thread.
class ModelSdfParser {
  public:
    /// Whether libsdformat accepts `sdf_string`. Model() is then its <model>
    /// element; after a rejected string it is empty.
    bool Parse(const std::string& sdf_string) {
        if (!sdf_) {
            sdf_.reset(new sdf::SDF());
            sdf::init(sdf_);
        }
        sdf_->Clear();
        model_.reset();
        if (!sdf::readString(sdf_string, sdf_))
            return false;
        model_ = sdf_->Root()->GetElement("model");
        return true;
    }

    /// The <model> element of the last accepted string. It lives until the next
    /// Parse(), which replaces it.
    sdf::ElementPtr Model() const { return model_; }

  private:
    sdf::SDFPtr sdf_;
    sdf::ElementPtr model_;
};

} // namespace xgc2_gazebo_scene
