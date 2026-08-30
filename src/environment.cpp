#include "fsoc/environment.hpp"

namespace fsoc {

std::optional<Projection> Environment::observe_ideal() const noexcept {
    return camera_.project(target_.position_m);
}

}  // namespace fsoc
