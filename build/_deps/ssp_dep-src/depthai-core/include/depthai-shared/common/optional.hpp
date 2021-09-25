#pragma once

#include <nlohmann/json.hpp>

#include "tl/optional.hpp"

// partial specialization (full specialization works too)
namespace nlohmann {
template <typename T>
struct adl_serializer<tl::optional<T>> {
    static void to_json(json& j, const tl::optional<T>& opt) {  // NOLINT this is a specialization, naming conventions don't apply
        if(opt == tl::nullopt) {
            j = nullptr;
        } else {
            j = *opt;  // this will call adl_serializer<T>::to_json which will
                       // find the free function to_json in T's namespace!
        }
    }

    static void from_json(const json& j, tl::optional<T>& opt) {  // NOLINT this is a specialization, naming conventions don't apply
        if(j.is_null()) {
            opt = tl::nullopt;
        } else {
            opt = j.get<T>();  // same as above, but with
                               // adl_serializer<T>::from_json
        }
    }
};
}  // namespace nlohmann