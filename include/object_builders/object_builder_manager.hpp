/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_OBJECT_BUILDER_MANAGER_HPP_
#define OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_OBJECT_BUILDER_MANAGER_HPP_

#include <memory>

#include "object_builders/base_object_builder.hpp"
#include "object_builders/min_box_object_builder.hpp"

namespace autosense {
namespace object_builder {

static std::unique_ptr<BaseObjectBuilder> createObjectBuilder() {
    std::unique_ptr<BaseObjectBuilder> builder;
    builder = std::unique_ptr<BaseObjectBuilder>(new MinBoxObjectBuilder);
    return builder;
}

}  // namespace object_builder
}  // namespace autosense

#endif  // OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_OBJECT_BUILDER_MANAGER_HPP_
