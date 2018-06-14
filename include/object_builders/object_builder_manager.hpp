#ifndef _OBJECT_BUILDER_MANAGER_HPP_
#define _OBJECT_BUILDER_MANAGER_HPP_

#include "./base_object_builder.hpp"
#include "./min_box_object_builder.hpp"

namespace object_builder {

    static std::unique_ptr<BaseObjectBuilder> createObjectBuilder()
    {
        std::unique_ptr<BaseObjectBuilder> builder;
        builder = std::unique_ptr<BaseObjectBuilder>(new MinBoxObjectBuilder);
        return builder;
    }
}

#endif /* _OBJECT_BUILDER_MANAGER_HPP_ */