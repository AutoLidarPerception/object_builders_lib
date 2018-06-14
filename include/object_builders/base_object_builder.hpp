#ifndef _BASE_OBJECT_BUILDER_HPP_
#define _BASE_OBJECT_BUILDER_HPP_

#include "common/types/type.h"
#include "common/types/object.hpp"

namespace object_builder {

    class BaseObjectBuilder {

    public:
        virtual void build(const std::vector<PointICloudPtr>& cloud_clusters, std::vector<ObjectPtr>* objects) = 0;

        virtual void build(ObjectPtr object) = 0;

        virtual std::string name() const = 0;
    }; /* class BaseObjectBuilder */
}

#endif /* _BASE_OBJECT_BUILDER_HPP_ */