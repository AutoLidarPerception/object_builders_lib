/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_MIN_BOX_OBJECT_BUILDER_HPP_
#define OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_MIN_BOX_OBJECT_BUILDER_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "common/types/object.hpp"
#include "object_builders/base_object_builder.hpp"

namespace autosense {
namespace object_builder {
/**
 * @brief Apollo doc
 *  盒构建器将恢复给定多边形点的完整边界框, 即使点云稀疏
 *  主要目的还是预估障碍物（例如，车辆）的方向
 */
class MinBoxObjectBuilder : public BaseObjectBuilder {
 public:
    MinBoxObjectBuilder() {}

    virtual ~MinBoxObjectBuilder() {}

    virtual void build(const std::vector<PointICloudPtr> &cloud_clusters,
                       std::vector<ObjectPtr> *objects);

    virtual void build(ObjectPtr object);

    virtual std::string name() const { return "MinBoxObjectBuilder"; }

 protected:
    bool buildObjects(std::vector<ObjectPtr> *objects);

    void buildObject(ObjectPtr object);

    void setDefaultValue(PointICloudPtr cloud,
                         ObjectPtr obj,
                         Eigen::Vector4f *min_pt,
                         Eigen::Vector4f *max_pt);

    void computeGeometricFeature(ObjectPtr obj);

    void computePolygon2dxy(ObjectPtr obj);

    void reconstructPolygon3d(ObjectPtr obj);

    double computeAreaAlongOneEdge(ObjectPtr obj,
                                   size_t first_in_point,
                                   Eigen::Vector3d *center,
                                   double *length,
                                   double *width,
                                   Eigen::Vector3d *dir);
};

}  // namespace object_builder
}  // namespace autosense

#endif  // OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_MIN_BOX_OBJECT_BUILDER_HPP_
