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

#include "object_builders/min_box_object_builder.hpp"

#include <pcl/common/common.h>      // pcl::getMinMax3D
#include <pcl/common/transforms.h>  // pcl::TransformPointCloud
#include <pcl/io/io.h>              // pcl::copyPointCloud

#include "common/algos/convex_hullxy.hpp"  // common::algos::ConvexHull2DXY
#include "common/common.hpp"               // common::convertPointCloud */
#include "common/geometry.hpp"  // common::geometry::computeTheta2dXyBetweenVectors
#include "common/time.hpp"      // common::Clock

namespace autosense {
namespace object_builder {

void MinBoxObjectBuilder::build(
    const std::vector<PointICloudPtr> &cloud_clusters,
    std::vector<ObjectPtr> *objects) {
    if (objects == nullptr) {
        return;
    }

    (*objects).clear();

    for (size_t idx = 0u; idx < cloud_clusters.size(); ++idx) {
        ObjectPtr obj(new Object);
        *(obj->cloud) = *cloud_clusters[idx];
        buildObject(obj);
        (*objects).push_back(obj);
    }
}

void MinBoxObjectBuilder::build(ObjectPtr object) { buildObject(object); }

void MinBoxObjectBuilder::setDefaultValue(PointICloudPtr cloud,
                                          ObjectPtr obj,
                                          Eigen::Vector4f *min_pt,
                                          Eigen::Vector4f *max_pt) {
    pcl::getMinMax3D(*cloud, *min_pt, *max_pt);
    Eigen::Vector3f center(((*min_pt)[0] + (*max_pt)[0]) / 2,
                           ((*min_pt)[1] + (*max_pt)[1]) / 2,
                           ((*min_pt)[2] + (*max_pt)[2]) / 2);

    // handle degeneration case 退化情况
    float epslin = 1e-3;
    for (int i = 0; i < 3; i++) {
        if ((*max_pt)[i] - (*min_pt)[i] < epslin) {
            (*max_pt)[i] = center[i] + epslin / 2;
            (*min_pt)[i] = center[i] - epslin / 2;
        }
    }

    // length
    obj->length = (*max_pt)[0] - (*min_pt)[0];
    // width
    obj->width = (*max_pt)[1] - (*min_pt)[1];
    if (obj->length - obj->width < 0) {
        float tmp = obj->length;
        obj->length = obj->width;
        obj->width = tmp;
        obj->direction = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else {
        obj->direction = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    // height
    obj->height = (*max_pt)[2] - (*min_pt)[2];
    // center
    obj->ground_center = Eigen::Vector3d(((*max_pt)[0] + (*min_pt)[0]) / 2,
                                         ((*max_pt)[1] + (*min_pt)[1]) / 2,
                                         ((*max_pt)[2] + (*min_pt)[2]) / 2);
    // polygon
    if (cloud->size() < 4) {
        obj->polygon.points.resize(4);
        obj->polygon.points[0].x = static_cast<double>((*min_pt)[0]);
        obj->polygon.points[0].y = static_cast<double>((*min_pt)[1]);
        obj->polygon.points[0].z = static_cast<double>((*min_pt)[2]);

        obj->polygon.points[1].x = static_cast<double>((*max_pt)[0]);
        obj->polygon.points[1].y = static_cast<double>((*min_pt)[1]);
        obj->polygon.points[1].z = static_cast<double>((*min_pt)[2]);

        obj->polygon.points[2].x = static_cast<double>((*max_pt)[0]);
        obj->polygon.points[2].y = static_cast<double>((*max_pt)[1]);
        obj->polygon.points[2].z = static_cast<double>((*min_pt)[2]);

        obj->polygon.points[3].x = static_cast<double>((*min_pt)[0]);
        obj->polygon.points[3].y = static_cast<double>((*max_pt)[1]);
        obj->polygon.points[3].z = static_cast<double>((*min_pt)[2]);
    }
}

/*
 * @brief: calc object feature, and fill fields.
 * @param [in]: options.
 * @param [in/out]: object list.
 */
bool MinBoxObjectBuilder::buildObjects(std::vector<ObjectPtr> *objects) {
    if (objects == NULL) {
        return false;
    }
    for (size_t i = 0; i < objects->size(); ++i) {
        if ((*objects)[i]) {
            (*objects)[i]->id = static_cast<IdType>(i);
            buildObject((*objects)[i]);
        }
    }
    return true;
}

// object: 当前帧检测到的一个障碍物: 障碍物分类/包含的点云等信息
/*
    // convex hull of the object
    PolygonDType polygon;

    // oriented boundingbox information
    // main direction
    Eigen::Vector3d direction;
    // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
    double theta = 0.0;
    // ground center of the object (cx, cy, z_min)
    Eigen::Vector3d center;
    // size of the oriented bbox, length is the size in the main direction
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
 */
// built took ~3.5ms for one object
void MinBoxObjectBuilder::buildObject(ObjectPtr object) {
    computeGeometricFeature(object);

    /*
     * @note Apollo's Object Coordinate
     *              |x
     *              |
     *              |
     *              |
     *              |
     * y-------------
     * @note Apollo's yaw: direction is a 2d vector from ground center
     * if (fabs(obj->direction[0]) < DBL_MIN) {
            obj->theta = obj->direction(1) > 0 ? M_PI / 2 : -M_PI / 2;
        } else {
            obj->theta = atan2(obj->direction[1], obj->direction[0]);
        }
     */
    /*if (fabs(object->direction[0]) < DBL_MIN) {
        object->yaw_rad = object->direction(1) > 0 ? M_PI / 2 : -M_PI / 2;
    } else {
        object->yaw_rad = atan2(object->direction[1], object->direction[0]);
    }*/
    /**
     * TODO(gary):
        Eigen::Vector3f coord_dir(0.0, 1.0, 0.0);
        double theta = VectorTheta2dXy(coord_dir, obj->direction);
     */
    Eigen::Vector3d coord_dir(1.0, 0.0, 0.0);
    object->yaw_rad =
        common::geometry::computeTheta2dXyBetweenVectors<Eigen::Vector3d>(
            coord_dir, object->direction);
}

/**
 * @brief Build object obj
 *  computePolygon2dxy(obj)
 *  reconstructPolygon3d(obj)
 * @param obj
 */
void MinBoxObjectBuilder::computeGeometricFeature(ObjectPtr obj) {
    // 得到最低点(min_pt.z) 上的2D边框: PolygonDType
    computePolygon2dxy(obj);
    // 障碍物3D boundingbox(航向角表示方向/地表中心点+长宽高表示空间位置)
    reconstructPolygon3d(obj);
}

/**
 * @brief 得到最低点(min_pt.z) 上的2D多边形边框,构建点云分割多边形凸包
 * PolygonDType polygon;
 * @result
 *  ObjectPtr->height = max_z - min_z
 *  ObjectPtr->polygon Convex Hull of the object(物体凸边框) projected into
 * xy-plane
 */
void MinBoxObjectBuilder::computePolygon2dxy(ObjectPtr obj) {
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    PointICloudPtr cloud = obj->cloud;
    setDefaultValue(cloud, obj, &min_pt, &max_pt);

    if (cloud->points.size() < 4u /* unsigned 4 */) {
        return;
    }
    // 直接获取点云 min_pt.x/y/z; max_pt.x/y/z
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    // 高度信息
    obj->height =
        static_cast<double>(max_pt[2]) - static_cast<double>(min_pt[2]);

    // epsilon: the difference between 1.0 and the next value representable by
    // the
    // floating-point type
    const double min_eps = 10 * std::numeric_limits<double>::epsilon();
    // double min_eps = 0.1;
    // if ((max_pt[0] - min_pt[0]) < min_eps) {
    //     _cloud->points[0].x += min_eps;
    // }
    // if ((max_pt[1] - min_pt[1]) < min_eps) {
    //     _cloud->points[0].y += min_eps;
    // }
    const double diff_x = cloud->points[1].x - cloud->points[0].x;
    const double diff_y = cloud->points[1].y - cloud->points[0].y;
    size_t idx = 0;
    for (idx = 2; idx < cloud->points.size(); ++idx) {
        const double tdiff_x = cloud->points[idx].x - cloud->points[0].x;
        const double tdiff_y = cloud->points[idx].y - cloud->points[0].y;
        if ((diff_x * tdiff_y - tdiff_x * diff_y) > min_eps) {
            break;
        }
    }
    if (idx >= cloud->points.size()) {
        cloud->points[0].x += min_eps;
        cloud->points[0].y += min_eps;
        cloud->points[1].x -= min_eps;
    }
    // Project point cloud into xy-plane
    PointICloudPtr cloud_xy(new PointICloud);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        PointI p = cloud->points[i];
        p.z = min_pt[2];
        cloud_xy->push_back(p);
    }
    // Get the Convex Hull of the xy-plane point cloud
    // TODO(gary): Apollo's pcl::ConvexHull is memory-efficient?
    common::algos::ConvexHull2DXY<PointI> hull;
    // pcl::ConvexHull<PointI> hull;
    hull.setInputCloud(cloud_xy);
    hull.setDimension(2);
    std::vector<pcl::Vertices> poly_vt;
    PointICloudPtr plane_hull(new PointICloud);
    hull.Reconstruct2dxy(plane_hull, &poly_vt);
    // hull.reconstruct(*plane_hull, poly_vt);

    // a valid Convex Polygon
    if (poly_vt.size() == 1u) {
        std::vector<int> indices(poly_vt[0].vertices.begin(),
                                 poly_vt[0].vertices.end());
        // Get Polygon vertices cloud from Convex Hull cloud
        common::convertPointCloud(plane_hull, indices, &obj->polygon);
        // cannot find a valid Convex Polygon
    } else {
        /*
         * 2D bounding box
         *                  |x
         *      C       D   |
         *                  |
         *                  |
         *      B       A   |
         * y----------------|
         */
        obj->polygon.points.resize(4);
        obj->polygon.points[0].x = static_cast<double>(min_pt[0]);
        obj->polygon.points[0].y = static_cast<double>(min_pt[1]);
        obj->polygon.points[0].z = static_cast<double>(min_pt[2]);

        obj->polygon.points[1].x = static_cast<double>(min_pt[0]);
        obj->polygon.points[1].y = static_cast<double>(max_pt[1]);
        obj->polygon.points[1].z = static_cast<double>(min_pt[2]);

        obj->polygon.points[2].x = static_cast<double>(max_pt[0]);
        obj->polygon.points[2].y = static_cast<double>(max_pt[1]);
        obj->polygon.points[2].z = static_cast<double>(min_pt[2]);

        obj->polygon.points[3].x = static_cast<double>(max_pt[0]);
        obj->polygon.points[3].y = static_cast<double>(min_pt[1]);
        obj->polygon.points[3].z = static_cast<double>(min_pt[2]);
    }
}

/**
 * @brief 以多边形凸包最长边出发, 选择最小面积Ground Box构建3D OBB
 * @param obj
 */
void MinBoxObjectBuilder::reconstructPolygon3d(ObjectPtr obj) {
    if (obj->polygon.points.size() <= 0) {
        return;
    }
    // <1> Find the longest line in Convex Hull
    size_t max_point_index = 0;
    size_t min_point_index = 0;
    Eigen::Vector3d p;
    p[0] = obj->polygon.points[0].x;
    p[1] = obj->polygon.points[0].y;
    p[2] = obj->polygon.points[0].z;
    Eigen::Vector3d max_point = p;
    Eigen::Vector3d min_point = p;
    for (size_t i = 1; i < obj->polygon.points.size(); ++i) {
        Eigen::Vector3d p;
        p[0] = obj->polygon.points[i].x;
        p[1] = obj->polygon.points[i].y;
        p[2] = obj->polygon.points[i].z;
        Eigen::Vector3d ray = p;
        // clock direction
        if (max_point[0] * ray[1] - ray[0] * max_point[1] < common::EPSILON) {
            max_point = ray;
            max_point_index = i;
        }
        // anti-clock direction
        if (min_point[0] * ray[1] - ray[0] * min_point[1] > common::EPSILON) {
            min_point = ray;
            min_point_index = i;
        }
    }
    Eigen::Vector3d line = max_point - min_point;

    // TODO(gary): <2>???
    // 其它顶点投影到最长边得到矩形边框长边边长(total_len),宽边边长(max_dis)
    double total_len = 0;
    double max_dis = 0;
    bool has_out = false;
    for (size_t i = min_point_index, count = 0;
         count < obj->polygon.points.size();
         i = (i + 1) % obj->polygon.points.size(), ++count) {
        Eigen::Vector3d p_x;
        p_x[0] = obj->polygon.points[i].x;
        p_x[1] = obj->polygon.points[i].y;
        p_x[2] = obj->polygon.points[i].z;
        size_t j = (i + 1) % obj->polygon.points.size();
        //确保构成最长边之外的其它边
        if (j != min_point_index && j != max_point_index) {
            Eigen::Vector3d p;
            p[0] = obj->polygon.points[j].x;
            p[1] = obj->polygon.points[j].y;
            p[2] = obj->polygon.points[j].z;
            Eigen::Vector3d ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < common::EPSILON) {
                double dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                                   (p[1] - p_x[1]) * (p[1] - p_x[1]));
                total_len += dist;
                if (dist - max_dis > common::EPSILON) {
                    max_dis = dist;
                }
            } else {
                // outline
                has_out = true;
            }
        } else if ((i == min_point_index && j == max_point_index) ||
                   (i == max_point_index && j == min_point_index)) {
            size_t k = (j + 1) % obj->polygon.points.size();
            Eigen::Vector3d p_k;
            p_k[0] = obj->polygon.points[k].x;
            p_k[1] = obj->polygon.points[k].y;
            p_k[2] = obj->polygon.points[k].z;
            Eigen::Vector3d p_j;
            p_j[0] = obj->polygon.points[j].x;
            p_j[1] = obj->polygon.points[j].y;
            p_j[2] = obj->polygon.points[j].z;
            Eigen::Vector3d ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
            } else {
                // outline
                has_out = true;
            }
        } else if (j == min_point_index || j == max_point_index) {
            Eigen::Vector3d p;
            p[0] = obj->polygon.points[j].x;
            p[1] = obj->polygon.points[j].y;
            p[2] = obj->polygon.points[j].z;
            Eigen::Vector3d ray = p_x - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < common::EPSILON) {
                double dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                                   (p[1] - p_x[1]) * (p[1] - p_x[1]));
                total_len += dist;
                if (dist > max_dis) {
                    max_dis = dist;
                }
            } else {
                // outline
                has_out = true;
            }
        }
    }

    // <3> 求解所有以最长边投影的可能矩形中面积最小的那个
    size_t count = 0;
    double min_area = std::numeric_limits<double>::max();
    for (size_t i = min_point_index; count < obj->polygon.points.size();
         i = (i + 1) % obj->polygon.points.size(), ++count) {
        Eigen::Vector3d p_x;
        p_x[0] = obj->polygon.points[i].x;
        p_x[1] = obj->polygon.points[i].y;
        p_x[2] = obj->polygon.points[i].z;
        size_t j = (i + 1) % obj->polygon.points.size();
        Eigen::Vector3d p_j;
        p_j[0] = obj->polygon.points[j].x;
        p_j[1] = obj->polygon.points[j].y;
        p_j[2] = obj->polygon.points[j].z;
        double dist = sqrt((p_x[0] - p_j[0]) * (p_x[0] - p_j[0]) +
                           (p_x[1] - p_j[1]) * (p_x[1] - p_j[1]));
        if (dist < max_dis && (dist / total_len) < 0.5) {
            continue;
        }

        if (j != min_point_index && j != max_point_index) {
            Eigen::Vector3d p;
            p[0] = obj->polygon.points[j].x;
            p[1] = obj->polygon.points[j].y;
            p[2] = obj->polygon.points[j].z;
            Eigen::Vector3d ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
                Eigen::Vector3d center;
                double length = 0;
                double width = 0;
                Eigen::Vector3d dir;
                double area = computeAreaAlongOneEdge(obj, i, &center, &length,
                                                      &width, &dir);
                if (area < min_area) {
                    obj->ground_center = center;
                    obj->length = length;
                    obj->width = width;
                    obj->direction = dir;
                    min_area = area;
                }
            } else {
                // outline
            }
        } else if ((i == min_point_index && j == max_point_index) ||
                   (i == max_point_index && j == min_point_index)) {
            if (!has_out) {
                continue;
            }
            Eigen::Vector3d center;
            double length = 0;
            double width = 0;
            Eigen::Vector3d dir;
            double area =
                computeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
            if (area < min_area) {
                obj->ground_center = center;
                obj->length = length;
                obj->width = width;
                obj->direction = dir;
                min_area = area;
            }
        } else if (j == min_point_index || j == max_point_index) {
            Eigen::Vector3d p;
            p[0] = obj->polygon.points[i].x;
            p[1] = obj->polygon.points[i].y;
            p[2] = obj->polygon.points[i].z;
            Eigen::Vector3d ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
                Eigen::Vector3d center;
                double length = 0.0;
                double width = 0.0;
                Eigen::Vector3d dir;
                double area = computeAreaAlongOneEdge(obj, i, &center, &length,
                                                      &width, &dir);
                if (area < min_area) {
                    obj->ground_center = center;
                    obj->length = length;
                    obj->width = width;
                    obj->direction = dir;
                    min_area = area;
                }
            } else {
                // outlier
            }
        }
    }

    // <4> 边框方向
    obj->direction.normalize();
}

/**
 * @brief compute area of obj->Polygon along <Edge: first_in_point->index>
 *  width: highest triangle
 *  length: longest distance between points projected into <Edge:
 * first_in_point->index>
 *  center: length,width extended rectangle
 *  dir: length's edge 2d direction vector
 * @param obj
 * @param first_in_point, <Edge: first_in_point->first_in_point's next>
 * @param center
 * @param length
 * @param width
 * @param dir
 * @return
 */
double MinBoxObjectBuilder::computeAreaAlongOneEdge(ObjectPtr obj,
                                                    size_t first_in_point,
                                                    Eigen::Vector3d *center,
                                                    double *length,
                                                    double *width,
                                                    Eigen::Vector3d *dir) {
    std::vector<Eigen::Vector3d> ns;
    Eigen::Vector3d v(0.0, 0.0, 0.0);
    Eigen::Vector3d vn(0.0, 0.0, 0.0);
    Eigen::Vector3d n(0.0, 0.0, 0.0);
    double len = 0;
    double wid = 0;
    // Edge: first_in_point->index
    size_t index = (first_in_point + 1) % obj->polygon.points.size();
    for (size_t i = 0; i < obj->polygon.points.size(); ++i) {
        if (i != first_in_point && i != index) {
            // compute v
            Eigen::Vector3d o(0.0, 0.0, 0.0);
            Eigen::Vector3d a(0.0, 0.0, 0.0);
            Eigen::Vector3d b(0.0, 0.0, 0.0);
            o[0] = obj->polygon.points[i].x;
            o[1] = obj->polygon.points[i].y;
            o[2] = 0;
            b[0] = obj->polygon.points[first_in_point].x;
            b[1] = obj->polygon.points[first_in_point].y;
            b[2] = 0;
            a[0] = obj->polygon.points[index].x;
            a[1] = obj->polygon.points[index].y;
            a[2] = 0;
            double k =
                ((a[0] - o[0]) * (b[0] - a[0]) + (a[1] - o[1]) * (b[1] - a[1]));
            k = k /
                ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
            k = k * -1;
            // n is pedal of src
            // TODO(gary): Projected to <Edge: first_in_point->index>???
            n[0] = (b[0] - a[0]) * k + a[0];
            n[1] = (b[1] - a[1]) * k + a[1];
            n[2] = 0;
            // compute height from src to line
            Eigen::Vector3d edge1 = o - b;
            Eigen::Vector3d edge2 = a - b;
            // cross product
            double height = fabs(edge1[0] * edge2[1] - edge2[0] * edge1[1]);
            height = height / sqrt(edge2[0] * edge2[0] + edge2[1] * edge2[1]);
            // highest triangle contributes the width of the Polygon
            if (height > wid) {
                wid = height;
                v = o;
                vn = n;
            }
        } else {
            n[0] = obj->polygon.points[i].x;
            n[1] = obj->polygon.points[i].y;
            n[2] = 0;
        }
        ns.push_back(n);
    }

    // longest distance between points projected into
    //   <Edge:first_in_point->index>
    // contributes the length of the Polygon
    size_t point_num1 = 0;
    size_t point_num2 = 0;
    for (size_t i = 0; i < ns.size() - 1; ++i) {
        Eigen::Vector3d p1 = ns[i];
        for (size_t j = i + 1; j < ns.size(); ++j) {
            Eigen::Vector3d p2 = ns[j];
            double dist = sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                               (p1[1] - p2[1]) * (p1[1] - p2[1]));
            if (dist > len) {
                len = dist;
                point_num1 = i;
                point_num2 = j;
            }
        }
    }
    /*
     * ground center
     *  v: vertex of the highest triangle
     *  vn: projected point of v into <Edge: first_in_point->index>
     *  ns[point_num1]/ns[point_num2]: longest distance's points projected into
     * <Edge: first_in_point->index>
     */
    Eigen::Vector3d vp1 = ns[point_num1] + (v - vn);
    Eigen::Vector3d vp2 = ns[point_num2] + (v - vn);
    (*center) = (vp1 + vp2 + ns[point_num1] + ns[point_num2]) / 4;
    (*center)[2] = obj->polygon.points[0].z;
    /*
     * only a 2D direction
     *  <Edge: first_in_point->index> is the direction: 2 projected points'
     * vector
     *  <Edge: first_in_point->index> vertical to the direction: wid's edge
     * vector
     */
    if (len > wid) {
        *dir = ns[point_num2] - ns[point_num1];
    } else {
        *dir = vp1 - ns[point_num1];
    }
    // length & width are interchangeable
    *length = len > wid ? len : wid;
    *width = len > wid ? wid : len;
    return (*length) * (*width);
}

}  // namespace object_builder
}  // namespace autosense
