#pragma once
#ifndef MPL_DEMO_SE3_RIGID_BODY_PLANNING_HPP
#define MPL_DEMO_SE3_RIGID_BODY_PLANNING_HPP

#include "../interpolate.hpp"
#include "../randomize.hpp"
// #include <bump/bvh_mesh.hpp>
#include <jilog.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <nigh/se3_space.hpp>
#include <array>

namespace mpl::demo {
    template <typename Scalar>
    auto mapToEigen(const aiMatrix4x4t<Scalar>& m) {
        using EigenType = const Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>;
        static_assert(sizeof(EigenType) == sizeof(m));
        return Eigen::Map<EigenType>(&m.a1);
    }

    template <typename Scalar>
    auto mapToEigen(const aiVector3t<Scalar>& v) {
        using EigenType = const Eigen::Matrix<Scalar, 3, 1>;
        static_assert(sizeof(EigenType) == sizeof(v));
        return Eigen::Map<const EigenType>(&v.x);
    }

    template <typename Scalar, typename Fn, int mode>
    std::size_t visitVertices(
        const aiScene* scene, const aiNode *node,
        Eigen::Transform<Scalar, 3, mode> transform,
        Fn&& visitor)
    {
        std::size_t count = 0;
        transform *= mapToEigen(node->mTransformation).template cast<Scalar>();
        for (unsigned i=0 ; i < node->mNumMeshes ; ++i) {
            const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            count += mesh->mNumVertices;
            for (unsigned j=0 ; j < mesh->mNumVertices ; ++j)
                visitor(transform * mapToEigen(mesh->mVertices[j]).template cast<Scalar>());
        }
        for (unsigned i=0 ; i < node->mNumChildren ; ++i)
            count += visitVertices(scene, node->mChildren[i], transform, std::forward<Fn>(visitor));
        return count;
    }


    template <typename Scalar, typename Fn, int mode>
    static std::size_t visitTriangles(
        const aiScene *scene, const aiNode *node,
        Eigen::Transform<Scalar, 3, mode> transform,
        Fn&& visitor)
    {
        std::size_t count = 0;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    
        transform *= mapToEigen(node->mTransformation).template cast<Scalar>();
        for (unsigned i=0 ; i<node->mNumMeshes ; ++i) {
            const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            for (unsigned j=0 ; j<mesh->mNumFaces ; ++j) {
                const aiFace& face = mesh->mFaces[j];
                if (face.mNumIndices < 3)
                    continue;
            
                // Support trangular decomposition by fanning out
                // around vertex 0.  The indexing follows as:
                //
                //   0---1   0 1 2
                //  /|\ /    0 2 3
                // 4-3-2     0 3 4
                //
                Vec3 v0 = transform * mapToEigen(mesh->mVertices[face.mIndices[0]]).template cast<Scalar>();
                Vec3 v1 = transform * mapToEigen(mesh->mVertices[face.mIndices[1]]).template cast<Scalar>();
                for (unsigned k=2 ; k<face.mNumIndices ; ++k) {
                    Vec3 v2 = transform * mapToEigen(mesh->mVertices[face.mIndices[k]]).template cast<Scalar>();
                    visitor(v0, v1, v2);
                    v1 = v2;
                }
                count += face.mNumIndices - 2;
            }
        }
        for (unsigned i=0 ; i<node->mNumChildren ; ++i)
            count += visitTriangles(scene, node->mChildren[i], transform, std::forward<Fn>(visitor));
    
        return count;
    }

    template <class Mesh>
    std::shared_ptr<Mesh> loadMesh(const std::string& name, bool shiftToCenter) {
        using S = typename Mesh::S;
        using Transform = Eigen::Transform<S, 3, Eigen::Isometry>;
        using Vec3 = Eigen::Matrix<S, 3, 1>;

        JI_LOG(INFO) << "Loading mesh \"" << name << "\"";
    
        std::shared_ptr<Mesh> model = std::make_shared<Mesh>();

        Assimp::Importer importer;
    
        static constexpr auto readOpts =
            aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
            aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes;

        const aiScene *scene = importer.ReadFile(name, readOpts);
        if (scene == nullptr)
            throw std::invalid_argument("could not load mesh file '" + name + "'");

        if (!scene->HasMeshes())
            throw std::invalid_argument("mesh file '" + name + "' does not contain meshes");
    
        // TODO: scene::inferBounds(bounds, vertices, factor_, add_);
    
        Transform rootTransform = Transform::Identity();

        if (shiftToCenter) {
            Vec3 center = Vec3::Zero();
            std::size_t nVertices = visitVertices(
                scene,
                scene->mRootNode,
                rootTransform,
                [&] (const Vec3& v) { center += v; });
            center /= nVertices;
            rootTransform *= Eigen::Translation<S, 3>(-center);
            JI_LOG(INFO) << "shifted mesh to center: " << center;
        }

        model->beginModel();
        std::size_t nTris = visitTriangles(
            scene,
            scene->mRootNode,
            rootTransform,
            [&] (const Vec3& a, const Vec3& b, const Vec3& c) {
                model->addTriangle(a, b, c);
            });
        model->endModel();
        model->computeLocalAABB();

        return model;
    };

    void extractTriangles(const aiScene *scene, const aiNode *node, aiMatrix4x4 transform,
                          std::vector<aiVector3D> &triangles)
    {
        transform *= node->mTransformation;
        for (unsigned int i = 0 ; i < node->mNumMeshes; ++i)
        {
            const aiMesh* a = scene->mMeshes[node->mMeshes[i]];
            for (unsigned int i = 0 ; i < a->mNumFaces ; ++i)
                if (a->mFaces[i].mNumIndices == 3)
                {
                    triangles.push_back(transform * a->mVertices[a->mFaces[i].mIndices[0]]);
                    triangles.push_back(transform * a->mVertices[a->mFaces[i].mIndices[1]]);
                    triangles.push_back(transform * a->mVertices[a->mFaces[i].mIndices[2]]);
                }
        }
        
        for (unsigned int n = 0; n < node->mNumChildren; ++n)
            extractTriangles(scene, node->mChildren[n], transform, triangles);
    }
    
    void extractVertices(const aiScene *scene, const aiNode *node, aiMatrix4x4 transform,
                            std::vector<aiVector3D> &vertices)
    {
        transform *= node->mTransformation;
        for (unsigned int i = 0 ; i < node->mNumMeshes; ++i) {
            const aiMesh* a = scene->mMeshes[node->mMeshes[i]];
            for (unsigned int i = 0 ; i < a->mNumVertices ; ++i)
                vertices.push_back(transform * a->mVertices[i]);
        }
        
        for (unsigned int n = 0; n < node->mNumChildren; ++n)
            extractVertices(scene, node->mChildren[n], transform, vertices);
    }

    template <class Mesh>
    std::shared_ptr<Mesh> loadMesh2(const std::string& name, bool shiftToCenter) {
        using S = typename Mesh::S;
        
        std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();

        Assimp::Importer importer;
        
        const aiScene *aiScene = importer.ReadFile(
            name,
            aiProcess_GenNormals | aiProcess_Triangulate |
            aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_OptimizeGraph);

        if (aiScene == nullptr)
            throw std::invalid_argument("unable to load: " + name);
        
        if (!aiScene->HasMeshes())
            throw std::invalid_argument("No mesh found in " + name);

        std::vector<aiVector3D> vertices;
        extractVertices(aiScene, aiScene->mRootNode, aiMatrix4x4(), vertices);
        aiVector3D center;
        center.Set(0, 0, 0);
        for (auto& v : vertices)
            center += v;
        center /= vertices.size();

        std::vector<fcl::Triangle> triangles;
        std::vector<fcl::Vector3<S>> pts;
        vertices.clear();
        extractTriangles(aiScene, aiScene->mRootNode, aiMatrix4x4(), vertices);
        assert(vertices.size() % 3 == 0);

        if (shiftToCenter) {
            for (auto& j : vertices)
                j -= center;
        }

        for (auto& j : vertices)
            pts.emplace_back(j[0], j[1], j[2]);
        
        for (unsigned j = 0 ; j<vertices.size() ; j += 3)
            triangles.emplace_back(j, j+1, j+2);

        mesh->beginModel();
        mesh->addSubModel(pts, triangles);
        mesh->endModel();
        mesh->computeLocalAABB();
        return mesh;
    }

    template <class Mesh>
    struct MeshLoad;

    template <class S>
    struct MeshLoad<fcl::BVHModel<fcl::OBBRSS<S>>> {
        using Mesh = fcl::BVHModel<fcl::OBBRSS<S>>;
        static Mesh load(const std::string& name, bool shiftToCenter) {
            using Transform = Eigen::Transform<S, 3, Eigen::Isometry>;
            using Vec3 = Eigen::Matrix<S, 3, 1>;

            JI_LOG(INFO) << "Loading mesh \"" << name << "\"";
            
            Mesh model;

            Assimp::Importer importer;
            
            static constexpr auto readOpts =
                aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes;
            
            const aiScene *scene = importer.ReadFile(name, readOpts);
            if (scene == nullptr)
                throw std::invalid_argument("could not load mesh file '" + name + "'");
            
            if (!scene->HasMeshes())
                throw std::invalid_argument("mesh file '" + name + "' does not contain meshes");
            
            // TODO: scene::inferBounds(bounds, vertices, factor_, add_);
            
            Transform rootTransform = Transform::Identity();
            
            if (shiftToCenter) {
                Vec3 center = Vec3::Zero();
                std::size_t nVertices = visitVertices(
                    scene,
                    scene->mRootNode,
                    Transform::Identity(),
                    [&] (const Vec3& v) { center += v; });
                center /= nVertices;
                rootTransform *= Eigen::Translation<S, 3>(-center);
                JI_LOG(INFO) << "shifted mesh to center: " << center;
            }
            
            model.beginModel();
            std::size_t nTris = visitTriangles(
                scene,
                scene->mRootNode,
                rootTransform,
                [&] (const Vec3& a, const Vec3& b, const Vec3& c) {
                    model.addTriangle(a, b, c);
                });
            model.endModel();
            model.computeLocalAABB();

            return model;
        }
    };
#if 0
    template <class I, class S>
    struct MeshLoad<bump::BVHMesh<bump::Triangle<I>, bump::OBB<S>>> {
        using Mesh = bump::BVHMesh<bump::Triangle<I>, bump::OBB<S>>;
        static Mesh load(const std::string& name, bool shiftToCenter) {
            using Transform = Eigen::Transform<S, 3, Eigen::AffineCompact>;
            using Vec3 = Eigen::Matrix<S, 3, 1>;
            Assimp::Importer importer;

            static constexpr auto readOpts =
                aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes;

            const aiScene *scene = importer.ReadFile(name, readOpts);

            if (scene == nullptr)
                throw std::invalid_argument("could not load mesh file '" + name + "'");
            
            if (!scene->HasMeshes())
                throw std::invalid_argument("mesh file '" + name + "' does not contain meshes");
            
            Transform rootTransform = Transform::Identity();
            
            if (shiftToCenter) {
                Vec3 center = Vec3::Zero();
                std::size_t nVertices = visitVertices(
                    scene,
                    scene->mRootNode,
                    Transform::Identity(),
                    [&] (const Vec3& v) { center += v; });
                center /= nVertices;
                rootTransform *= Eigen::Translation<S, 3>(-center);
                JI_LOG(INFO) << "shifted mesh to center: " << center;
            }
            
            std::vector<Vec3> vertices;
            std::vector<bump::Triangle<I>> triangles;
            
            std::size_t nTris = visitTriangles(
                scene,
                scene->mRootNode,
                rootTransform,
                [&] (const Vec3& a, const Vec3& b, const Vec3& c) {
                    I i = vertices.size();
                    vertices.push_back(a);
                    vertices.push_back(b);
                    vertices.push_back(c);
                    
                    triangles.emplace_back(i, i+1, i+2);
                });
            
            return bump::BVHMesh<bump::Triangle<int>>(vertices, triangles);
        }
    };
#endif
    
    template <class S, std::intmax_t so3weight = 1, std::intmax_t l2weight = 1>
    class SE3RigidBodyScenario {
    public:
        using Space = unc::robotics::nigh::SE3Space<S, so3weight, l2weight>;
        using State = typename Space::Type;
        using Bound = Eigen::Matrix<S, 3, 1>;
        using Distance = S;

    private:
        using Mesh = fcl::BVHModel<fcl::OBBRSS<S>>;
        // using Mesh = bump::BVHMesh<bump::Triangle<int>, bump::OBB<S>>;
        // using Transform = Eigen::Transform<S, 3, Eigen::Isometry>;
        using Transform = fcl::Transform3<S>;

        Space space_;
    
        Mesh environment_;
        Mesh robot_;

        State goal_;

        Bound min_;
        Bound max_;

        Distance goalRadius_{0.1};
        Distance invStepSize_;

        mutable std::atomic<unsigned> calls_{0};

        static Transform stateToTransform(const State& q) {
            return Eigen::Translation<S, 3>(std::get<Eigen::Matrix<S, 3, 1>>(q))
                * std::get<Eigen::Quaternion<S>>(q);
        }

    public:
        template <class Min, class Max>
        SE3RigidBodyScenario(
            const std::string& envMesh,
            const std::string& robotMesh,
            const State& goal,
            const Eigen::MatrixBase<Min>& min,
            const Eigen::MatrixBase<Max>& max,
            S checkResolution)
            : environment_(MeshLoad<Mesh>::load(envMesh, false))
            , robot_(MeshLoad<Mesh>::load(robotMesh, true))
            , goal_(goal)
            , min_(min)
            , max_(max)
            , invStepSize_(1/checkResolution)
        {
            // JI_LOG(INFO) << "loaded env: " << environment_->num_vertices << " vertices, " << environment_->num_tris << " tris";
            // JI_LOG(INFO) << "loaded robot: " << robot_->num_vertices << " vertices, " << robot_->num_tris << " tris";

            // auto robotTest = MeshLoad<Mesh>::load(robotMesh, true);
            
            // fcl::CollisionRequest<S> req;
            // fcl::CollisionResult<S> res;
            // Transform id = Transform::Identity();
            // Transform a = Transform::Identity();
            // a *= Eigen::Translation<S,3>(0,0,0.001);
            // JI_LOG(INFO) << a.matrix();
            // JI_LOG(INFO) << "self collision check: " << fcl::collide(robot_, a, robotTest.get(), id, req, res);
            // a *= Eigen::Translation<S,3>(100,1000,100.0);
            // JI_LOG(INFO) << a.matrix();
            // JI_LOG(INFO) << "self collision check: " << fcl::collide(robot_.get(), a, robotTest.get(), id, req, res);
        }

        ~SE3RigidBodyScenario() {
            JI_LOG(INFO) << "isValid calls: " << calls_.load();
        }

        const Space& space() const {
            return space_;
        }

        template <class RNG>
        const State& sampleGoal(RNG& rng) {
            return goal_;
        }

        template <class RNG>
        State randomSample(RNG& rng) {
            State q;
            randomize(std::get<Eigen::Quaternion<S>>(q), rng);
            randomize(std::get<Eigen::Matrix<S,3,1>>(q), rng, min_, max_);
            return q;
        }

        bool isGoal(const State& q) const {
            return space_.distance(goal_, q) <= goalRadius_;            
        }

        bool isValid(const State& q) const {
            ++calls_;
            fcl::CollisionRequest<S> req;
            fcl::CollisionResult<S> res;

            Transform tf = stateToTransform(q);
            
            return !fcl::collide(&robot_, tf, &environment_, Transform::Identity(), req, res);
            
            // static Transform id{Transform::Identity()};

            // Transform tf = stateToTransform(q);
            // return !fcl::collide(robot_.get(), tf, environment_.get(), id, req, res);
        }    

        bool isValid(const State& from, const State& to) const {
            assert(isValid(from));
            if (!isValid(to))
                return false;

            std::size_t steps = std::ceil(space_.distance(from, to) * invStepSize_);

            // JI_LOG(DEBUG) << "steps = " << steps;
            
            if (steps < 2)
                return true;

            Distance delta = 1 / Distance(steps);
            // for (std::size_t i = 1 ; i < steps ; ++i)
            //     if (!isValid(interpolate(from, to, i*delta)))
            //         return false;
            
            // if (true) return true;
            
            std::array<std::pair<std::size_t, std::size_t>, 1024> queue;
            queue[0] = std::make_pair(std::size_t(1), steps-1);
            std::size_t qStart = 0, qEnd = 1;
            while (qStart != qEnd) {
                auto [min, max] = queue[qStart++ % queue.size()];
                if (min == max) {
                    if (!isValid(interpolate(from, to, min * delta)))
                        return false;
                } else if (qEnd + 2 < qStart + queue.size()) {
                    std::size_t mid = (min + max) / 2;
                    if (!isValid(interpolate(from, to, mid * delta)))
                        return false;
                    if (min < mid)
                        queue[qEnd++ % queue.size()] = std::make_pair(min, mid-1);
                    if (mid < max)
                        queue[qEnd++ % queue.size()] = std::make_pair(mid+1, max);
                } else {
                    // queue is full
                    for (std::size_t i=min ; i<=max ; ++i)
                        if (!isValid(interpolate(from, to, i*delta)))
                            return false;
                }
            }

            return true;
        }
    };
}

#endif
