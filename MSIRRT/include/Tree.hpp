#pragma once

#include <nanoflann.hpp>
#include "Vertex.hpp"

#define TREE_DIMENSIONALITY 6

namespace MDP::MSIRRT
{
    struct Tree;
    typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, MDP::MSIRRT::Tree>, MDP::MSIRRT::Tree> KdTree; 

    struct Tree
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Tree(const int& dimensionality);
        Tree(const std::string &tree_name_, size_t tree_idx_, const int& dimensionality);
        ~Tree();                                     // destructor
        Tree(const Tree &other) = delete;            // copy constructor
        Tree(Tree &&other) = default;                // move constructor
        Tree &operator=(const Tree &other) = delete; // copy assignment
        Tree &operator=(Tree &&other) = default;     // move assignment

        std::string tree_name;
        size_t tree_idx;

        std::vector<MDP::MSIRRT::Vertex*> array_of_vertices; //TODO: Maybe move to private?
        MDP::MSIRRT::KdTree kd_tree;

        MDP::MSIRRT::Vertex &getNearestState(const MDP::MSIRRT::Vertex q);

        void add_vertex(const MDP::MSIRRT::Vertex::VertexCoordType &q_new_coords, std::pair<int, int> q_new_safe_interval, MDP::MSIRRT::Vertex *q_parent, double departure_time, double arrival_time);
        void add_vertex(const std::vector<double> &q_new_coords, std::pair<int, int> q_new_safe_interval, MDP::MSIRRT::Vertex *q_parent, double departure_time, double arrival_time);

        // void delete_vertex(int vertex_id);

        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
        inline size_t kdtree_get_point_count() const { return array_of_vertices.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return array_of_vertices[idx]->coords[dim]; }
    };
}
