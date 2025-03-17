#include <nanoflann.hpp>
#include "Vertex.hpp"
#include "Tree.hpp"

MDP::MSIRRT::Tree::Tree(const std::string &tree_name_, size_t tree_idx_, const int& dimensionality) : tree_name(tree_name_), tree_idx(tree_idx_),
                                                                           kd_tree(dimensionality, *this, nanoflann::KDTreeSingleIndexAdaptorParams(25)) {};

MDP::MSIRRT::Tree::Tree(const int& dimensionality) : tree_name(""), tree_idx(-1), kd_tree(dimensionality, *this, nanoflann::KDTreeSingleIndexAdaptorParams(25)) {};

MDP::MSIRRT::Tree::~Tree() {

    for(int i=0;i< this->array_of_vertices.size();i++){
        delete array_of_vertices[i];
    }
    array_of_vertices.clear();
}; // destructor

void MDP::MSIRRT::Tree::add_vertex(const MDP::MSIRRT::Vertex::VertexCoordType &q_new_coords, std::pair<int, int> q_new_safe_interval , MDP::MSIRRT::Vertex *q_parent,  double departure_time, double arrival_time)
{   
    MDP::MSIRRT::Vertex* q_new = new MDP::MSIRRT::Vertex(q_new_coords, q_new_safe_interval);
    q_new->tree_id = this->tree_idx;
    q_new->parent = q_parent;
    q_new->arrival_time = arrival_time;
    q_new->departure_from_parent_time = departure_time;
    this->array_of_vertices.push_back(q_new);

    size_t N{array_of_vertices.size() - 1};

    // this->array_of_vertices.back().ID_in_array = N;
    this->kd_tree.addPoints(N, N);

    if (q_parent != nullptr)
    {
        q_parent->children.push_back(this->array_of_vertices.back());
    }
}

void MDP::MSIRRT::Tree::add_vertex(const std::vector<double> &q_new_coords, std::pair<int, int> q_new_safe_interval, MDP::MSIRRT::Vertex *q_parent, double departure_time, double arrival_time){
    this->add_vertex(MDP::MSIRRT::Vertex::VertexCoordType(q_new_coords.data()), q_new_safe_interval, q_parent, departure_time, arrival_time);
}

// void MDP::MSIRRT::Tree::delete_vertex(int vertex_id)
// {
//     this->kd_tree.removePoint(vertex_id);
// }

