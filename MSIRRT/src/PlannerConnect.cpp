
#include <vector>

#include "PlannerConnect.hpp"
#include "config_read_writer/config_read.hpp"

MDP::MSIRRT::PlannerConnect::PlannerConnect(MDP::ConfigReader::SceneTask scene_task_, int random_seed) : scene_task(scene_task_),
                                                                                                         collision_manager(scene_task_), probability_gen(0.0, 1.0),
                                                                                                         gen(random_seed), rd()
{

    this->start_tree = new MDP::MSIRRT::Tree("start_tree", 0);
    this->goal_tree = new MDP::MSIRRT::Tree("goal_tree", 1);
    this->orphan_tree = new MDP::MSIRRT::Tree("orphan_tree", -1);
    this->start_tree->array_of_vertices.reserve(1000000);  // TODO: decide to leave it or remove it
    this->goal_tree->array_of_vertices.reserve(1000000);   // TODO: decide to leave it or remove it
    this->orphan_tree->array_of_vertices.reserve(1000000); // TODO: decide to leave it or remove it

    this->robot_limits = this->collision_manager.get_planned_robot_limits();

    assert(is_coords_in_limits(MDP::MSIRRT::Vertex::VertexCoordType(scene_task.start_configuration.data()))); // check start conf bound;

    assert(is_coords_in_limits(MDP::MSIRRT::Vertex::VertexCoordType(scene_task.end_configuration.data()))); // check end conf bounds;
    this->dof = scene_task.start_configuration.size();
    std::vector<std::pair<int, int>> start_safe_intervals = this->collision_manager.get_safe_intervals(scene_task.start_configuration); // get start cond save intervals;

    // сheck if first start at 0
    assert(start_safe_intervals.size() > 0);
    assert(start_safe_intervals[0].first == 0);

    this->start_tree->add_vertex(MDP::MSIRRT::Vertex(scene_task.start_configuration, start_safe_intervals[0]), nullptr, -1, 0);
    for (int another_safe_interval_id = 1; another_safe_interval_id < start_safe_intervals.size(); another_safe_interval_id++)
    {
        this->orphan_tree->add_vertex(MDP::MSIRRT::Vertex(scene_task.start_configuration, start_safe_intervals[another_safe_interval_id]), nullptr, -1, -1);
    }

    this->root_node = &this->start_tree->array_of_vertices[0];
    this->root_node->arrival_time = 0;

    this->goal_coords = MDP::MSIRRT::Vertex::VertexCoordType(scene_task.end_configuration.data());
    this->goal_safe_intervals = this->collision_manager.get_safe_intervals(scene_task.end_configuration);

    this->goal_tree->add_vertex(MDP::MSIRRT::Vertex(this->goal_coords, this->goal_safe_intervals.back()), nullptr, -1, this->goal_safe_intervals.back().second);

    assert(this->goal_safe_intervals.back().second == scene_task.frame_count - 1);
    this->current_tree = this->goal_tree;
    this->other_tree = this->start_tree;
    this->max_planning_time = 20;
    this->stop_when_path_found = true;
    this->goal_reached = false;
    this->planner_range = 1;
    this->vmax = 3.1415;
}
// destructor
MDP::MSIRRT::PlannerConnect::~PlannerConnect()
{
    delete this->start_tree;
    delete this->goal_tree;
    delete this->orphan_tree;
}

bool MDP::MSIRRT::PlannerConnect::solve()
{
    this->solver_start_time = std::chrono::steady_clock::now();
    int iter = 0;
    while (this->check_planner_termination_condition() && !this->goal_reached)
    {
        // std::cout << "iteration #" << iter++ << std::endl;
        MDP::MSIRRT::Vertex::VertexCoordType coord_rand;
        coord_rand = get_random_configuration();

        bool is_ok = this->extend(coord_rand);
        if (!is_ok)
        {
            continue;
        }

        std::vector<double> robot_angles(coord_rand.data(), coord_rand.data() + coord_rand.rows() * coord_rand.cols());

        std::vector<std::pair<int, int>> safe_intervals_of_coord_rand;

        safe_intervals_of_coord_rand = this->collision_manager.get_safe_intervals(robot_angles);

        std::vector<MDP::MSIRRT::Vertex *> new_nodes = this->grow_tree(coord_rand, safe_intervals_of_coord_rand);

        if (new_nodes.size() == 0)
        {
            this->swap_trees();
            continue;
        }
        bool connected = connect_trees(coord_rand, safe_intervals_of_coord_rand, new_nodes);
        this->swap_trees();
    }
    return this->goal_reached;
}
void MDP::MSIRRT::PlannerConnect::swap_trees()
{
    std::swap(this->current_tree, this->other_tree);
}

bool MDP::MSIRRT::PlannerConnect::connect_trees(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand, std::vector<MDP::MSIRRT::Vertex *> another_tree_new_nodes)
{
    const MDP::MSIRRT::Vertex::VertexCoordType original_coord_rand = coord_rand;

    while (true)
    {
        coord_rand = original_coord_rand;
        this->swap_trees();
        bool is_ok = this->extend(coord_rand);
        this->swap_trees();
        if (!is_ok)
        {
            break;
        }

        std::vector<double> robot_angles(coord_rand.data(), coord_rand.data() + coord_rand.rows() * coord_rand.cols());

        std::vector<std::pair<int, int>> safe_intervals_of_coord_rand;

        safe_intervals_of_coord_rand = this->collision_manager.get_safe_intervals(robot_angles);
        this->swap_trees();
        std::vector<MDP::MSIRRT::Vertex *> new_nodes = this->grow_tree(coord_rand, safe_intervals_of_coord_rand);
        this->swap_trees();
        if (new_nodes.size() == 0)
        {
            break;
        }

        if (original_coord_rand == coord_rand) // if we reached original nodes of first tree
        {
            for (MDP::MSIRRT::Vertex *node : new_nodes) // for new node in second tree
            {
                for (MDP::MSIRRT::Vertex *another_tree_node : another_tree_new_nodes) // for new node in first tree
                {
                    // assert(node->coords == another_tree_node->coords);
                    if (node->coords == another_tree_node->coords)
                    {
                        if (node->safe_interval == another_tree_node->safe_interval)
                        {
                            if (node->tree_id == 1) // TODO: change tree_id to enums. 1==goal_Tree
                            {
                                if (node->arrival_time >= another_tree_node->arrival_time)
                                {
                                    this->goal_reached = true;
                                    this->goal_nodes = std::pair<MDP::MSIRRT::Vertex *, MDP::MSIRRT::Vertex *>(another_tree_node, node);
                                    // std::cout << "goal reached!" << std::endl;
                                    //
                                    this->prune_goal_tree();
                                }
                            }
                            else
                            {
                                if (node->arrival_time <= another_tree_node->arrival_time)
                                {
                                    this->goal_reached = true;
                                    this->goal_nodes = std::pair<MDP::MSIRRT::Vertex *, MDP::MSIRRT::Vertex *>(node, another_tree_node);
                                    // std::cout << "goal reached!" << std::endl;
                                    this->prune_goal_tree();
                                }
                            }
                            return true;
                        }
                    }
                }
                if (this->goal_reached)
                {
                    break;
                }
            }
            break;
        }
        coord_rand = original_coord_rand;
    }

    return false;
}

// std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::PlannerConnect::extend_naive(MDP::MSIRRT::Vertex::VertexCoordType &coords_of_new)
// {
//     MDP::MSIRRT::Vertex *q_nearest = this->get_nearest_node(coords_of_new, &(this->current_tree)); // find nearest neighbor

//     MDP::MSIRRT::Vertex::VertexCoordType delta_vector = coords_of_new - q_nearest->coords;
//     std::vector<MDP::MSIRRT::Vertex *> added_vertices;

//     double delta = delta_vector.norm();
//     // std::cout << "delta: " << delta << std::endl;
//     if (delta <= (0.01)) // zero division check
//     {
//         std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl; // we don't expect such behavior
//         return added_vertices;
//     }

//     if (delta >= this->planner_range)
//     {
//         coords_of_new = q_nearest->coords + delta_vector.normalized() * this->planner_range;
//     }

//     std::vector<double> robot_angles(coords_of_new.data(), coords_of_new.data() + coords_of_new.rows() * coords_of_new.cols());

//     std::vector<std::pair<int, int>> safe_intervals_of_coords_of_new = this->collision_manager.get_safe_intervals(robot_angles);

//     int safe_interval_ind = 0;
//     int time_to_goal = (coords_of_new - this->goal_coords).norm() * (double)this->scene_task.fps / this->vmax;
//     for (std::pair<int, int> safe_int : safe_intervals_of_coords_of_new) // For each interval
//     {
//         // if we can't reach goal from that interval - skip
//         if (safe_int.first + time_to_goal > scene_task.frame_count)
//         {
//             continue;
//         }
//         bool found_parent = false;
//         // std::cout << safe_int.first << " " << safe_int.second  << std::endl;

//         for (int arrival_time = safe_int.first; arrival_time < std::min(safe_int.second, (int)this->scene_task.frame_count - time_to_goal); arrival_time++)
//         {

//             int parent_low_bound_arrive = ((coords_of_new - q_nearest->coords).norm() * ((double)this->scene_task.fps) / (this->vmax));

//             // if don't intersect
//             if ((q_nearest->arrival_time + parent_low_bound_arrive > safe_int.second)) // || (q_nearest->safe_interval.second + parent_low_bound_arrive < safe_int.first))

//             {
//                 continue;
//             }
//             for (int departure_time = arrival_time - parent_low_bound_arrive; departure_time >= q_nearest->arrival_time; departure_time--)
//             {
//                 if ((departure_time < q_nearest->arrival_time) || (departure_time > q_nearest->safe_interval.second))
//                 {
//                     break;
//                 }
//                 // std::cout << "departure_time: " << q_nearest->arrival_time << " " << q_nearest->safe_interval.second << " " << departure_time << std::endl;
//                 MDP::MSIRRT::Vertex::VertexCoordType last_valid_coord;
//                 int last_valid_time;
//                 if (!is_collision_motion(q_nearest->coords, coords_of_new, departure_time, arrival_time, last_valid_coord, last_valid_time))
//                 {
//                     // assert(!is_collision_motion(q_nearest->coords, q_nearest->coords, q_nearest->arrival_time, departure_time));
//                     this->current_tree.add_vertex(MDP::MSIRRT::Vertex(coords_of_new, safe_int), q_nearest, departure_time, arrival_time);
//                     added_vertices.push_back(&(this->current_tree.array_of_vertices.back()));
//                     found_parent = true;
//                     break;
//                 }
//                 else
//                 {
//                     MDP::MSIRRT::Vertex *q_nearest_to_last_valid = this->get_nearest_node(last_valid_coord, &(this->current_tree)); // find nearest neighbor

//                     MDP::MSIRRT::Vertex::VertexCoordType delta_vector = last_valid_coord - q_nearest_to_last_valid->coords;

//                     delta = delta_vector.norm();
//                     // std::cout << "delta: " << delta << std::endl;
//                     if (delta <= (0.1))
//                     {
//                         // std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl; // we don't expect such behavior
//                         continue;
//                     }
//                     std::vector<double> robot_angles(last_valid_coord.data(), last_valid_coord.data() + last_valid_coord.rows() * last_valid_coord.cols());

//                     std::vector<std::pair<int, int>> safe_intervals_of_last_valid_coord = this->collision_manager.get_safe_intervals(robot_angles);
//                     for (const std::pair<int, int> safe_int : safe_intervals_of_last_valid_coord)
//                     {
//                         if (safe_int.first <= last_valid_time && safe_int.second >= last_valid_time)
//                         {
//                             this->current_tree->add_vertex(MDP::MSIRRT::Vertex(last_valid_coord, safe_int), q_nearest, departure_time, last_valid_time);
//                             added_vertices.push_back(&(this->current_tree->array_of_vertices.back()));
//                             found_parent = true;
//                             break;
//                         }
//                         else if (safe_int.first > last_valid_time){
//                             break;
//                         }
//                     }
//                     if (found_parent)
//                     {
//                         break;
//                     }
//                 }
//             }
//             if (found_parent)
//             {
//                 break;
//             }
//         }

//         // if (!found_parent)
//         // {
//         //     this->orphan_tree.add_vertex(MDP::MSIRRT::Vertex(coords_of_new, safe_intervals_of_coords_of_new[safe_interval_ind]), nullptr, -1, -1);
//         // }
//         safe_interval_ind++;
//     }
//     return added_vertices;
// }

// bool MDP::MSIRRT::PlannerConnect::is_collision_motion(const MDP::MSIRRT::Vertex::VertexCoordType &start_coords, const MDP::MSIRRT::Vertex::VertexCoordType &end_coords, int &start_time, double &end_time, MDP::MSIRRT::Vertex::VertexCoordType &last_valid_coord, double &last_valid_time)
// {

//     if (start_time > end_time)
//     {
//         // std::cout << "start_time > end_time" << std::endl;
//         return true;
//     }
//     MDP::MSIRRT::Vertex::VertexCoordType dir_vector = end_coords - start_coords;
//     if ((dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time))) >= this->vmax)
//     {
//         // std::cout << "((dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time))) >= this->vmax)" << std::endl;
//         return true;
//     }

//     int interpolation_steps = std::max({(int)(dir_vector.norm() / 0.1), (int)(end_time - start_time + 0.5), 1});
//     for (int step = 0; step <= interpolation_steps; step++)
//     {

//         MDP::MSIRRT::Vertex::VertexCoordType temp_coords = start_coords + dir_vector * (double)step / (double)interpolation_steps;
//         int time_frame = start_time + (double)(end_time - start_time) * (double)step / (double)interpolation_steps;

//         if (this->is_collision_state(temp_coords, time_frame))
//         {
//             // std::cout << "is_collision!!" << std::endl;

//             return true;
//         }
//     }
//     return false;
// }

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::PlannerConnect::grow_tree(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand)
{
    std::vector<MDP::MSIRRT::Vertex *> result;
    std::vector<MDP::MSIRRT::Vertex *> array_of_new_nodes = this->set_parent(coord_rand, safe_intervals_of_coord_rand);

    if (array_of_new_nodes.size() == 0)
    {
        // std::cout << "no_new_node" << std::endl;
        return result;
    }
    result.reserve(result.size() + std::distance(array_of_new_nodes.begin(), array_of_new_nodes.end()));
    result.insert(result.end(), array_of_new_nodes.begin(), array_of_new_nodes.end());

    // std::cout << "find_new_node!" << std::endl;
    // std::cout << "tree size:" << this->start_tree->array_of_vertices.size() << " " << this->goal_tree->array_of_vertices.size() << "  " << this->orphan_tree->array_of_vertices.size() << std::endl;

    while (array_of_new_nodes.size() != 0)
    {
        MDP::MSIRRT::Vertex *node = array_of_new_nodes.back();
        // std::vector<MDP::MSIRRT::Vertex *> rewired_nodes = this->rewire(node); // rewire

        array_of_new_nodes.pop_back();
        // if (rewired_nodes.size() != 0)
        // {
        //     result.reserve(result.size() + std::distance(rewired_nodes.begin(), rewired_nodes.end()));
        //     result.insert(result.end(), rewired_nodes.begin(), rewired_nodes.end());

        //     array_of_new_nodes.reserve(array_of_new_nodes.size() + std::distance(rewired_nodes.begin(), rewired_nodes.end()));
        //     array_of_new_nodes.insert(array_of_new_nodes.end(), rewired_nodes.begin(), rewired_nodes.end());
        // }
    }
    return result;
}

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::PlannerConnect::rewire(MDP::MSIRRT::Vertex *node)
{

    std::vector<MDP::MSIRRT::Vertex *> rewired_nodes;

    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> nearest_nodes = this->get_nearest_node_by_radius(node->coords, 1.2 * this->planner_range * this->planner_range, (this->current_tree));
    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> nearest_nodes_orphan_tree = this->get_nearest_node_by_radius(node->coords, 1.2 * this->planner_range * this->planner_range, (this->orphan_tree));
    nearest_nodes.reserve(nearest_nodes.size() + std::distance(nearest_nodes_orphan_tree.begin(), nearest_nodes_orphan_tree.end()));
    nearest_nodes.insert(nearest_nodes.end(), nearest_nodes_orphan_tree.begin(), nearest_nodes_orphan_tree.end());

    if (nearest_nodes.size() == 0)
    {
        // std::cout << "BBBB" << std::endl;
    }
    int max_frame = node->before_rewiring_arrival_time;
    if (max_frame == -1)
    {
        max_frame = node->safe_interval.second;
    }

    node->before_rewiring_arrival_time = -1;

    // for each candidate
    if (this->current_tree == this->start_tree)
    {
        for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes)
        {
            bool is_rewired = false;
            // assert(candidate_node.first);
            if (candidate_node.first == node)
            {
                continue;
            }
            if (candidate_node.first == node->parent)
            {
                continue;
            }
            if ((candidate_node.first->safe_interval.first == candidate_node.first->arrival_time) && (candidate_node.first->parent)) // Can't do better
            {
                continue;
            }
            if (candidate_node.first->arrival_time == -1)
            {
                candidate_node.first->arrival_time = candidate_node.first->safe_interval.second + 1;
            }

            // don't overlap
            if ((!((candidate_node.first->safe_interval.first >= node->arrival_time && candidate_node.first->safe_interval.first <= max_frame) || (candidate_node.first->arrival_time >= node->arrival_time && candidate_node.first->arrival_time <= max_frame) ||
                   (node->arrival_time >= candidate_node.first->safe_interval.first && node->arrival_time <= candidate_node.first->arrival_time) || (max_frame >= candidate_node.first->safe_interval.first && max_frame <= candidate_node.first->arrival_time))))
            {
                continue;
            }

            MDP::MSIRRT::Vertex::VertexCoordType start_coords = node->coords;

            // for each arrival_time time
            for (double arrival_time = candidate_node.first->safe_interval.first; arrival_time < candidate_node.first->arrival_time; arrival_time++)
            {
                int max_time = std::min(max_frame, (int)arrival_time);
                for (double departure_time = node->arrival_time; departure_time < max_time; departure_time++)
                {
                    // if (departure_time > candidate_node.first->arrival_time)
                    // {
                    //     break;
                    // }
                    if (!is_collision_motion(start_coords, candidate_node.first->coords, departure_time, arrival_time))
                    {
                        // assert(!is_collision_motion(start_coords, start_coords, candidate_node.first->arrival_time, departure_time));

                        if (!candidate_node.first->parent) // if was orphan
                        {
                            this->current_tree->add_vertex(*(candidate_node.first), node, departure_time, arrival_time);

                            this->orphan_tree->delete_vertex(candidate_node.second);

                            rewired_nodes.push_back(&(this->current_tree->array_of_vertices.back()));
                            is_rewired = true;
                        }
                        else
                        {
                            candidate_node.first->parent = node;
                            candidate_node.first->before_rewiring_arrival_time = candidate_node.first->arrival_time;
                            candidate_node.first->arrival_time = arrival_time;
                        }

                        rewired_nodes.push_back(candidate_node.first);
                        is_rewired = true;
                        break;
                    }
                }
                if (is_rewired)
                {
                    break;
                }
                // assert(!is_collision_motion(start_coords, start_coords, node->arrival_time, departure_time));
            }
            if (is_rewired)
            {
                continue;
            }
        }
    }
    else
    {
        for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes)
        {
            bool is_rewired = false;
            // assert(candidate_node.first);
            if (candidate_node.first == node)
            {
                continue;
            }
            if (candidate_node.first == node->parent)
            {
                continue;
            }
            if ((candidate_node.first->safe_interval.first == candidate_node.first->arrival_time) && (candidate_node.first->parent)) // Can't do better
            {
                continue;
            }
            if (candidate_node.first->arrival_time == -1)
            {
                candidate_node.first->arrival_time = candidate_node.first->safe_interval.second + 1;
            }
            // don't overlap
            if ((!((candidate_node.first->safe_interval.first >= node->arrival_time && candidate_node.first->safe_interval.first <= max_frame) || (candidate_node.first->arrival_time >= node->arrival_time && candidate_node.first->arrival_time <= max_frame) ||
                   (node->arrival_time >= candidate_node.first->safe_interval.first && node->arrival_time <= candidate_node.first->arrival_time) || (max_frame >= candidate_node.first->safe_interval.first && max_frame <= candidate_node.first->arrival_time))))
            {
                continue;
            }

            MDP::MSIRRT::Vertex::VertexCoordType start_coords = node->coords;

            // for each arrival_time time
            for (double arrival_time = candidate_node.first->safe_interval.second; arrival_time > candidate_node.first->arrival_time; arrival_time--)
            {
                int min_time = std::max(max_frame, (int)arrival_time);
                for (double departure_time = node->safe_interval.second; departure_time > min_time; departure_time--)
                {
                    // if (departure_time < candidate_node.first->arrival_time)
                    // {
                    //     break;
                    // }
                    if (!is_collision_motion(candidate_node.first->coords, start_coords, arrival_time, departure_time))
                    {

                        if (!candidate_node.first->parent) // if was orphan
                        {
                            this->current_tree->add_vertex(*(candidate_node.first), node, departure_time, arrival_time);

                            this->orphan_tree->delete_vertex(candidate_node.second);

                            rewired_nodes.push_back(&(this->current_tree->array_of_vertices.back()));
                            is_rewired = true;
                        }
                        else
                        {
                            candidate_node.first->parent = node;
                            candidate_node.first->before_rewiring_arrival_time = candidate_node.first->arrival_time;
                            candidate_node.first->arrival_time = arrival_time;
                        }

                        rewired_nodes.push_back(candidate_node.first);
                        is_rewired = true;
                        break;
                    }
                }
                if (is_rewired)
                {
                    break;
                }
                // assert(!is_collision_motion(start_coords, start_coords, node->arrival_time, departure_time));
            }
            if (is_rewired)
            {
                continue;
            }
        }
    }

    // rewired_nodes.clear();
    return rewired_nodes;
}

std::vector<MDP::MSIRRT::Vertex*> MDP::MSIRRT::PlannerConnect::get_final_path() const
{
    std::vector<MDP::MSIRRT::Vertex *> result;
    if (!this->goal_reached)
    {
        return result;
    }
    MDP::MSIRRT::Vertex *current = this->finish_node;

    while (current)
    {
        result.push_back(current);
        current = current->parent;
    }
    std::reverse(result.begin(), result.end());
    return result;
}

bool MDP::MSIRRT::PlannerConnect::check_planner_termination_condition() const
{
    if (this->stop_when_path_found)
    {
        return !this->goal_reached && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - this->solver_start_time).count() < this->max_planning_time;
    }
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - this->solver_start_time).count() < this->max_planning_time;
}

bool MDP::MSIRRT::PlannerConnect::is_coords_in_limits(const MDP::MSIRRT::Vertex &q) const
{
    return this->is_coords_in_limits(q.coords);
}
bool MDP::MSIRRT::PlannerConnect::is_coords_in_limits(const MDP::MSIRRT::Vertex::VertexCoordType &coords) const
{
    // assert(this->robot_limits.size() == coords.size());
    for (int joint_ind = 0; joint_ind < this->robot_limits.size(); joint_ind++)
    {
        if ((coords[joint_ind] < robot_limits[joint_ind].first) | (coords[joint_ind] > robot_limits[joint_ind].second))
        {
            return false;
        }
    }
    return true;
}

double MDP::MSIRRT::PlannerConnect::get_random_between_0_1()
{
    return this->probability_gen(this->gen);
}
MDP::MSIRRT::Vertex::VertexCoordType MDP::MSIRRT::PlannerConnect::get_random_configuration()
{
    MDP::MSIRRT::Vertex::VertexCoordType result;

    for (int joint_ind = 0; joint_ind < this->dof; joint_ind++)
    {
        result[joint_ind] = this->get_random_between_0_1() * (this->robot_limits[joint_ind].second - robot_limits[joint_ind].first) + robot_limits[joint_ind].first;
    }
    return result;
}

bool MDP::MSIRRT::PlannerConnect::extend(MDP::MSIRRT::Vertex::VertexCoordType &coords_of_new)
{
    MDP::MSIRRT::Vertex *q_nearest = this->get_nearest_node(coords_of_new); // find nearest neighbor

    // find new coords
    MDP::MSIRRT::Vertex::VertexCoordType delta_vector = coords_of_new - q_nearest->coords;
    double delta = delta_vector.norm();
    if (delta <= (1 / 100000000)) // zero division check
    {
        // std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
        return false;
    }

    if (delta < this->planner_range)
    {
        return true;
    }

    coords_of_new = q_nearest->coords + delta_vector.normalized() * this->planner_range;

    return true;
}
MDP::MSIRRT::Vertex *MDP::MSIRRT::PlannerConnect::get_nearest_node(const MDP::MSIRRT::Vertex::VertexCoordType &coords)
{
    const size_t num_results = 1;
    size_t ret_index;
    double out_dist_sqr;
    // nanoflann::SearchParams search_params;
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index, &out_dist_sqr);
    this->current_tree->kd_tree.findNeighbors(resultSet, coords.data(), {0});
    return &(this->current_tree->array_of_vertices[ret_index]);
}

std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> MDP::MSIRRT::PlannerConnect::get_nearest_node_by_radius(const MDP::MSIRRT::Vertex::VertexCoordType &coords, double radius, MDP::MSIRRT::Tree *tree)
{
    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> result;

    // Unsorted radius search
    std::vector<std::pair<size_t, double>> indices_dists;
    // search_params.sorted = false;
    nanoflann::RadiusResultSet<double, size_t> resultSet(radius, indices_dists);

    tree->kd_tree.findNeighbors(resultSet, coords.data(), {0});

    result.reserve(resultSet.m_indices_dists.size());
    for (auto node_ind_dist_pair : resultSet.m_indices_dists)
    {
        result.emplace_back(&(tree->array_of_vertices[node_ind_dist_pair.first]), node_ind_dist_pair.first);
    }

    return result;
}

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::PlannerConnect::set_parent(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand)
{
    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> nearest_nodes = this->get_nearest_node_by_radius(coord_rand, 9 * this->planner_range * this->planner_range, (this->current_tree));

    if (nearest_nodes.size() == 0)
    {
        // std::cout << "BBBB" << std::endl;
    }

    std::vector<MDP::MSIRRT::Vertex *> added_vertices;
    int safe_interval_ind = 0;
    if (this->current_tree == this->start_tree)
    {
        double time_to_goal = (coord_rand - this->goal_coords).norm() * (double)this->scene_task.fps / this->vmax;
        std::sort(nearest_nodes.begin(), nearest_nodes.end(), [](const std::pair<MDP::MSIRRT::Vertex *, int> &a, std::pair<MDP::MSIRRT::Vertex *, int> &b)
                  { return a.first->arrival_time < b.first->arrival_time; });
        // for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes){
        //     std::cout<<candidate_node.first->arrival_time<<", ";
        // }
        // std::cout<<std::endl;
        for (std::pair<int, int> safe_int : safe_intervals_of_coord_rand) // For each interval
        {
            // if we can't reach goal from that interval - skip
            if ((safe_int.first + time_to_goal) >= this->scene_task.frame_count)
            {
                safe_interval_ind++;
                continue;
            }
            bool found_parent = false;

            // for each parent
            for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes)
            {

                double time_to_node = (coord_rand - candidate_node.first->coords).norm() * (double)this->scene_task.fps / this->vmax;

                // candidate nodes are sorted by ascending arrival time. If arrival time > safe int bound + time to node - > break;
                if (candidate_node.first->arrival_time + time_to_node > safe_int.second)
                {
                    break;
                }

                // if intervals don't overlap - skip
                if ((candidate_node.first->safe_interval.second + time_to_node < safe_int.first))
                {
                    continue;
                }
                // assert(candidate_node.first);

                MDP::MSIRRT::Vertex::VertexCoordType start_coords = candidate_node.first->coords;

                // for each departure time
                for (double departure_time = std::max(candidate_node.first->arrival_time, (double)safe_int.first - time_to_node); departure_time <= std::min((double)candidate_node.first->safe_interval.second, (double)safe_int.second - time_to_node); departure_time += 1)
                {
                    // assert(!is_collision_motion(candidate_node.first->coords, candidate_node.first->coords, candidate_node.first->arrival_time, departure_time));
                    double arrival_time = departure_time + time_to_node;
                    // fix rounding errors
                    if (arrival_time > safe_intervals_of_coord_rand[safe_interval_ind].second)
                    {
                        departure_time -= arrival_time - safe_intervals_of_coord_rand[safe_interval_ind].second;
                        arrival_time -= arrival_time - safe_intervals_of_coord_rand[safe_interval_ind].second;
                    }
                    if (arrival_time < safe_intervals_of_coord_rand[safe_interval_ind].first)
                    {
                        departure_time += safe_intervals_of_coord_rand[safe_interval_ind].first - arrival_time;
                        arrival_time += safe_intervals_of_coord_rand[safe_interval_ind].first - arrival_time;
                    }
                    if (!is_collision_motion(start_coords, coord_rand, departure_time, arrival_time))
                    {
                        // std::cout <<departure_time<<" "<<candidate_node.first->arrival_time<<" "<<candidate_node.first->safe_interval.first<<" "<<candidate_node.first->safe_interval.second<<std::endl;
                        // assert(!is_collision_motion(start_coords, start_coords, candidate_node.first->arrival_time, departure_time));
                        // assert((safe_intervals_of_coord_rand[safe_interval_ind].first <= arrival_time && safe_intervals_of_coord_rand[safe_interval_ind].second >= arrival_time) || fabs(safe_intervals_of_coord_rand[safe_interval_ind].second - arrival_time) < 0.001 || fabs(safe_intervals_of_coord_rand[safe_interval_ind].first - arrival_time) < 0.001);
                        // assert((candidate_node.first->safe_interval.first <= departure_time && candidate_node.first->safe_interval.second >= departure_time));
                        this->current_tree->add_vertex(MDP::MSIRRT::Vertex(coord_rand, safe_intervals_of_coord_rand[safe_interval_ind]), candidate_node.first, departure_time, arrival_time);
                        added_vertices.push_back(&(this->current_tree->array_of_vertices.back()));
                        found_parent = true;
                        break;
                    }
                }
                if (found_parent)
                {
                    break;
                }
            }

            if (!found_parent)
            {
                this->orphan_tree->add_vertex(MDP::MSIRRT::Vertex(coord_rand, safe_intervals_of_coord_rand[safe_interval_ind]), nullptr, -1, safe_intervals_of_coord_rand[safe_interval_ind].second + 1);
            }
            safe_interval_ind++;
        }
    }
    else if (this->current_tree == this->goal_tree)
    {
        double time_to_start = (coord_rand - this->root_node->coords).norm() * (double)this->scene_task.fps / this->vmax;
        std::sort(nearest_nodes.begin(), nearest_nodes.end(), [](const std::pair<MDP::MSIRRT::Vertex *, int> &a, std::pair<MDP::MSIRRT::Vertex *, int> &b)
                  { return a.first->arrival_time > b.first->arrival_time; });
        // for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes){
        //     std::cout<<candidate_node.first->arrival_time<<", ";
        // }
        // std::cout<<std::endl;
        for (std::pair<int, int> safe_int : safe_intervals_of_coord_rand) // For each interval
        {
            // if we can't reach start from that interval - skip
            if ((safe_int.second - time_to_start) < 0)
            {
                safe_interval_ind++;
                continue;
            }
            bool found_parent = false;

            // for each parent
            for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes)
            {

                double time_to_node = (coord_rand - candidate_node.first->coords).norm() * (double)this->scene_task.fps / this->vmax;

                // candidate nodes are sorted by descending arrival time. If arrival time of parent < safe int bound + time to node - > break;
                if (candidate_node.first->arrival_time - time_to_node < safe_int.first)
                {
                    break;
                }

                // if intervals don't overlap - skip
                if ((candidate_node.first->safe_interval.first - time_to_node > safe_int.second))
                {
                    continue;
                }
                // assert(candidate_node.first);

                MDP::MSIRRT::Vertex::VertexCoordType start_coords = candidate_node.first->coords;

                // for each departure time
                for (double departure_time = std::min(candidate_node.first->arrival_time, (double)safe_int.second + time_to_node); departure_time >= std::max((double)candidate_node.first->safe_interval.first, (double)safe_int.first + time_to_node); departure_time -= 1)
                {
                    // assert(!is_collision_motion(candidate_node.first->coords, candidate_node.first->coords, candidate_node.first->arrival_time, departure_time));
                    double arrival_time = departure_time - time_to_node; // we travel to the past
                    // fix rounding errors
                    if (arrival_time > safe_intervals_of_coord_rand[safe_interval_ind].second)
                    {
                        departure_time -= arrival_time - safe_intervals_of_coord_rand[safe_interval_ind].second;
                        arrival_time -= arrival_time - safe_intervals_of_coord_rand[safe_interval_ind].second;
                    }
                    if (arrival_time < safe_intervals_of_coord_rand[safe_interval_ind].first)
                    {
                        departure_time += safe_intervals_of_coord_rand[safe_interval_ind].first - arrival_time;
                        arrival_time += safe_intervals_of_coord_rand[safe_interval_ind].first - arrival_time;
                    }
                    if (!is_collision_motion(coord_rand, start_coords, arrival_time, departure_time))
                    {
                        // assert(!is_collision_motion(start_coords, start_coords, departure_time, candidate_node.first->arrival_time));
                        // std::cout<<safe_intervals_of_coord_rand[safe_interval_ind].first<<" "<<safe_intervals_of_coord_rand[safe_interval_ind].second<<" "<<arrival_time<<std::endl;
                        // assert((safe_intervals_of_coord_rand[safe_interval_ind].first <= arrival_time && safe_intervals_of_coord_rand[safe_interval_ind].second >= arrival_time) || fabs(safe_intervals_of_coord_rand[safe_interval_ind].second - arrival_time) < 0.001 || fabs(safe_intervals_of_coord_rand[safe_interval_ind].first - arrival_time) < 0.001);
                        this->current_tree->add_vertex(MDP::MSIRRT::Vertex(coord_rand, safe_intervals_of_coord_rand[safe_interval_ind]), candidate_node.first, departure_time, arrival_time);
                        added_vertices.push_back(&(this->current_tree->array_of_vertices.back()));
                        found_parent = true;
                        break;
                    }
                }
                if (found_parent)
                {
                    break;
                }
            }

            if (!found_parent)
            {
                this->orphan_tree->add_vertex(MDP::MSIRRT::Vertex(coord_rand, safe_intervals_of_coord_rand[safe_interval_ind]), nullptr, -1, safe_intervals_of_coord_rand[safe_interval_ind].second + 1);
            }
            safe_interval_ind++;
        }
    }

    return added_vertices;
}

std::pair<int, int> MDP::MSIRRT::PlannerConnect::calculate_delta(MDP::MSIRRT::Vertex *candidate_node, MDP::MSIRRT::Vertex::VertexCoordType &end_coords, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand, int &safe_interval_ind)
{
}

bool MDP::MSIRRT::PlannerConnect::is_goal(const MDP::MSIRRT::Vertex::VertexCoordType &coord)
{
    MDP::MSIRRT::Vertex::VertexCoordType delta = this->goal_coords - coord;
    double delta_norm = delta.norm();
    return delta_norm < 0.01;
}

bool MDP::MSIRRT::PlannerConnect::is_collision_state(MDP::MSIRRT::Vertex::VertexCoordType &coords, int &time)
{
    std::vector<double> robot_angles;
    for (int joint_int = 0; joint_int < coords.size(); joint_int++)
    {
        robot_angles.push_back((double)coords[joint_int]);
    }
    return this->collision_manager.check_collision_frame(robot_angles, time);
}

bool MDP::MSIRRT::PlannerConnect::is_collision_motion(const MDP::MSIRRT::Vertex::VertexCoordType &start_coords, const MDP::MSIRRT::Vertex::VertexCoordType &end_coords, double &start_time, double &end_time)
{

    if (start_time > end_time)
    {
        // std::cout << "start_time > end_time" << std::endl;
        return true;
    }
    MDP::MSIRRT::Vertex::VertexCoordType dir_vector = end_coords - start_coords;
    if ((dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time))) > this->vmax)
    {
        // std::cout << "((dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time))) >= this->vmax)" << std::endl;
        return true;
    }

    int interpolation_steps = std::max({(int)(dir_vector.norm() / 0.1), (int)(end_time - start_time + 0.5), 1});

    // std::cout<<"interpolation_steps: "<<interpolation_steps<<" "<< dir_vector.norm()<<std::endl;
    for (int step = 0; step <= interpolation_steps; step++)
    {

        MDP::MSIRRT::Vertex::VertexCoordType temp_coords = start_coords + dir_vector * (double)step / (double)interpolation_steps;
        int time_frame = start_time + (double)(end_time - start_time) * (double)step / (double)interpolation_steps;

        if (this->is_collision_state(temp_coords, time_frame))
        {
            // std::cout << "is_collision!!" << std::endl;

            return true;
        }
    }
    return false;
}

bool MDP::MSIRRT::PlannerConnect::check_path(std::vector<MDP::MSIRRT::Vertex*> &path)
{

    for (int node_id = 0; node_id < path.size() - 1; node_id++)
    {
        if (is_collision_motion(path[node_id]->coords, path[node_id]->coords, path[node_id]->arrival_time, path[node_id + 1]->departure_from_parent_time) &&
            is_collision_motion(path[node_id]->coords, path[node_id + 1]->coords, path[node_id + 1]->departure_from_parent_time, path[node_id + 1]->arrival_time))
        {
            return false;
        }
        if (!((path[node_id]->safe_interval.first <= path[node_id]->arrival_time) && (path[node_id]->safe_interval.second >= path[node_id]->arrival_time)))
        {
            return false;
        }
        if (!((path[node_id]->safe_interval.first <= path[node_id + 1]->departure_from_parent_time) && (path[node_id]->safe_interval.second >= path[node_id + 1]->departure_from_parent_time)))
        {
            return false;
        }
    }
    return true;
}

void MDP::MSIRRT::PlannerConnect::prune_goal_tree()
{
    assert(this->goal_reached);

    MDP::MSIRRT::Vertex *start_tree_node = this->goal_nodes.first;
    MDP::MSIRRT::Vertex *goal_tree_node_child = this->goal_nodes.second;

    assert(goal_tree_node_child->parent); // Must have parent, because RRTConnect can't just connect to root node
    MDP::MSIRRT::Vertex *goal_tree_node = goal_tree_node_child->parent; 
    
    while (goal_tree_node)
    {
        double time_to_node = (start_tree_node->coords - goal_tree_node->coords).norm() * (double)this->scene_task.fps / this->vmax;

        MDP::MSIRRT::Vertex::VertexCoordType start_coords = start_tree_node->coords;
        bool found_dep_time = false;
        // for each departure time
        for (double departure_time = std::max(start_tree_node->arrival_time, (double)goal_tree_node->safe_interval.first - time_to_node); departure_time <= goal_tree_node_child->arrival_time; departure_time += 1)
        {
            
            // assert(!is_collision_motion(start_coords, start_coords, start_tree_node->arrival_time, departure_time));
            double arrival_time = departure_time + time_to_node;
            if (!is_collision_motion(start_coords, goal_tree_node->coords, departure_time, arrival_time))
            {
                this->start_tree->add_vertex(MDP::MSIRRT::Vertex(goal_tree_node->coords, goal_tree_node->safe_interval), start_tree_node, departure_time, arrival_time);
                start_tree_node = &(this->start_tree->array_of_vertices.back());
                found_dep_time = true;
                break;
            }
        }
        if(!found_dep_time)
        {
            this->start_tree->add_vertex(MDP::MSIRRT::Vertex(goal_tree_node->coords, goal_tree_node->safe_interval), start_tree_node, goal_tree_node_child->arrival_time, goal_tree_node_child->departure_from_parent_time);
            start_tree_node = &(this->start_tree->array_of_vertices.back());
        }
        goal_tree_node_child = goal_tree_node;
        goal_tree_node = goal_tree_node->parent;

    }
    this->finish_node = start_tree_node;
}