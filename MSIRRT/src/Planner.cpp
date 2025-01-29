
#include <vector>

#include "Planner.hpp"
#include "config_read_writer/config_read.hpp"

MDP::MSIRRT::Planner::Planner(MDP::ConfigReader::SceneTask scene_task_) : scene_task(scene_task_),
                                                                          collision_manager(scene_task_), probability_gen(0.0, 1.0),
                                                                          gen(rd()), rd()
{

    this->start_tree.array_of_vertices.reserve(1000000); // TODO: decide to leave it or remove it

    this->orphan_tree.array_of_vertices.reserve(1000000); // TODO: decide to leave it or remove it

    this->robot_limits = this->collision_manager.get_planned_robot_limits();

    assert(is_coords_in_limits(MDP::MSIRRT::Vertex::VertexCoordType(scene_task.start_configuration.data()))); // check start conf bound;

    assert(is_coords_in_limits(MDP::MSIRRT::Vertex::VertexCoordType(scene_task.end_configuration.data()))); // check end conf bounds;
    this->dof = scene_task.start_configuration.size();
    std::vector<std::pair<int, int>> start_safe_intervals = this->collision_manager.get_safe_intervals(scene_task.start_configuration); // get start cond save intervals;

    // сheck if first start at 0
    assert(start_safe_intervals.size() > 0);
    assert(start_safe_intervals[0].first == 0);
    // todo safe start intervals as orphan
    this->start_tree.add_vertex(MDP::MSIRRT::Vertex(scene_task.start_configuration, start_safe_intervals[0]), nullptr, -1, 0);
    for (int another_safe_interval_id = 1; another_safe_interval_id < start_safe_intervals.size(); another_safe_interval_id++)
    {
        this->orphan_tree.add_vertex(MDP::MSIRRT::Vertex(scene_task.start_configuration, start_safe_intervals[another_safe_interval_id]), nullptr, -1, -1);
    }

    this->root_node = &this->start_tree.array_of_vertices[0];
    this->root_node->arrival_time = 0;

    this->goal_coords = MDP::MSIRRT::Vertex::VertexCoordType(scene_task.end_configuration.data());

    this->max_planning_time = 30;
    this->stop_when_path_found = true;
    this->goal_bias = 0.1;
    this->goal_sampled = false;
    this->goal_safe_intervals = this->collision_manager.get_safe_intervals(scene_task.end_configuration);
    this->planner_range = 1;
    this->vmax = 3.1415926535;
}
// destructor
MDP::MSIRRT::Planner::~Planner()
{
}

bool MDP::MSIRRT::Planner::solve()
{
    this->solver_start_time = std::chrono::steady_clock::now();
    int iter = 0;
    while (check_planner_termination_condition() && !this->goal_reached)
    {
        std::cout << "iteration #" << iter++ << std::endl;
        MDP::MSIRRT::Vertex::VertexCoordType coord_rand;
        this->goal_sampled = this->goal_bias > this->get_random_between_0_1();
        if (this->goal_sampled)
        {
            coord_rand = this->goal_coords;
        }
        else
        {
            coord_rand = this->get_random_configuration();
        }
        MDP::MSIRRT::Vertex::VertexCoordType original_coords_rand = coord_rand;
        while (true)
        {

            bool is_ok = this->extend(coord_rand);
            if (!is_ok)
            {
                break;
            }
            std::vector<double> robot_angles(coord_rand.data(), coord_rand.data() + coord_rand.rows() * coord_rand.cols());

            std::vector<std::pair<int, int>> safe_intervals_of_coord_rand;

            safe_intervals_of_coord_rand = this->collision_manager.get_safe_intervals(robot_angles);

            std::vector<MDP::MSIRRT::Vertex *> array_of_new_nodes = this->set_parent(coord_rand, safe_intervals_of_coord_rand);

            if (array_of_new_nodes.size() == 0)
            {
                std::cout << "no_new_node" << std::endl;

                break;
            }
            std::cout << "find_new_node!" << std::endl;
            std::cout << "tree size:" << this->start_tree.array_of_vertices.size() << "  " << this->orphan_tree.array_of_vertices.size() << std::endl;

            while (array_of_new_nodes.size() != 0)
            {
                MDP::MSIRRT::Vertex *node = array_of_new_nodes.back();
                std::vector<MDP::MSIRRT::Vertex *> rewired_nodes = this->rewire(node); // rewire

                if (this->is_goal(node->coords))
                {
                    std::cout << "goal reached!" << std::endl;
                    this->goal_node = node;
                    this->goal_reached = true;
                    this->goal_sampled = false;
                }

                array_of_new_nodes.pop_back();
                if (rewired_nodes.size() != 0)
                {
                    array_of_new_nodes.reserve(array_of_new_nodes.size() + std::distance(rewired_nodes.begin(), rewired_nodes.end()));
                    array_of_new_nodes.insert(array_of_new_nodes.end(), rewired_nodes.begin(), rewired_nodes.end());
                }
            }
            coord_rand = original_coords_rand;
            if (this->goal_reached)
            {
                break;
            }
        }
    }
    return this->goal_reached;
}

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::Planner::rewire(MDP::MSIRRT::Vertex *node)
{

    std::vector<MDP::MSIRRT::Vertex *> rewired_nodes;

    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> nearest_nodes = this->get_nearest_node_by_radius(node->coords, 4 * this->planner_range * this->planner_range, &(this->start_tree));
    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> nearest_nodes_orphan_tree = this->get_nearest_node_by_radius(node->coords, 4 * this->planner_range * this->planner_range, &(this->orphan_tree));
    nearest_nodes.reserve(nearest_nodes.size() + std::distance(nearest_nodes_orphan_tree.begin(), nearest_nodes_orphan_tree.end()));
    nearest_nodes.insert(nearest_nodes.end(), nearest_nodes_orphan_tree.begin(), nearest_nodes_orphan_tree.end());

    if (nearest_nodes.size() == 0)
    {
        std::cout << "BBBB" << std::endl;
    }
    double max_frame = node->before_rewiring_arrival_time;
    if (max_frame < 0)
    {
        max_frame = node->safe_interval.second;
    }

    node->before_rewiring_arrival_time = -1;

    // for each candidate
    for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes)
    {
        bool is_rewired = false;
        assert(candidate_node.first);
        if ((candidate_node.first == node) || (candidate_node.first == node->parent))
        {
            continue;
        }
        if (candidate_node.first->safe_interval.first == candidate_node.first->arrival_time) // Can't do better
        {
            continue;
        }

        double time_to_node = (node->coords - candidate_node.first->coords).norm() * (double)this->scene_task.fps / this->vmax;

        // don't overlap
        if ((node->arrival_time + time_to_node > candidate_node.first->arrival_time) || (max_frame + time_to_node < candidate_node.first->safe_interval.first))
        {
            continue;
        }

        MDP::MSIRRT::Vertex::VertexCoordType start_coords = node->coords;

        for (double departure_time = std::max(node->arrival_time, (double)candidate_node.first->safe_interval.first - time_to_node); departure_time <= std::min(max_frame, (double)candidate_node.first->arrival_time - time_to_node); departure_time += 1)
        {
            // if (departure_time > candidate_node.first->arrival_time)
            // {
            //     break;
            // }
            double arrival_time = departure_time + time_to_node;
            if (!is_collision_motion(start_coords, candidate_node.first->coords, departure_time, arrival_time))
            {

                if (!candidate_node.first->parent) // if was orphan
                {
                    this->start_tree.add_vertex(*(candidate_node.first), node, departure_time, arrival_time);

                    this->orphan_tree.delete_vertex(candidate_node.second);

                    rewired_nodes.push_back(&(this->start_tree.array_of_vertices.back()));
                    is_rewired = true;
                }
                else
                {
                    candidate_node.first->parent = node;
                    candidate_node.first->departure_from_parent_time = departure_time;
                    candidate_node.first->before_rewiring_arrival_time = candidate_node.first->arrival_time;
                    candidate_node.first->arrival_time = arrival_time;
                    rewired_nodes.push_back(candidate_node.first);
                }

                is_rewired = true;
                break;
            }
        }

        // assert(!is_collision_motion(start_coords, start_coords, node->arrival_time, departure_time));

        if (is_rewired)
        {
            continue;
        }
    }
    rewired_nodes.clear();
    return rewired_nodes;
}

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::Planner::get_final_path() const
{
    std::vector<MDP::MSIRRT::Vertex *> result;
    if (!this->goal_reached)
    {
        return result;
    }
    MDP::MSIRRT::Vertex *current = this->goal_node;

    while (current)
    {
        result.push_back(current);
        current = current->parent;
    }
    std::reverse(result.begin(), result.end());
    return result;
}

bool MDP::MSIRRT::Planner::check_planner_termination_condition() const
{
    if (this->stop_when_path_found)
    {
        return !this->goal_reached && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - this->solver_start_time).count() < this->max_planning_time;
    }
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - this->solver_start_time).count() < this->max_planning_time;
}

bool MDP::MSIRRT::Planner::is_coords_in_limits(const MDP::MSIRRT::Vertex &q) const
{
    return this->is_coords_in_limits(q.coords);
}
bool MDP::MSIRRT::Planner::is_coords_in_limits(const MDP::MSIRRT::Vertex::VertexCoordType &coords) const
{
    assert(this->robot_limits.size() == coords.size());
    for (int joint_ind = 0; joint_ind < this->robot_limits.size(); joint_ind++)
    {
        if ((coords[joint_ind] < robot_limits[joint_ind].first) | (coords[joint_ind] > robot_limits[joint_ind].second))
        {
            return false;
        }
    }
    return true;
}

double MDP::MSIRRT::Planner::get_random_between_0_1()
{
    return this->probability_gen(this->gen);
}
MDP::MSIRRT::Vertex::VertexCoordType MDP::MSIRRT::Planner::get_random_configuration()
{
    MDP::MSIRRT::Vertex::VertexCoordType result;

    for (int joint_ind = 0; joint_ind < this->dof; joint_ind++)
    {
        result[joint_ind] = this->get_random_between_0_1() * (this->robot_limits[joint_ind].second - robot_limits[joint_ind].first) + robot_limits[joint_ind].first;
    }
    return result;
}

bool MDP::MSIRRT::Planner::extend(MDP::MSIRRT::Vertex::VertexCoordType &coords_of_new)
{
    MDP::MSIRRT::Vertex *q_nearest = this->get_nearest_node(coords_of_new); // find nearest neighbor

    // find new coords
    MDP::MSIRRT::Vertex::VertexCoordType delta_vector = coords_of_new - q_nearest->coords;
    double delta = delta_vector.norm();
    if (delta <= (1 / 100000000)) // zero division check
    {
        std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
        return false;
    }

    if (delta < this->planner_range)
    {
        return true;
    }

    coords_of_new = q_nearest->coords + delta_vector.normalized() * this->planner_range;

    return true;
}
MDP::MSIRRT::Vertex *MDP::MSIRRT::Planner::get_nearest_node(const MDP::MSIRRT::Vertex::VertexCoordType &coords)
{
    const size_t num_results = 1;
    size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index, &out_dist_sqr);
    this->start_tree.kd_tree.findNeighbors(resultSet, coords.data(), {0});
    return &(this->start_tree.array_of_vertices[ret_index]);
}

std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> MDP::MSIRRT::Planner::get_nearest_node_by_radius(const MDP::MSIRRT::Vertex::VertexCoordType &coords, double radius, MDP::MSIRRT::Tree *tree)
{
    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> result;

    // Unsorted radius search
    std::vector<std::pair<size_t, double>> indices_dists;
    // nanoflann::SearchParams search_params;
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

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::Planner::set_parent(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand)
{
    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> nearest_nodes = this->get_nearest_node_by_radius(coord_rand, 9 * this->planner_range * this->planner_range, &(this->start_tree));
    if (nearest_nodes.size() == 0)
    {
        std::cout << "BBBB" << std::endl;
    }
    std::vector<MDP::MSIRRT::Vertex *> added_vertices;
    int safe_interval_ind = 0;
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
                if (!is_collision_motion(start_coords, coord_rand, departure_time, arrival_time))
                {
                    this->start_tree.add_vertex(MDP::MSIRRT::Vertex(coord_rand, safe_intervals_of_coord_rand[safe_interval_ind]), candidate_node.first, departure_time, arrival_time);
                    added_vertices.push_back(&(this->start_tree.array_of_vertices.back()));
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
            this->orphan_tree.add_vertex(MDP::MSIRRT::Vertex(coord_rand, safe_intervals_of_coord_rand[safe_interval_ind]), nullptr, -1, safe_intervals_of_coord_rand[safe_interval_ind].second + 1);
        }
        safe_interval_ind++;
    }

    return added_vertices;
}

std::pair<int, int> MDP::MSIRRT::Planner::calculate_delta(MDP::MSIRRT::Vertex *candidate_node, MDP::MSIRRT::Vertex::VertexCoordType &end_coords, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand, int &safe_interval_ind)
{
}

bool MDP::MSIRRT::Planner::is_goal(const MDP::MSIRRT::Vertex::VertexCoordType &coord)
{
    MDP::MSIRRT::Vertex::VertexCoordType delta = this->goal_coords - coord;
    double delta_norm = delta.norm();
    return delta_norm < 0.01;
}

bool MDP::MSIRRT::Planner::is_collision_state(MDP::MSIRRT::Vertex::VertexCoordType &coords, int &time)
{
    std::vector<double> robot_angles;
    for (int joint_int = 0; joint_int < coords.size(); joint_int++)
    {
        robot_angles.push_back((double)coords[joint_int]);
    }
    return this->collision_manager.check_collision_frame(robot_angles, time);
}

bool MDP::MSIRRT::Planner::is_collision_motion(const MDP::MSIRRT::Vertex::VertexCoordType &start_coords, const MDP::MSIRRT::Vertex::VertexCoordType &end_coords, double &start_time, double &end_time)
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
