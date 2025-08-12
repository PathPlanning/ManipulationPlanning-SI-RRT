
#include <vector>

#include "PlannerConnect.hpp"
#include "config_read_writer/config_read.hpp"

MDP::MSIRRT::PlannerConnect::PlannerConnect(MDP::ConfigReader::SceneTask scene_task_, int random_seed) : scene_task(scene_task_),
                                                                                                         collision_manager(scene_task_), probability_gen(0.0, 1.0),
                                                                                                         gen(random_seed), rd()
{
    this->dof = scene_task.start_configuration.size();
    this->start_tree = new MDP::MSIRRT::Tree("start_tree", 0, this->dof);
    this->goal_tree = new MDP::MSIRRT::Tree("goal_tree", 1, this->dof);
    this->orphan_tree = new MDP::MSIRRT::Tree("orphan_tree", -1, this->dof);
    this->start_tree->array_of_vertices.reserve(1000000);  // TODO: decide to leave it or remove it
    this->goal_tree->array_of_vertices.reserve(1000000);   // TODO: decide to leave it or remove it
    this->orphan_tree->array_of_vertices.reserve(1000000); // TODO: decide to leave it or remove it

    this->robot_limits = this->collision_manager.get_planned_robot_limits();

    assert(is_coords_in_limits(MDP::MSIRRT::Vertex::VertexCoordType(scene_task.start_configuration.data()))); // check start conf bound;

    assert(is_coords_in_limits(MDP::MSIRRT::Vertex::VertexCoordType(scene_task.end_configuration.data()))); // check end conf bounds;

    // std::cout<<"--------------------------------------------"<<std::endl;
    // std::cout<<"Get staret tree safe intervals!!!!"<<std::endl;
    std::vector<std::pair<int, int>> start_safe_intervals = this->collision_manager.get_safe_intervals(scene_task.start_configuration); // get start cond save intervals;

    // сheck if first start at 0
    assert(start_safe_intervals.size() > 0);
    assert(start_safe_intervals[0].first == 0);

    this->start_tree->add_vertex(scene_task.start_configuration, start_safe_intervals[0], nullptr, -1, 0);
    
    // std::cout<<"start safe intervals:"<<std::endl;
    // for (int another_safe_interval_id = 1; another_safe_interval_id < start_safe_intervals.size(); another_safe_interval_id++)
    // {
    //     // std::cout<< start_safe_intervals[another_safe_interval_id].first<<" "<<start_safe_intervals[another_safe_interval_id].second<<std::endl;
    //     this->orphan_tree->add_vertex(scene_task.start_configuration, start_safe_intervals[another_safe_interval_id], nullptr, -1, -1);
    // }

    this->root_node = this->start_tree->array_of_vertices[0];
    this->root_node->arrival_time = 0;

    this->goal_coords = MDP::MSIRRT::Vertex::VertexCoordType(scene_task.end_configuration.data());
    this->goal_safe_intervals = this->collision_manager.get_safe_intervals(scene_task.end_configuration);

    assert(this->goal_safe_intervals.size()>0);
    if (this->goal_safe_intervals.size()==0){
        std::cout<<"this->goal_safe_intervals.size()==0" <<std::endl;
        std::exit(1);
    }
    // std::cout<<"goal safe intervals:"<<std::endl;
    for (const std::pair<int,int>& safe_int:this->goal_safe_intervals){
    //     std::cout<< safe_int.first<<" "<<safe_int.second<<std::endl;
        this->goal_tree->add_vertex(this->goal_coords, safe_int, nullptr, -1, safe_int.second);

    }
    //// this->goal_tree->add_vertex(this->goal_coords, this->goal_safe_intervals.back(), nullptr, -1, this->goal_safe_intervals.back().second);
    
    // сheck if the latest safe interval at goal is safe
    // assert(this->goal_safe_intervals.back().second == scene_task.frame_count - 1);
    this->current_tree = this->start_tree;
    this->other_tree = this->goal_tree;
    this->goal_reached = false;
    this->max_planning_time = 20;      // TODO: remove hardcode
    this->stop_when_path_found = true; // TODO: remove hardcode
    this->planner_range = 3;           // TODO: remove hardcode
    this->vmax = 3.1415;               // TODO: remove hardcode
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
    
    if(this->root_node->safe_interval.second < scene_task.frame_count/5){
        this->warmup_start_tree();
    }

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

        if (new_nodes.size() == 0 && this->was_static_obstacle)
        {
            coord_rand = this->last_valid_coord;
            safe_intervals_of_coord_rand = this->collision_manager.get_safe_intervals(std::vector<double>(coord_rand.data(), coord_rand.data() + coord_rand.rows() * coord_rand.cols()));
            new_nodes = this->grow_tree(coord_rand, safe_intervals_of_coord_rand);
        }   
        // assert(this->was_static_obstacle);
        if(new_nodes.size() == 0){
            this->swap_trees();
            continue;
        }
        bool connected = connect_trees(coord_rand, safe_intervals_of_coord_rand, new_nodes);
        this->swap_trees();
    }
    return this->goal_reached;
}

void MDP::MSIRRT::PlannerConnect::warmup_start_tree()
{
    // std::cout<<"warmup_start_tree"<<std::endl;
    this->current_tree = this->start_tree;
    this->other_tree = this->goal_tree;
    double old_planner_range = this->planner_range;
    this->planner_range = 1.5*this->root_node->safe_interval.second/this->scene_task.fps * this->vmax;
    for(int i = 0; i < 25; i++){
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

        if (new_nodes.size() == 0 && this->was_static_obstacle)
        {
            coord_rand = this->last_valid_coord;
            safe_intervals_of_coord_rand = this->collision_manager.get_safe_intervals(std::vector<double>(coord_rand.data(), coord_rand.data() + coord_rand.rows() * coord_rand.cols()));
            new_nodes = this->grow_tree(coord_rand, safe_intervals_of_coord_rand);
        }   
        // if (new_nodes.size() != 0){
        //     std::cout<<"warmup_start_tree: new_nodes.size() != 0"<<std::endl;
        // }
    }

    this->planner_range = old_planner_range;
}


void MDP::MSIRRT::PlannerConnect::swap_trees()
{
    std::swap(this->current_tree, this->other_tree);
}

bool MDP::MSIRRT::PlannerConnect::connect_trees(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand, std::vector<MDP::MSIRRT::Vertex *> another_tree_new_nodes)
{
    const MDP::MSIRRT::Vertex::VertexCoordType original_coord_rand = coord_rand;

    while (this->check_planner_termination_condition())
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
        if (new_nodes.size() == 0 && this->was_static_obstacle)
        {
            coord_rand = this->last_valid_coord;
            safe_intervals_of_coord_rand = this->collision_manager.get_safe_intervals(std::vector<double>(coord_rand.data(), coord_rand.data() + coord_rand.rows() * coord_rand.cols()));
            new_nodes = this->grow_tree(coord_rand, safe_intervals_of_coord_rand);
            // assert(this->was_static_obstacle);
            break; // because we hit static obstacle

        }   
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

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::PlannerConnect::grow_tree(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand)
{
    std::vector<MDP::MSIRRT::Vertex *> result;
    std::vector<MDP::MSIRRT::Vertex *> array_of_new_nodes = this->set_parent(coord_rand, safe_intervals_of_coord_rand);

    if (array_of_new_nodes.size() == 0)
    {
        // std::cout << "no_new_node" << std::endl;
        return array_of_new_nodes;
    }
    // result.reserve(result.size() + std::distance(array_of_new_nodes.begin(), array_of_new_nodes.end()));
    // result.insert(result.end(), array_of_new_nodes.begin(), array_of_new_nodes.end());

    // std::cout << "find_new_node!" << std::endl;
    // std::cout << "tree size:" << this->start_tree->array_of_vertices.size() << " " << this->goal_tree->array_of_vertices.size() << "  " << this->orphan_tree->array_of_vertices.size() << std::endl;

    return array_of_new_nodes;
}

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::PlannerConnect::get_final_path() const
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
        return false;
    }

    if (delta < this->planner_range)
    {
        return true;
    }

    coords_of_new = q_nearest->coords + delta_vector.normalized() * this->planner_range;

    MDP::MSIRRT::Vertex *q_nearest_of_new = this->get_nearest_node(coords_of_new); // find nearest neighbor

     delta_vector = coords_of_new - q_nearest_of_new->coords;
     delta = delta_vector.norm();
    if (delta <= (1 / 100000000)) // zero division check
    {
        return false;   
    }
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
    return this->current_tree->array_of_vertices[ret_index];
}

std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> MDP::MSIRRT::PlannerConnect::get_nearest_node_by_radius(const MDP::MSIRRT::Vertex::VertexCoordType &coords, double radius, MDP::MSIRRT::Tree *tree)
{
    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> result;

    // Unsorted radius search
    std::vector<nanoflann::ResultItem<size_t, double>> indices_dists;
    // search_params.sorted = false;
    nanoflann::RadiusResultSet<double, size_t> resultSet(radius, indices_dists);

    tree->kd_tree.findNeighbors(resultSet, coords.data(), {0});

    result.reserve(resultSet.m_indices_dists.size());
    for (auto node_ind_dist_pair : resultSet.m_indices_dists)
    {
        result.emplace_back(tree->array_of_vertices[node_ind_dist_pair.first], node_ind_dist_pair.first);
    }

    return result;
}

std::vector<MDP::MSIRRT::Vertex *> MDP::MSIRRT::PlannerConnect::set_parent(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand)
{
    this->was_static_obstacle = false;
    std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> nearest_nodes = this->get_nearest_node_by_radius(coord_rand,25* this->planner_range * this->planner_range, (this->current_tree));

    std::vector<MDP::MSIRRT::Vertex *> added_vertices;
    // int safe_interval_ind = 0;
    
    
    if (this->current_tree == this->start_tree)
    {   
        // int max_dep_time_for_safe_int_check = 3;
        // int col_coord_safe_int_second = -1;
        // int col_coord_safe_int_first = -max_dep_time_for_safe_int_check-1;
        // std::cout<<"set_parent start_tree"<<std::endl;
        double time_to_goal = (coord_rand - this->goal_coords).norm() * (double)this->scene_task.fps / this->vmax;
        // for (const std::pair<MDP::MSIRRT::Vertex *, int> &node : nearest_nodes){
        //     if (!node.first){
        //         std::cout<<"null pointer in sort!"<<std::endl;
        //         assert(false);
        //     }
        // }
        std::sort(nearest_nodes.begin(), nearest_nodes.end(), [&coord_rand,this](const std::pair<MDP::MSIRRT::Vertex *, int> &a, std::pair<MDP::MSIRRT::Vertex *, int> &b)
                  { 
                    // Add null pointer checks to prevent segfault
                    if (!a.first || !b.first) {
                        std::cout<<"null pointer in sort!"<<std::endl;

                        
                        return a.first != nullptr; // null pointers go to the end
                    }
                    return a.first->arrival_time+((coord_rand - a.first->coords).norm() * (double)this->scene_task.fps / this->vmax) < b.first->arrival_time+((coord_rand - b.first->coords).norm() * (double)this->scene_task.fps / this->vmax); 
                  });
                  // std::sort(nearest_nodes.begin(), nearest_nodes.end(), [](const std::pair<MDP::MSIRRT::Vertex *, int> &a, std::pair<MDP::MSIRRT::Vertex *, int> &b)
        //           { return a.first->arrival_time < b.first->arrival_time; });

        // for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes){
        //     std::cout<<candidate_node.first->arrival_time<<", ";
        // }
        // std::cout<<std::endl;
        for (std::pair<int, int> safe_int : safe_intervals_of_coord_rand) // For each interval
        {
            // std::cout<<"safe_int: "<<safe_int.first<<" "<<safe_int.second<<std::endl;
            // if we can't reach goal from that interval - skip
            if ((safe_int.first + time_to_goal) >= this->scene_task.frame_count)
            {
                // safe_interval_ind++;
                continue;
            }
            bool found_parent = false;

            // for each parent
            for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes)
            {
                // std::cout<<"candidate_node: "<<candidate_node.first->arrival_time<<" "<<candidate_node.first->safe_interval.first<<" "<<candidate_node.first->safe_interval.second<<std::endl;

                double time_to_node = (coord_rand - candidate_node.first->coords).norm() * (double)this->scene_task.fps / this->vmax;
                if (time_to_node<1){ // duplicate... this point was already added
                    found_parent = true;
                    break;
                }
                // candidate nodes are sorted by ascending arrival time. If arrival time > safe int bound + time to node -> break;
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
                    // std::cout<<"departure_time: "<<departure_time<<std::endl;
                    // assert(!is_collision_motion(candidate_node.first->coords, candidate_node.first->coords, candidate_node.first->arrival_time, departure_time));
                    double arrival_time = departure_time + time_to_node;
                    // fix rounding errors
                    if (arrival_time > safe_int.second)
                    {
                        //TODO: Check, if this breaks everything
                        departure_time -= arrival_time - safe_int.second;
                        if(departure_time <candidate_node.first->arrival_time){
                            continue;
                        }
                        arrival_time =  safe_int.second;
                        assert(arrival_time <= safe_int.second);
                    }

                    if (arrival_time < safe_int.first)
                    {
                        // departure_time += safe_int.first - arrival_time;
                        arrival_time = safe_int.first;
                        //it's ok, because we are slowing down
                    }

                    MDP::MSIRRT::Vertex::VertexCoordType collision_coord;

                    // std::cout<<departure_time<<" "<<arrival_time<<" "<<time_to_node<<" "<<safe_int.first<<" "<<safe_int.second<<" "<<candidate_node.first->arrival_time<<" "<<candidate_node.first->safe_interval.second<<std::endl;
                    assert(departure_time < arrival_time);
                    if (!is_collision_motion(start_coords, coord_rand, departure_time, arrival_time,collision_coord))
                    {
                        // std::cout <<departure_time<<" "<<candidate_node.first->arrival_time<<" "<<candidate_node.first->safe_interval.first<<" "<<candidate_node.first->safe_interval.second<<std::endl;
                        // assert(!is_collision_motion(start_coords, start_coords, candidate_node.first->arrival_time, departure_time));
                        // assert((safe_int.first <= arrival_time && safe_int.second >= arrival_time) || fabs(safe_int.second - arrival_time) < 0.001 || fabs(safe_int.first - arrival_time) < 0.001);
                        // assert((candidate_node.first->safe_interval.first <= departure_time && candidate_node.first->safe_interval.second >= departure_time));
                        // std::cout<<"add_vertex"<<std::endl;
                        // std::cout<<coord_rand.transpose()<<std::endl;
                        // std::cout<<safe_int.first<<" "<<safe_int.second<<std::endl;
                        // std::cout<<candidate_node.first->coords.transpose()<<std::endl;
                        // std::cout<<departure_time<<" "<<arrival_time<<std::endl;    
                        this->current_tree->add_vertex(coord_rand, safe_int, candidate_node.first, departure_time, arrival_time);
                        added_vertices.push_back(this->current_tree->array_of_vertices.back());
                        found_parent = true;
                        break;
                    }

            
                    // else if((departure_time > col_coord_safe_int_second || departure_time > col_coord_safe_int_first+max_dep_time_for_safe_int_check ) && departure_time < arrival_time)
                    else if(departure_time < arrival_time)
                       {
                            std::vector<double> vec_collision_manager(collision_coord.data(), collision_coord.data() + collision_coord.rows() * collision_coord.cols());
                            std::vector<std::pair<int, int>> collision_coord_safe_intervals = this->collision_manager.get_safe_intervals(vec_collision_manager);
                            if (collision_coord_safe_intervals.size()==0)
                            {
                                // safe_interval_ind++;
                                this->was_static_obstacle = true;
                                break;
                            }
                            // if does not overlap, break,
                            // if overlaps, set departure time according to lowest time at safe interval
                            bool no_overlaps_found = true;
                            for (const std::pair<int, int>& collision_safe_int:collision_coord_safe_intervals){
                                //we assume, that safe_intervals are sorted by time.
                                double time_from_candidate_to_collision =  (candidate_node.first->coords - collision_coord).norm() * (double)this->scene_task.fps / this->vmax;
                                double time_collision_to_rand =  (coord_rand - collision_coord).norm() * (double)this->scene_task.fps / this->vmax;
                                // assert(std::abs(time_from_candidate_to_collision+time_collision_to_rand  - time_to_node)<0.1 );
                                // projected to coll. coordinate parent low bound (departure time)
                                double pr_par_low = departure_time + time_from_candidate_to_collision;
                                // projected to coll. coordinate parent high bound
                                double pr_par_high = candidate_node.first->safe_interval.second + time_from_candidate_to_collision;

                                // projected to coll. coordinate random coord low bound
                                double pr_ran_low = safe_int.first - time_collision_to_rand;
                                // projected to coll. coordinate random coord high bound
                                double pr_ran_high = safe_int.second - time_collision_to_rand;

                                //ignore, if safe int is lower than dep time
                                if(collision_safe_int.second < pr_par_low){
                                    continue;
                                }
                                //stop iterating, if safe int is higher, than max dep time
                                if(collision_safe_int.first > std::min(pr_par_high,pr_ran_high)){
                                    break;
                                }

                                // get overlapping between parent and col

                                double overlap_par_col_low = std::max(pr_par_low,(double)collision_safe_int.first);
                                double overlap_par_col_high = std::min(pr_par_high,(double)collision_safe_int.second);
                                if (overlap_par_col_low > overlap_par_col_high){
                                    continue;
                                }

                                // get resulting overlapping with random coord safe_int

                                double result_overlap_low = std::max(overlap_par_col_low,pr_ran_low);
                                double result_overlap_high = std::min(overlap_par_col_high,pr_ran_high);
                                if (result_overlap_low > result_overlap_high){
                                    continue;
                                }

                                no_overlaps_found = false;

                                // std::cout<<departure_time<<" "<<candidate_node.first->safe_interval.second <<" "<<pr_par_low<<" "<<pr_par_high<<std::endl;
                                // std::cout<<safe_int.first<<" "<<safe_int.second <<" "<<pr_ran_low<<" "<<pr_ran_high<<std::endl;
                                // std::cout<<collision_safe_int.first<<" "<<collision_safe_int.second<<" "<<time_from_candidate_to_collision<<" "<<time_collision_to_rand<<std::endl;

                                // assert(result_overlap_low >=pr_par_low);
                                // std::cout<<result_overlap_low-time_from_candidate_to_collision << " "<< departure_time<<std::endl;
                                // assert(result_overlap_low-time_from_candidate_to_collision >=departure_time);
                                // std::cout<<result_overlap_low-time_from_candidate_to_collision <<" "<<result_overlap_high-time_from_candidate_to_collision<< " "<< std::min((double)candidate_node.first->safe_interval.second, (double)safe_int.second - time_to_node)<<std::endl;
                                // assert(result_overlap_low-time_from_candidate_to_collision <=std::min((double)candidate_node.first->safe_interval.second, (double)safe_int.second - time_to_node));
                                // if (!is_collision_motion(start_coords, collision_coord, departure_time, result_overlap_low))
                                // {
                                //     this->current_tree->add_vertex(collision_coord, safe_int, candidate_node.first, departure_time, result_overlap_low);
                                //     // added_vertices.push_back(this->current_tree->array_of_vertices.back());
                                // }
                                if ((result_overlap_low - time_from_candidate_to_collision - departure_time)>1)
                                {
                                    departure_time = result_overlap_low - time_from_candidate_to_collision-1 + (result_overlap_high - result_overlap_low)/3;
                                    // col_coord_safe_int_first = departure_time+1;
                                    // col_coord_safe_int_second = result_overlap_high- time_from_candidate_to_collision;
                                }

                                }
                            if (no_overlaps_found){
                                // safe_interval_ind++;
                                this->was_static_obstacle = true;
                                break;
                            }
                        }
                }
                if (found_parent)
                {
                    this->was_static_obstacle = false;
                    break;
                }
            }

            // if (!found_parent)
            // {
            //     this->orphan_tree->add_vertex(coord_rand, safe_int, nullptr, -1, safe_int.second + 1);
            // }
            // safe_interval_ind++;
        }
    }
    else if (this->current_tree == this->goal_tree)
    {
        // int max_dep_time_for_safe_int_check = 3;
        // int col_coord_safe_int_second = scene_task.frame_count+max_dep_time_for_safe_int_check;
        // int col_coord_safe_int_first = scene_task.frame_count;

        // s-1td::cout<<"set_parent goal_tree"<<std::endl;

        double time_to_start = (coord_rand - this->root_node->coords).norm() * (double)this->scene_task.fps / this->vmax;
        // for (const std::pair<MDP::MSIRRT::Vertex *, int> &node : nearest_nodes){
        //     if (!node.first){
        //         std::cout<<"null pointer in sort!"<<std::endl;
        //         assert(false);
        //     }
        // }
        std::sort(nearest_nodes.begin(), nearest_nodes.end(), [&coord_rand,this](const std::pair<MDP::MSIRRT::Vertex *, int> &a, std::pair<MDP::MSIRRT::Vertex *, int> &b)
                  { 
                    // Add null pointer checks to prevent segfault
                    if (!a.first || !b.first) {
                        std::cout<<"null pointer in sort!"<<std::endl;
                        
                        return a.first != nullptr; // null pointers go to the end
                    }
                    return a.first->arrival_time-((coord_rand - a.first->coords).norm() * (double)this->scene_task.fps / this->vmax) > b.first->arrival_time-((coord_rand - b.first->coords).norm() * (double)this->scene_task.fps / this->vmax); 
                  });
        // std::sort(nearest_nodes.begin(), nearest_nodes.end(), [](const std::pair<MDP::MSIRRT::Vertex *, int> &a, std::pair<MDP::MSIRRT::Vertex *, int> &b)
        //     { return a.first->arrival_time > b.first->arrival_time; });

        // for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes){
        //     std::cout<<candidate_node.first->arrival_time<<", ";
        // }
        // std::cout<<std::endl;
        for (std::pair<int, int> safe_int : safe_intervals_of_coord_rand) // For each interval
        {
            // std::cout<<"safe_int: "<<safe_int.first<<" "<<safe_int.second<<std::endl;
            // if we can't reach start from that interval - skip
            if ((safe_int.second - time_to_start) < 0)
            {
                // safe_interval_ind++;
                continue;
            }
            bool found_parent = false;

            // for each parent
            for (std::pair<MDP::MSIRRT::Vertex *, int> candidate_node : nearest_nodes)
            {
                // std::cout<<"candidate_node: "<<candidate_node.first->arrival_time<<" "<<candidate_node.first->safe_interval.first<<" "<<candidate_node.first->safe_interval.second<<std::endl;

                double time_to_node = (coord_rand - candidate_node.first->coords).norm() * (double)this->scene_task.fps / this->vmax;
                if (time_to_node<1){ // duplicate... this point was already added
                    found_parent = true;
                    break;
                }
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
                    // std::cout<<"departure_time: "<<departure_time<<std::endl;
                    // assert(!is_collision_motion(candidate_node.first->coords, candidate_node.first->coords, candidate_node.first->arrival_time, departure_time));
                    double arrival_time = departure_time - time_to_node; // we travel to the past
                    // fix rounding errors
                    if (arrival_time > safe_int.second)
                    {
                        // departure_time -= arrival_time - safe_int.second;
                        arrival_time = safe_int.second;
                    }
                    if (arrival_time < safe_int.first)
                    {
                        departure_time += safe_int.first - arrival_time;
                        if(departure_time < candidate_node.first->arrival_time){
                            continue;
                        }
                        arrival_time = departure_time - time_to_node;
                        assert(arrival_time >= safe_int.first);
                    }
                    MDP::MSIRRT::Vertex::VertexCoordType collision_coord;
                    assert(departure_time > arrival_time);
                    if (!is_collision_motion(coord_rand, start_coords, arrival_time, departure_time,collision_coord))
                    {
                        // assert(!is_collision_motion(start_coords, start_coords, departure_time, candidate_node.first->arrival_time));
                        // std::cout<<safe_int.first<<" "<<safe_int.second<<" "<<arrival_time<<std::endl;
                        // assert((safe_int.first <= arrival_time && safe_int.second >= arrival_time) || fabs(safe_int.second - arrival_time) < 0.001 || fabs(safe_int.first - arrival_time) < 0.001);
                        // std::cout<<"add_vertex"<<std::endl;
                        // std::cout<<coord_rand.transpose()<<std::endl;
                        // std::cout<<safe_int.first<<" "<<safe_int.second<<std::endl;
                        // std::cout<<candidate_node.first->coords.transpose()<<std::endl;
                        // std::cout<<departure_time<<" "<<arrival_time<<std::endl;    
                        this->current_tree->add_vertex(coord_rand, safe_int, candidate_node.first, departure_time, arrival_time);
                        added_vertices.push_back(this->current_tree->array_of_vertices.back());
                        found_parent = true;
                        break;
                    }

                    // else if((departure_time < col_coord_safe_int_first || departure_time < col_coord_safe_int_second-max_dep_time_for_safe_int_check ) && departure_time > arrival_time)
                    else if(departure_time > arrival_time)
                        {
                            
                            std::vector<std::pair<int, int>> collision_coord_safe_intervals = this->collision_manager.get_safe_intervals(std::vector<double>(collision_coord.data(), collision_coord.data() + collision_coord.rows() * collision_coord.cols()));
                            if (collision_coord_safe_intervals.size()==0)
                            {
                                this->was_static_obstacle = true;
                                break;
                            }
                            // if does not overlap, break,
                            // if overlaps, set departure time according to lowest time at safe interval
                            bool no_overlaps_found = true;

                            // reverse vector, so it sorted from latest to earliest safe intervals
                            std::reverse(collision_coord_safe_intervals.begin(), collision_coord_safe_intervals.end());

                            for (const std::pair<int, int>& collision_safe_int:collision_coord_safe_intervals){
                                //we assume, that safe_intervals are sorted by time from latest to earliest.
                                double time_from_candidate_to_collision =  (candidate_node.first->coords - collision_coord).norm() * (double)this->scene_task.fps / this->vmax;
                                double time_collision_to_rand =  (coord_rand - collision_coord).norm() * (double)this->scene_task.fps / this->vmax;

                                // projected to coll. coordinate parent low bound
                                double pr_par_low = candidate_node.first->safe_interval.first - time_from_candidate_to_collision;
                                // projected to coll. coordinate parent high bound (departure time)
                                double pr_par_high = departure_time - time_from_candidate_to_collision;

                                // projected to coll. coordinate random coord low bound
                                double pr_ran_low = safe_int.first + time_collision_to_rand;
                                // projected to coll. coordinate random coord high bound
                                double pr_ran_high = safe_int.second + time_collision_to_rand;

                                //ignore, if safe int is higher than dep time
                                if(collision_safe_int.first > pr_par_high){
                                    continue;
                                }
                                //stop iterating, if safe int is lower, than min dep time
                                if(collision_safe_int.second < std::min(pr_par_low,pr_ran_low)){
                                    break;
                                }

                                // get overlapping between parent and col

                                double overlap_par_col_low = std::max(pr_par_low,(double)collision_safe_int.first);
                                double overlap_par_col_high = std::min(pr_par_high,(double)collision_safe_int.second);
                                if (overlap_par_col_low > overlap_par_col_high){
                                    continue;
                                }

                                // get resulting overlapping with random coord safe_int

                                double result_overlap_low = std::max(overlap_par_col_low,pr_ran_low);
                                double result_overlap_high = std::min(overlap_par_col_high,pr_ran_high);
                                if (result_overlap_low > result_overlap_high){
                                    continue;
                                }

                                no_overlaps_found = false;
                                // if (!is_collision_motion(collision_coord, start_coords , result_overlap_high, departure_time))
                                // {
                                //     this->current_tree->add_vertex(collision_coord, safe_int, candidate_node.first, departure_time, result_overlap_high);
                                //     // added_vertices.push_back(this->current_tree->array_of_vertices.back());
                                // }
                                // assert(result_overlap_high <= pr_par_high);
                                // assert(result_overlap_high + time_from_candidate_to_collision <=departure_time);
                                // assert(result_overlap_high + time_from_candidate_to_collision >= std::max((double)candidate_node.first->safe_interval.first, (double)safe_int.first + time_to_node));
                                if (( departure_time-(result_overlap_high + time_from_candidate_to_collision))>1)
                                {
                                    departure_time = result_overlap_high + time_from_candidate_to_collision+1 - (result_overlap_high - result_overlap_low)/5;
                                    // col_coord_safe_int_first = result_overlap_low + time_from_candidate_to_collision;
                                    // col_coord_safe_int_second = departure_time -1;
                                }

                                }
                            if (no_overlaps_found){
                            //    safe_interval_ind++;
                            this->was_static_obstacle = true;   
                                break;
                            }
                        }
                }
                if (found_parent)
                {
                    break;
                }
            }

            // if (!found_parent)
            // {
            //     this->orphan_tree->add_vertex(coord_rand, safe_int, nullptr, -1, safe_int.second + 1);
            // }
            // safe_interval_ind++;
        }
    }

    return added_vertices;
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

bool MDP::MSIRRT::PlannerConnect::is_collision_motion(const MDP::MSIRRT::Vertex::VertexCoordType &start_coords, const MDP::MSIRRT::Vertex::VertexCoordType &end_coords, double &start_time, double &end_time, MDP::MSIRRT::Vertex::VertexCoordType &collision_coord)
{
    last_valid_coord = start_coords;
    if (start_time >= end_time)
    {
        std::cout << "start_time > end_time" << std::endl;

        return true;
    }
    MDP::MSIRRT::Vertex::VertexCoordType dir_vector = end_coords - start_coords;
    if ((dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time))) > (this->vmax+0.001)) // We need 0.001 to fix floating point comparison errors.
    {
        std::cout << "((dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time))) >= this->vmax)" << std::endl;
        std::cout <<std::fixed <<std::setprecision(15) <<(dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time)))<< " "<< dir_vector.norm() << " " << (double)this->scene_task.fps << " " << ((double)(end_time - start_time)) << " "<<std::fixed <<std::setprecision(15)<< this->vmax  << std::endl;
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
            collision_coord = temp_coords;
            return true;
        }
        last_valid_coord = temp_coords;
    }
    return false;
}

bool MDP::MSIRRT::PlannerConnect::is_collision_motion(const MDP::MSIRRT::Vertex::VertexCoordType &start_coords, const MDP::MSIRRT::Vertex::VertexCoordType &end_coords, double &start_time, double &end_time)
{

    if (start_time >= end_time)
    {
        // std::cout << "start_time > end_time" << std::endl;
        return true;
    }
    MDP::MSIRRT::Vertex::VertexCoordType dir_vector = end_coords - start_coords;
    if ((dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time))) > (this->vmax+0.001)) // We need 0.001 to fix floating point comparison errors.
    {
        std::cout << "((dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time))) >= this->vmax)" << std::endl;
        std::cout <<std::fixed <<std::setprecision(15) <<(dir_vector.norm() * ((double)this->scene_task.fps) / ((double)(end_time - start_time)))<< " "<< dir_vector.norm() << " " << (double)this->scene_task.fps << " " << ((double)(end_time - start_time)) << " "<<std::fixed <<std::setprecision(15)<< this->vmax  << std::endl;
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

bool MDP::MSIRRT::PlannerConnect::check_path(std::vector<MDP::MSIRRT::Vertex *> &path)
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
                this->start_tree->add_vertex(goal_tree_node->coords, goal_tree_node->safe_interval, start_tree_node, departure_time, arrival_time);
                start_tree_node = this->start_tree->array_of_vertices.back();
                found_dep_time = true;
                break;
            }
        }
        if (!found_dep_time)
        {
            this->start_tree->add_vertex(goal_tree_node->coords, goal_tree_node->safe_interval, start_tree_node, goal_tree_node_child->arrival_time, goal_tree_node_child->departure_from_parent_time);
            start_tree_node = this->start_tree->array_of_vertices.back();
        }
        goal_tree_node_child = goal_tree_node;
        goal_tree_node = goal_tree_node->parent;
    }
    this->finish_node = start_tree_node;
}


int MDP::MSIRRT::PlannerConnect::get_start_tree_vertices_count() const
{
    return this->start_tree->array_of_vertices.size();
}

int MDP::MSIRRT::PlannerConnect::get_goal_tree_vertices_count() const
{
    return this->goal_tree->array_of_vertices.size();
}