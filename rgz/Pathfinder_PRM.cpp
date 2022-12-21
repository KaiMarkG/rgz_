#include "Pathfinder_PRM.h"

std::vector<int> Pathfinder_PRM::traveling_salesman(int start_vertex, int end_vertex, Graph& graph)
{
    double min_cost = std::numeric_limits<double>::max();
    std::vector<int> min_path;
    std::vector<int> perm;
    for (int i = 0; i < targets.size(); i++) if (i != start_vertex && i!= end_vertex) perm.push_back(i);
    do {
        double cur_cost = graph.edge_weight(start_vertex, perm[0]) + graph.edge_weight(perm.back(), end_vertex);
        for (int i = 0; i < perm.size() - 1; i++) {
            cur_cost += graph.edge_weight(perm[i], perm[i + 1]);
            if (cur_cost > min_cost) break;
        }
        if (cur_cost < min_cost) {
            min_cost = cur_cost;
            min_path = perm;
            min_path.push_back(1);
            min_path.insert(min_path.begin(), 0);
        }
    } while (std::next_permutation(perm.begin(), perm.end()));
    
    return min_path;
}

double Pathfinder_PRM::heuristic(point node, point end)
{
    return abs(node.x - end.x) + abs(node.y - end.y);
}

void Pathfinder_PRM::PRM()
{
    for (int i = 0; i < targets.size(); i++) {
        for (auto& obs : obstacles) {
            if (obs.coll_point(targets[i], rr)) {
                solvable = false;
                return;
            }
        }
        graph.add_vertex(i);
        for (auto& v : nodeXY) {
            bool addv = true;
            for (auto& obs : obstacles) {
                if (obs.coll_line(v.second, targets[i], rr)) {
                    addv = false;
                }
            }
            if (addv) graph.add_edge(v.first, i, dist(v.second, targets[i]));
        }
        nodeXY.insert(std::pair<int, point>(i, targets[i]));

    }
    std::mt19937 rng;
    for (int i = targets.size(); i < n + targets.size(); i++) {
        bool retry;
        do {
            retry = false;
            float x = rng() % (unsigned int)XY.x,
                y = rng() % (unsigned int)XY.y;

            point p = point(x, y);
            // îòìåíà, åñëè òî÷êà âíóòðè ïðåïÿòñòâèÿ
            for (auto& obs : obstacles) {
                if (obs.coll_point(x, y, rr)) {
                    retry = true;
                    break;
                }
            }
            if (!retry) {
                for (auto& v : nodeXY) {
                    bool addi = true;
                    for (auto& obs : obstacles) {
                        if (obs.coll_line(p, v.second, rr)) {
                            addi = false;
                            break;
                        }
                    }
                    if (addi) {
                        graph.add_edge(i, v.first, dist(v.second, p));
                        nodeXY.insert(std::pair<int, point>(i, p));
                    }
                }
                if (!graph.has_vertex(i)) retry = true;
            }
        } while (retry);
    }
}

std::vector<int> Pathfinder_PRM::astar(int start_vertex, int end_vertex, Graph& graph)
{

    std::unordered_map<int, int> came_from;
    std::unordered_map<int, double> cost_so_far;
    std::vector<int> path;

    PriorityQueue<int, double> frontier;
    frontier.put(start_vertex, 0);

    came_from[start_vertex] = start_vertex;
    cost_so_far[start_vertex] = 0;

    while (!frontier.empty()) {
        auto current = frontier.get(); //òåêóùàÿ âåðøèíà
        if (current == end_vertex) {
            break;
        }
        for (auto& next : graph.get_adjacent_vertices(current)) {
            double new_cost = cost_so_far[current] + graph.edge_weight(current, next);
            if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(nodeXY[next], targets[1]);
                frontier.put(next, priority);
                came_from[next] = current;

            }

        }

    }

    try {
        int nexti = came_from.at(end_vertex);
        for (int i = end_vertex; i != start_vertex; i = nexti) {
            path.insert(path.begin(), i);
            nexti = came_from.at(i);
        }
        path.insert(path.begin(), start_vertex);
        calculations_done = true;
    }
    catch (std::exception) {
        solvable = false;
        obstacles.clear();
        path.clear();
        graph.clear();
        nodeXY.clear();
    }
    last_cost = cost_so_far[end_vertex];
    return path;
}

Pathfinder_PRM::Pathfinder_PRM(point size, std::vector<point> targs, std::vector<CircleObst> obsts, float _rr, int _n) :
    XY{ size }, targets{ targs }, obstacles{ obsts }, rr{_rr}, solvable{true}, calculations_done{false}, n{_n}
{
}

void Pathfinder_PRM::find_path()
{
    PRM();
    for (int i = 0; i < targets.size(); i++) {
        for (int j = i + 1; j < targets.size(); j++) {
            std::pair<int, int> ij(i, j), ji(j,i);
            paths[ji] = paths[ij] = astar(i, j, graph);
            if (paths[ij].size() == 0) {
                solvable = false;
                calculations_done = false;
                return;
            }
            std::reverse(paths[ji].begin(), paths[ji].end());
            keygraph.add_edge(i, j, last_cost);
        }
    }
    if (targets.size() > 2) {
        auto min_path = traveling_salesman(0, 1, keygraph);
        for (int i = 0; i < min_path.size() - 1; i++) {
            std::vector<int>& cur = paths[std::pair<int, int>(min_path[i], min_path[i + 1])];
            final_path.insert(final_path.end(), cur.begin(), cur.end());
        }
        for (int i = 0; i < final_path.size() - 1; i++) {
            if (final_path[i] == final_path[i + 1]) {
                final_path.erase(final_path.begin() + i + 1);
                i--;
            }
        }
    }
    else {
        final_path = paths[std::pair<int, int>(0, 1)];
    }
    calculations_done = true;
}

std::vector<int> Pathfinder_PRM::get_final()
{
        return final_path;
}

std::map<int, point> Pathfinder_PRM::get_vertex_pos()
{
        return nodeXY; 
}

Graph Pathfinder_PRM::get_graph()
{
        return graph;
}

bool Pathfinder_PRM::solved()
{
    return calculations_done && solvable;
}
