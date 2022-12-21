#pragma once
#include<random>
#include <unordered_map>
#include <map>
#include <queue>
#include <algorithm>
#include "graph.h"
#include "ModelStructures.h"

class Pathfinder_PRM {

	Graph graph, keygraph;
	point XY;
	std::vector<CircleObst> obstacles;
    std::map<std::pair<int, int>, std::vector<int>> paths;

    std::vector<point> targets;
    std::map<int, point> nodeXY;
    std::vector<int> final_path;
    int n;
    float rr, last_cost;
    bool calculations_done;
    bool solvable;

    std::vector<int> traveling_salesman(int start_vertex, int end_vertex, Graph& graph);

    template<typename T, typename priority_t>
    struct PriorityQueue {
        typedef std::pair<priority_t, T> PQElement;

        std::priority_queue
            <
            PQElement,
            std::vector<PQElement>,
            std::greater<PQElement>
            > elements;

        inline bool empty() const { return elements.empty(); }

        inline void put(T item, priority_t priority) {
            elements.emplace(priority, item);
        }

        inline T get() {
            T best_item = elements.top().second;
            elements.pop();
            return best_item;
        }
    };

    double heuristic(point node, point end);

    void PRM();

    std::vector<int> astar(int start_vertex, int end_vertex, Graph& graph);
   


public:

    Pathfinder_PRM(point size, std::vector<point> targs, std::vector<CircleObst> obsts, float _rr, int _n);

    void find_path();

    std::vector<int> get_final();

    std::map<int, point> get_vertex_pos();

    Graph get_graph();

    bool solved();
    
};
