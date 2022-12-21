#pragma once
#include<random>
#include <unordered_map>
#include <map>
#include <queue>
#include <algorithm>
#include "graph.h"
#include "ModelStructures.h"

class Pathfinder_PRM {
    
    // graph - граф всех вершин и путей
    // keygraph - граф только ключевых вершин
	Graph graph, keygraph;
    // параметры экрана
	point XY;
    // список преп€тствий
	std::vector<CircleObst> obstacles;
    // словарь подстановочных путей пежду ключевыми вершинами
    std::map<std::pair<int, int>, std::vector<int>> paths;
    // начальна€ вершина = 0
    // конечна€ вершина = 1
    // ѕотом следует targets.size()-2 дополнительных вершин
    std::vector<point> targets;
    // список позиций вершин
	std::map<int, point> nodeXY;
    // список дл€ финального пути
    std::vector<int> final_path;
    // количесвто вершин при генерации PRM
    int n;
    // радиус робота и переменна€ дл€ хранени€ результатов astar
	float rr, last_cost;
    // флаги дл€ проверки, возможно ли найти путь
    bool calculations_done;
    bool solvable;

    // –ешение задачи коммиво€жера (полный перебор)
    std::vector<int> traveling_salesman(int start_vertex, int end_vertex, Graph& graph);

    //—труктура дл€ хранени€ и подсчета пути и цены
    template<typename T, typename priority_t>
    struct PriorityQueue {

        //Ёлемент PQElement - {¬ершина, длина до вершины}
        typedef std::pair<priority_t, T> PQElement;

        std::priority_queue
            <
            PQElement,
            std::vector<PQElement>,
            std::greater<PQElement>
            > elements;

        //проверка, пуста ли структура
        inline bool empty() const { return elements.empty(); }

        //наполнение структуры
        inline void put(T item, priority_t priority) {
            elements.emplace(priority, item);
        }

        //получение вершины по наименьшему приоритету
        inline T get() {
            T best_item = elements.top().second;
            elements.pop();
            return best_item;
        }
    };

    //рассчет эвристики
    double heuristic(point node, point end);

    // ѕостроение графа с помощью Probabilistic Road Maps
    void PRM();

    //јлгоритм поиска кратчайшего пути а*
    std::vector<int> astar(int start_vertex, int end_vertex, Graph& graph);
   


public:

    Pathfinder_PRM(point size, std::vector<point> targs, std::vector<CircleObst> obsts, float _rr, int _n);

    // найти путь из стартовой вершины в конечную, посеща€ все дополнительные
    void find_path();

    // вовзвращает найденный путь
    std::vector<int> get_final();

    // вовзвращает позиции вершин
    std::map<int, point> get_vertex_pos();

    Graph get_graph();

    // возвращает, был ли найден путь
    bool solved();
    
};