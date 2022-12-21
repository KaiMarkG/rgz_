#include "graph.h"

#include <stdexcept>

using namespace std;

void Graph::add_vertex(int vertex) {
    if (!has_vertex(vertex)) {
        vertices[vertex] = std::map<int, double>();
    }
}

void Graph::add_edge(int start_vertex, int end_vertex, double dist) {
    add_vertex(start_vertex);
    add_vertex(end_vertex);
    vertices[start_vertex][end_vertex] = vertices[end_vertex][start_vertex] = dist;
}

std::vector<int> Graph::get_vertices() const {
    std::vector<int> result;
    for (const auto &p: vertices) {
        result.push_back(p.first);
    }
    return result;
}

std::vector<int> Graph::get_adjacent_vertices(int src_vertex) const {    
    const auto it = vertices.find(src_vertex);
    if (it == vertices.end()) {
        return std::vector<int> {};
    }
    vector<int> result;
    for (const auto &p: it->second) {
        result.push_back(p.first);
    }
    return result;
}

vector<edge> Graph::get_adjacent_edges(int src_vertex) const {
    const auto it = vertices.find(src_vertex);
    if (it == vertices.end()) {
        return vector<edge> {};
    }
    vector<edge> result;
    for (const auto &p: it->second) {
        result.push_back(make_pair(p.first, p.second));
    }
    return result;
}

bool Graph::has_vertex(int vertex) const {
    return (vertices.find(vertex) != vertices.end());
}

bool Graph::has_edge(int start_vertex, int end_vertex) const {    
    const auto it = vertices.find(start_vertex);
    if (it == vertices.end()) {
        return false;
    }
    return (it->second.find(end_vertex) != it->second.end());
}

double Graph::edge_weight(int start_vertex, int end_vertex) const {        
    const auto it_s = vertices.find(start_vertex);
    if (it_s == vertices.end()) {
        throw invalid_argument("Edge doesn't exist");
    }
    const auto it_e = it_s->second.find(end_vertex);
    if (it_e == it_s->second.end()) {
        throw invalid_argument("Edge doesn't exist");   
    }
    return it_e->second;
}

void Graph::remove_vertex(int vertex) {
    /// Remove adjacent edges.
    auto adjacent_vertices = get_adjacent_vertices(vertex);
    for (const auto &adj_v: adjacent_vertices) {
        remove_edge(adj_v, vertex);
    }
    /// Remove the vertex itself.
    vertices.erase(vertex);    
}

void Graph::remove_edge(int start_vertex, int end_vertex) {
    auto it_s = vertices.find(start_vertex);
    if (it_s != vertices.end()) {
        it_s->second.erase(end_vertex);
    }
    auto it_e = vertices.find(end_vertex);
    if (it_e != vertices.end()) {
        it_e->second.erase(start_vertex);
    }
}

void Graph::clear()
{
    vertices.clear();
}

std::ostream& operator<<(std::ostream& out, const Graph& g)
{
    out << g.vertices.size() << std::endl;
    for (auto& i : g.vertices) {
        out << i.first << ' ' << i.second.size() << std::endl;
        for (auto& j : i.second) {
            out << j.first << ' ' << j.second << ' ';
        }
    }
    return out;
}

std::istream& operator>>(std::istream& in, Graph& g)
{
    int n;
    in >> n;
    for (int i = 0; i < n; i++) {
        int vert, nv;
        in >> vert >> nv;
        g.add_vertex(vert);
        for (int j = 0; j < nv; j++) {
            int to;
            double cost;
            in >> to >> cost;
            g.add_edge(vert, to, cost);
        }
    }
    return in;
}
