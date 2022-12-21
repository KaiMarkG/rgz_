#pragma once

#include <map>
#include <vector>
#include <tuple>
#include <initializer_list>
#include <cmath>
#include <istream>
#include<ostream>

typedef std::pair<int, double> edge;

/**
 * Non-oriented weighted graph.
**/
class Graph {
public:
    Graph() {}

    /// Add single vertex to the graph.
    void add_vertex(int vertex);

    /// Add vertices (if necessary) and an edge between them to the graph.
    void add_edge(int start_vertex, int end_vertex, double dist = 1.);

    /// Return all vertices of the graph.
    std::vector<int> get_vertices() const;

    /// Return all adjacent by edges vertices for the specified vertex.
    std::vector<int> get_adjacent_vertices(int src_vertex) const;

    /// Get adjacent edges for the vertex as vector of (end vertex, edge weight).
    std::vector<edge> get_adjacent_edges(int src_vertex) const;

    /// Return true if the vertex exists in the graph, false otherwise.
    bool has_vertex(int vertex) const;

    /// Return true if vertices exist and have an edge between them, false otherwise.
    bool has_edge(int start_vertex, int end_vertex) const;

    /// Return weight of an edge between vertices (if there is any), throw an exception otherwise.
    double edge_weight(int start_vertex, int end_vertex) const;

    /// Remove the vertex and all its adjacent edges from the graph.
    void remove_vertex(int vertex);

    /// Remove the edge from the graph (but not the vertices).
    void remove_edge(int start_vertex, int end_vertex);

    void clear();

    friend std::ostream& operator<< (std::ostream& out, const Graph& g);
    friend std::istream& operator>> (std::istream& in, Graph& g);

private:
    std::map<int, std::map<int, double>> vertices;
};

std::ostream& operator<< (std::ostream& out, const Graph& g);
std::istream& operator>> (std::istream& in, Graph& g);
