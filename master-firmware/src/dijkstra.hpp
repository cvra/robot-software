#pragma once

#include <iterator>
#include <climits>

namespace pathfinding {

/** Class storing a node in the graph for Dijkstra
 *
 * @parameter Data an application-specific piece of data, such as an arm configuration.
 * @parameter N The maximum number of neighbors (connected nodes) for each node.
 */
template <typename Data, int N = 10>
class Node {
protected:
    Node<Data>* edges[N];
    int edge_count;

    /* Used for dijkstra */
    bool visited;
    int distance;
    Node<Data>* parent;

public:
    template <typename T>
    friend int dijkstra(T* nodes, int node_count, T& start, T& end);

    Node(Data d)
        : edge_count(0)
        , data{d}
    {
    }

    void connect(Node<Data>& n)
    {
        edges[edge_count] = &n;
        edge_count += 1;
    }

    Node<Data>* path_next; ///< Pointer to the next node in the calculated path
    Data data;
};

/** Computes the shortest path in the given graph.
 *
 * The path will be stored in the nodes themselves as a linked list. To traverse the path, follow the path_next pointer.
 * Ex:
 *
 * for (auto *p = &start; p->path_next != nullptr; p = p->path_next) {
 *   move_to(p.data);
 * }
 *
 * @returns The path length.
 */
template <typename Node>
int dijkstra(Node* nodes, int node_count, Node& start, Node& end)
{
    for (auto i = 0; i < node_count; i++) {
        nodes[i].visited = false;
        nodes[i].distance = INT_MAX;
        nodes[i].parent = nullptr;
    }

    start.distance = 0;

    auto v = &start;

    while (v->visited == false) {
        v->visited = true;

        for (auto i = 0; i < v->edge_count; i++) {
            if (v->edges[i]->distance > v->distance + 1) {
                v->edges[i]->distance = v->distance + 1;
                v->edges[i]->parent = v;
            }
        }

        int min_distance = INT_MAX;

        for (auto i = 0; i < node_count; i++) {
            if (!nodes[i].visited && min_distance > nodes[i].distance) {
                min_distance = nodes[i].distance;
                v = &nodes[i];
            }
        }
    }

    int len = 0;
    for (auto* p = &end; p != &start; p = p->parent) {
        p->parent->path_next = p;
        len++;
    }

    return len;
}
} // namespace pathfinding

