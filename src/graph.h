#ifndef GRAPH_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define GRAPH_H

#include <string>
#include <vector>
#include <map>

//A undirected, weighted graph
class Graph {
  typedef std::string vertex;
  typedef unsigned int weight;

  //The '_t' notation is used to denote a new type
  typedef std::map<vertex, weight> edges_t;
  typedef std::map<vertex, edges_t> graph_t;

public:
  //Adds the passed in vertex to the graph (with no edges).
  void addVertex(vertex);
  //Checks if the vertex exists.
  bool hasVertex(vertex);
  //Adds an edge between the two vertices with the given weight
  void addEdge(vertex, vertex, weight);
  //Returns a vector containing all the vertices.
  std::vector<vertex> getVertices(void);
  //Returns a vector containing the neighbours of a given vertex
  std::vector<vertex> getNeighbours(vertex);
  //Return a tree using breadth first search on the entire graph
  std::vector<vertex> bfs(vertex);

private:
    graph_t weightedGraph_; //The data structure storing our graph.
};

#endif // GRAPH_H
