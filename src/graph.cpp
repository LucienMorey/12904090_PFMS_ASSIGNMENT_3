#include <set>
#include <queue>
#include "graph.h"
#include <iostream>

void Graph::addVertex(vertex v)
{
  // Insert a vertex with no edges
  edges_t edges_;
  weightedGraph_.insert({ v, edges_ });
}

bool Graph::hasVertex(vertex v)
{
  // Check if the vertex v, exists in the graph
  return weightedGraph_.count(v);
}

void Graph::addEdge(vertex u, vertex v, weight w)
{
  // Assumes that u & v have already been added to the graph
  // We need to record the same edge twice as this is an undirected graph
  weightedGraph_.at(u).insert({ v, w });  // Inserting an edge between u and v, with weight w
  weightedGraph_.at(v).insert({ u, w });  // Inserting an edge between v and u, with weight w
}

std::vector<Graph::vertex> Graph::getVertices(void)
{
  // Iterate through the weightedGraph_ and push back each vertex to the vertices vector
  std::vector<Graph::vertex> vertices;
  for (auto vertex : weightedGraph_)
  {
    vertices.push_back(vertex.first);
  }
  return vertices;
}

std::vector<Graph::vertex> Graph::bfs(vertex start)
{
  // Perform a breadth first search on the entire graph
  // temp queue to be searched
  std::queue<vertex> queue;
  // set of isited nodes to prevent double search
  std::set<vertex> visited;
  // temporary current node
  vertex current;
  // vector of adjoining nodes in breadth first search order
  std::vector<vertex> vertices;

  queue.push(start);

  // record initial vertex as visited
  visited.insert(start);
  while (!queue.empty())
  {
    // get current vertex
    current = queue.front();
    // add current node to output vector
    vertices.push_back(current);
    // remove current vertext from front of queue
    queue.pop();

    // iterate through adjoining vertices
    for (auto edges_iterator : weightedGraph_.at(current))
    {
      // only add vertices to the queue if they haven't been noted previously
      if (!visited.count(edges_iterator.first))
      {
        // add new vertex to the queue
        queue.push(edges_iterator.first);
        // add the new found vertex to the list of nodes visited
        visited.insert(edges_iterator.first);
      }
    }
  }

  return vertices;
}
