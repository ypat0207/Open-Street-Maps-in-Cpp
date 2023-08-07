// graph.h
// <Yash Patel>
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2022
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

using namespace std;

template <typename VertexT, typename WeightT>
class graph {
 private:
  map<VertexT, map<VertexT, WeightT>> adjList;
  bool _LookupVertex(VertexT v) const {
    if (adjList.count(v) == 1) {
      return true;
    } else {
      return false;
    }
  }

 public:
  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
	  return adjList.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int edgesCount = 0;

    for (auto i : adjList) {
      edgesCount += i.second.size();
    }
    return edgesCount;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    if (_LookupVertex(v)) {
      return false;
    }

    map<VertexT, WeightT> newAdjList;
    adjList[v] = newAdjList;
    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    if (!_LookupVertex(from)) {
      return false;
    }

    if (!_LookupVertex(to)) {
      return false;
    }

    adjList[from][to] = weight;
    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT &weight) const {

    if (!_LookupVertex(from)) {
      return false;
    }

    if (!_LookupVertex(to)) {
      return false;
    }

    if (adjList.at(from).count(to) == 0) {
      return false;
    }

    weight = adjList.at(from).at(to);
    return true;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    if (!_LookupVertex(v)) {
      return S;
    }
    for (auto c : adjList.at(v)) {
      S.insert(c.first);
    }
    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> vertices;
    for (auto x : adjList) {
      vertices.push_back(x.first);
    }
    return vertices;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream &output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    int i = 0;
    for (auto x : adjList) {
      output << " " << i << ". " << x.first << endl;
      i++;
    }

    output << endl;
    output << "**Edges:" << endl;

    for (auto &c : this->adjList) {
      output << " row " << c.first << ": ";
      for (auto &d : adjList) {
        bool found = false;

        for (auto const &b : c.second) {
          if (d.first == b.first) {
            output << "(T," << b.second << ") ";
            found = true;
          }
        }
        if (found == false) {
          output << "F ";
        }
      }
      output << endl;
    }
    output << "**************************************************" << endl;
  }
};