// application.cpp <Starter Code>
// <Yash Patel>
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
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <stack>
#include <string>
#include <vector>
#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;
const double INF = numeric_limits<double>::max();

class prioritize {
 public:
  bool operator()(const pair<long long, double>& p1,
                  const pair<long long, double>& p2) const {
    return p1.second > p2.second;
  }
};

vector<long long> Dijkstra(long long startV, graph<long long, double>& G,
                           map<long long, double>& distances,
                           map<long long, long long>& prev) {
  double alternativePath;
  vector<long long> visited;

  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisitedQueue;
  set<long long> visitedSet;

  vector<long long> vertex = G.getVertices();

  for (long long x : vertex) {
    unvisitedQueue.push(make_pair(x, INF));
    distances[x] = INF;
    prev[x] = -1;
  }

  distances[startV] = 0;
  unvisitedQueue.push(make_pair(startV, 0));

  long long currV;

  while (!unvisitedQueue.empty()) {
    currV = unvisitedQueue.top().first;
    double weight = unvisitedQueue.top().second;
    unvisitedQueue.pop();

    if (distances[currV] == INF) {
      break;
    } else if (weight == INF) {
      break;
    } else if (visitedSet.count(currV) > 0) {
      continue;
    } else {
      visited.push_back(currV);
      visitedSet.insert(currV);
    }

    set<long long> neighbors = G.neighbors(currV);

    for (auto& y : neighbors) {
      double edgeWeight;
      G.getWeight(currV, y, edgeWeight);
      alternativePath = distances[currV] + edgeWeight;

      if (alternativePath < distances[y]) {
        unvisitedQueue.push(make_pair(y, alternativePath));
        prev[y] = currV;
        distances[y] = alternativePath;
      }
    }
  }
  return visited;
}

void getPath(map<long long, long long> predecessors, long long startVert,
             long long destVert) {
  stack<long long> mystack;

  long long curr = destVert;
  while (curr != startVert) {
    mystack.push(curr);
    curr = predecessors[curr];
  }

  cout << startVert;
  while (!mystack.empty()) {
    cout << "->" << mystack.top();
    mystack.pop();
  }

  cout << endl;
}

bool search(string building, vector<BuildingInfo>& Buildings,
            BuildingInfo& info) {
  for (auto const& b : Buildings) {
    if (b.Abbrev == building) {
      info = b;
      return true;
    }
    if (b.Fullname.find(building) != string::npos) {
      info = b;
      return true;
    }
  }

  return false;
}

void creative() {

}
void application(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways, vector<BuildingInfo>& Buildings, graph<long long, double> G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);
  bool searching1 = true;
  bool searching2 = true;
  BuildingInfo info1, info2;
  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
    searching1 = search(person1Building, Buildings, info1);
    searching2 = search(person2Building, Buildings, info2);
    if (searching1 == false) {
      cout << "Person 1's building not found" << endl;
      cout << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      continue;
    } else if (searching2 == false) {
      cout << "Person 2's building not found" << endl;
      cout << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      continue;
    } else {
      cout << "Person 1's point:" << endl;
      cout << " " << info1.Fullname << endl;
      cout << " (" << info1.Coords.Lat << ", " << info1.Coords.Lon << ")" << endl;

      cout << "Person 2's point:" << endl;
      cout << " " << info2.Fullname << endl;
      cout << " (" << info2.Coords.Lat << ", " << info2.Coords.Lon << ")" << endl;

      Coordinates mid;

      mid = centerBetween2Points(info1.Coords.Lat, info1.Coords.Lon,
                                      info2.Coords.Lat, info2.Coords.Lon);

      double min = INF;
      double distance;
      BuildingInfo destBuilding;
      for (auto const& building : Buildings) {
        distance = distBetween2Points(mid.Lat, mid.Lon,
                                      building.Coords.Lat, building.Coords.Lon);

        if (distance < min) {
          destBuilding = building;
          min = distance;
        }
      }

      cout << "Destination Building:" << endl;
      cout << " " << destBuilding.Fullname << endl;
      cout << " (" << destBuilding.Coords.Lat << ", " << destBuilding.Coords.Lon << ")" << endl;

      long long firstStart, firstDest;
      double startMinDist, endMinDist;
      firstStart = Footways[0].Nodes[0];
      firstDest = Footways[0].Nodes[0];
      startMinDist = distBetween2Points(Nodes[Footways[0].Nodes[0]].Lat,
                                        Nodes[Footways[0].Nodes[0]].Lon,
                                        info1.Coords.Lat, info1.Coords.Lon);
      endMinDist = distBetween2Points(Nodes[Footways[0].Nodes[0]].Lat,
                                      Nodes[Footways[0].Nodes[0]].Lon,
                                      info2.Coords.Lat, info2.Coords.Lon);
      for (auto const& x : Footways) {
        for (auto const& y : x.Nodes) {
          if (distBetween2Points(Nodes[y].Lat, Nodes[y].Lon, info1.Coords.Lat,
                                 info1.Coords.Lon) < startMinDist) {
            firstStart = y;
            startMinDist = distBetween2Points(
                Nodes[y].Lat, Nodes[y].Lon, info1.Coords.Lat, info1.Coords.Lon);
          }

          if (distBetween2Points(Nodes[y].Lat, Nodes[y].Lon, info2.Coords.Lat,
                                 info2.Coords.Lon) < endMinDist) {
            firstDest = y;
            endMinDist = distBetween2Points(Nodes[y].Lat, Nodes[y].Lon,
                                            info2.Coords.Lat, info2.Coords.Lon);
          }
        }
      }
      cout << endl;
      cout << "Nearest P1 node:" << endl;
      cout << " " << firstStart << endl;
      cout << " (" << Nodes[firstStart].Lat << ","
           << " " << Nodes[firstStart].Lon << ")" << endl;
      cout << "Nearest P2 node:" << endl;
      cout << " " << firstDest << endl;
      cout << " (" << Nodes[firstDest].Lat << ","
           << " " << Nodes[firstDest].Lon << ")" << endl;

      double minimum = INF;
      long long dest = Footways[0].Nodes[0];
      for (auto const& x : Footways) {
        for (auto const& y : x.Nodes) {
          distance = distBetween2Points(Nodes[y].Lat, Nodes[y].Lon,
                                        destBuilding.Coords.Lat,
                                        destBuilding.Coords.Lon);
          if (distance < minimum) {
            dest = y;
            minimum = distance;
          }
        }
      }
      map<long long, double> distances1;
      map<long long, long long> predecessors;
      cout << "Nearest destination node:" << endl;
      cout << " " << dest << endl;
      cout << " (" << Nodes[dest].Lat << ","
           << " " << Nodes[dest].Lon << ")" << endl;
      cout << endl;
      vector<long long> ans = Dijkstra(firstStart, G, distances1, predecessors);
      if (distances1[firstDest] >= INF) {
        cout << "Sorry, destination unreachable." << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or "
                "#> ";
        getline(cin, person1Building);
        continue;
      }
      map<long long, double> distances2;

      if (distances1[dest] >= INF || distances2[dest] >= INF) {
        cout << "At least one person was unable to reach the destination "
                "building.";
        cout << " Finding next closest building..." << endl;
        set<string> unreachableBuildings;
        unreachableBuildings.insert(destBuilding.Fullname);
        BuildingInfo newDestBuilding;
        double newMin = INF;
        double newDist;
        for (auto const& building : Buildings) {
          if (unreachableBuildings.count(building.Fullname) != 0) {
            continue;
          }
          newDist = distBetween2Points(mid.Lat, mid.Lon,
                                 building.Coords.Lat, building.Coords.Lon);

          if (newDist < newMin) {
            newDestBuilding = building;
            newMin = newDist;
          }
        }
        cout << "New destination building:" << endl;
        cout << " " << newDestBuilding.Fullname << endl;
        cout << " (" << newDestBuilding.Coords.Lat << ", "
             << newDestBuilding.Coords.Lon << ")" << endl;
        newMin = INF;
        long long newDestNode = Footways[0].Nodes[0];
        for (auto const& a : Footways) {
          for (auto const& b : a.Nodes) {
            newDist = distBetween2Points(Nodes[b].Lat, Nodes[b].Lon,
                                             newDestBuilding.Coords.Lat,
                                             newDestBuilding.Coords.Lon);
            if (newDist < newMin) {
              newDestNode = b;
              newMin = newDist;
            }
          }
        }
        map<long long, double> newDist1;
        map<long long, double> newDist2;
        map<long long, long long> newPred;
        cout << "Nearest destination node:" << endl;
        cout << " " << newDestNode << endl;
        cout << " (" << Nodes[newDestNode].Lat << ","
             << " " << Nodes[newDestNode].Lon << ")" << endl;
        cout << endl;
        vector<long long> newAns =
            Dijkstra(firstStart, G, newDist1, newPred);

        cout << "Person 1's distance to dest: " << newDist1[newDestNode]
             << " miles" << endl;
        cout << "Path: ";
        getPath(newPred, firstStart, newDestNode);
        cout << endl;
        newAns = Dijkstra(firstDest, G, newDist2, newPred);
        cout << "Person 2's distance to dest: " << newDist2[newDestNode]
             << " miles" << endl;
        cout << "Path: ";
        getPath(newPred, firstDest, newDestNode);
        cout << endl;

        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
        continue;
      }

      cout << "Person 1's distance to dest: " << distances1[dest] << " miles"
           << endl;
      cout << "Path: ";
      getPath(predecessors, firstStart, dest);
      cout << endl;
      ans = Dijkstra(firstDest, G, distances2, predecessors);

      cout << "Person 2's distance to dest: " << distances2[dest] << " miles"
           << endl;
      cout << "Path: ";
      getPath(predecessors, firstDest, dest);

      //
      // another navigation?
      //

      //
      // TO DO: lookup buildings, find nearest start and dest nodes, find center
      // run Dijkstra's alg from each start, output distances and paths to
      // destination:
      //

      // cout << "Person 1's building not found" << endl;
      // cout << "Person 2's building not found" << endl;

      //
      // another navigation?
      //
      cout << endl;
      cout << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
    }
  }
}

int main() {
  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  graph<long long, double> G;
  for (auto x : Nodes) {
    G.addVertex(x.first);
  }

  for (auto const c : Footways) {
    for (int i = 0; i < ((int)c.Nodes.size() - 1); i++) {
      G.addEdge(
          c.Nodes[i], c.Nodes[i + 1],
          distBetween2Points(Nodes.at(c.Nodes[i]).Lat, Nodes.at(c.Nodes[i]).Lon,
                             Nodes.at(c.Nodes[i + 1]).Lat,
                             Nodes.at(c.Nodes[i + 1]).Lon));
      G.addEdge(
          c.Nodes[i + 1], c.Nodes[i],
          distBetween2Points(Nodes.at(c.Nodes[i]).Lat, Nodes.at(c.Nodes[i]).Lon,
                             Nodes.at(c.Nodes[i + 1]).Lat,
                             Nodes.at(c.Nodes[i + 1]).Lon));
    }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  //
  // Menu
  //
  string userInput;
  cout << "Enter \"a\" for the standard application or "
        << "\"c\" for the creative component application> ";
  getline(cin, userInput);
  if (userInput == "a") {
    // TO DO: add argument for the graph you make.
    application(Nodes, Footways, Buildings, G);
  } else if (userInput == "c") {
    // TO DO: add arguments
    creative();
  }
  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}