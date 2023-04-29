#ifndef TROJAN_MAP_H
#define TROJAN_MAP_H

#include <math.h>

#include <algorithm>
#include <cfloat>
#include <climits>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <regex>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>

class ModifiedUnionFind {
public:
    ModifiedUnionFind(int size) {
        for (int i = 0; i < size; ++i) {
            parent.push_back(i);
        }
    }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    void unite(int x, int y) {
        int root_x = find(x);
        int root_y = find(y);
        if (root_x != root_y) {
            parent[root_x] = root_y;
        }
    }

private:
    std::vector<int> parent;
};





// A Node is the location of one point in the map.
class Node {
 public:
  Node(){};
  Node(const Node &n) {
    id = n.id;
    lat = n.lat;
    lon = n.lon;
    name = n.name;
    neighbors = n.neighbors;
    attributes = n.attributes;
  };
  std::string id;    // A unique id assigned to each point.
  double lat;        // Latitude
  double lon;        // Longitude
  std::string name;  // Name of the location. E.g. "Bank of America".
  std::vector<std::string>
      neighbors;  // List of the ids of all neighbor points.
  std::unordered_set<std::string>
      attributes;  // List of the attributes of the location.
};

class TrojanMap {
 public:
  // Constructor
  TrojanMap() { CreateGraphFromCSVFile(); };

  // A map of ids to Nodes.
  std::unordered_map<std::string, Node> data;

  //-----------------------------------------------------
  // Read in the data
  void CreateGraphFromCSVFile();

  //-----------------------------------------------------
  // TODO: Implement these functions and create unit tests for them:
  // Get the Latitude of a Node given its id.
  double GetLat(const std::string &id);

  // Get the Longitude of a Node given its id.
  double GetLon(const std::string &id);

  // Get the name of a Node given its id.
  std::string GetName(const std::string &id);

  // Get the id given its name.
  std::string GetID(const std::string &name);


  // Get the neighbor ids of a Node.
  std::vector<std::string> GetNeighborIDs(const std::string &id);

  // Returns a vector of names given a partial name.
  std::vector<std::string> Autocomplete(std::string name);

  // GetAllCategories: Return all the possible unique location categories, i.e.
  //  there should be no duplicates in the output.
  std::vector<std::string> GetAllCategories();

  std::vector<std::string> GetAllLocationsFromCategory(std::string category);

  std::vector<std::string> GetLocationRegex(std::regex location);

  // Returns lat and lon of the given the name.
  std::pair<double, double> GetPosition(std::string name);

  // Calculate location names' edit distance
  int CalculateEditDistance(std::string, std::string);

  // Find the closest name
  std::string FindClosestName(std::string name);

  // Get the distance between 2 nodes.
  double CalculateDistance(const std::string &a, const std::string &b);

  // Calculates the total path length for the locations inside the vector.
  double CalculatePathLength(const std::vector<std::string> &path);

  // Given the name of two locations, it should return the **ids** of the nodes
  // on the shortest path.
  std::vector<std::string> CalculateShortestPath_Dijkstra(
      std::string location1_name, std::string location2_name);
  std::vector<std::string> CalculateShortestPath_Bellman_Ford(
      std::string location1_name, std::string location2_name);

  // Given CSV filename, it read and parse locations data from CSV file,
  // and return locations vector for topological sort problem.
  std::vector<std::string> ReadLocationsFromCSVFile(
      std::string locations_filename);

  // Given CSV filenames, it read and parse dependencise data from CSV file,
  // and return dependencies vector for topological sort problem.
  std::vector<std::vector<std::string>> ReadDependenciesFromCSVFile(
      std::string dependencies_filename);

  // Given a vector of location names, it should return a sorting of nodes
  // that satisfies the given dependencies.
  std::vector<std::string> DeliveringTrojan(
      std::vector<std::string> &location_names,
      std::vector<std::vector<std::string>> &dependencies);
  
 

  // Given a vector of location ids, it should reorder them such that the path
  // that covers all these points has the minimum length.
  // The return value is a pair where the first member is the total_path,
  // and the second member is the reordered vector of points.
  // (Notice that we don't find the optimal answer. You can return an estimated
  // path.)
  std::pair<double, std::vector<std::vector<std::string>>>
  TravelingTrojan_Brute_force(std::vector<std::string> location_ids);

  std::pair<double, std::vector<std::vector<std::string>>>
  TravelingTrojan_Backtracking(std::vector<std::string> location_ids);

  std::pair<double, std::vector<std::vector<std::string>>> TravelingTrojan_2opt(
      std::vector<std::string> location_ids);

  std::pair<double, std::vector<std::vector<std::string>>> TravelingTrojan_3opt(
      std::vector<std::string> location_ids);

  std::vector<std::string> TrojanPath(std::vector<std::string> &location_names);
    
  // Check whether the id is in square or not
  bool inSquare(std::string id, std::vector<double> &square);

  // Get the subgraph based on the input
  std::vector<std::string> GetSubgraph(std::vector<double> &square);

  // Given a subgraph specified by a square-shape area, determine whether there
  // is a cycle or not in this subgraph.
  bool CycleDetection(std::vector<std::string> &subgraph,
                      std::vector<double> &square);

  // Given a location id and k, find the k closest points on the map
  std::vector<std::string> FindNearby(std::string, std::string, double, int);
  
  // Takes in a vector of queries. Each query consists of a pair: <tank_capacity, [source, destination]>.
  // Returns the result of each query in a vector.
  std::vector<bool> Queries(const std::vector<std::pair<double, std::vector<std::string>>> &q);

  //----------------------------------------------------- User-defined functions



  std::vector<std::vector<std::string>>TravelingTrojan_Brute_force_helper( std::vector<std::string> &location_ids, int m, std::unordered_map<std::string, std::vector<std::vector<std::string>>> &memo);

  std::string vectorToString(const std::vector<std::string> &vec);

  void TravelingTrojan_Backtracking_helper(std::vector<std::string>& current,std::vector<std::string>& location_ids, std::pair<double, std::vector<std::vector<std::string>>>& records);

   void DFS_Helper(std::string location, std::map<std::string, bool>& visited, 
                          std::unordered_map<std::string, std::vector<std::string> >&graph, 
                          std::vector<std::string>& result);

   
  //find the locaion exsit or not
  bool FindLocationName(std::string location);

  std::vector<std::string> GetAllLocationIDs();


  //dfs
  bool dfs(std::string current_id,std::unordered_map<std::string, bool> &visited, std::string parent_id);
//   int CountBits(int n);
  std::vector<std::string> ClockwiseOrder(std::vector<std::string> location_names);
};

#endif
