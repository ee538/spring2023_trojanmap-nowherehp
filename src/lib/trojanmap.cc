#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Students should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return
 * -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string &id) { 
  double Lat = data[id].lat;
  return Lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist,
 * return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string &id) {
  double Lon = data[id].lon;
  return Lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return
 * "NULL".
 *
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string &id) {
  if(id == "")
    return "";
  else {
      std::string name = data[id].name;
      return name;
  }
  
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return
 * an empty vector.
 *
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string &id) {
  // auto it = data.find(id);
  // if (it == data.end()) {
  //   return {}; // If the id is not in the data map, return an empty vector.
  // }

  // return it->second.neighbors; // Return the neighbors vector of the Node object associated with the given id.

  
  std::vector<std::string> res;
  if (data.count(id) == 0)
  {
    return res;
  }
  else
  {
    res = data[id].neighbors;
  }
  return res;
}


/**
 * GetID: Given a location name, return the id.
 * If the node does not exist, return an empty string.
 * The location name must be unique, which means there is only one node with the name.
 *
 * @param  {std::string} name          : location name
 * @return {std::string}               : id
 */
std::string TrojanMap::GetID(const std::string &name) {
  
  
  std::string ID = "";

  for (auto it = data.begin(); it != data.end(); it++)
  {
    if (it->second.name == name)
    {
      ID = it->second.id;
      break;
    }
  }
  return ID;
  
}




// phase1
/**
 * GetPosition: Given a location name, return the position. If id does not
 * exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  // Remove extra spaces around the dot operator
  std::transform(name.begin(), name.end(), name.begin(), ::tolower); 

  // Remove the duplicate declaration of `results`
  std::pair<double, double> results(-1, -1);

  if (name.empty()) return results;
  auto it = std::find_if(data.begin(), data.end(), [&name](const std::pair<std::string, Node>& datum) { 
    std::string lowercase_name(datum.second.name);
    std::transform(lowercase_name.begin(), lowercase_name.end(), lowercase_name.begin(), ::tolower);

    // Compare the transformed lowercase names instead of the original names
    return lowercase_name == name;
  });
  if (it != data.end()) results = {it->second.lat, it->second.lon};

  return results;
}


//phase2
/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * @param  {std::string} a          : first string
 * @param  {std::string} b          : second string
 * @return {int}                    : edit distance between two strings
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b) {     
  return 0;
}

//phase2
/**
 * FindClosestName: Given a location name, return the name with the smallest edit
 * distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : the closest name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp = ""; // Start with a dummy word
  return tmp;
}





// phase1
/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);

  for (const auto& node : data) {
    std::string node_name;
    std::transform(node.second.name.begin(), node.second.name.end(), std::back_inserter(node_name), ::tolower);
    
    if (node_name.find(name) == 0) {
      results.push_back(node.second.name);
    }
  }

  return results; 
}


//phase2

/**
 * GetAllCategories: Return all the possible unique location categories, i.e.
 * there should be no duplicates in the output.
 *
 * @return {std::vector<std::string>}  : all unique location categories
 */
std::vector<std::string> TrojanMap::GetAllCategories() {
  return {};
}

//phase2
/**
 * GetAllLocationsFromCategory: Return all the locations of the input category (i.e.
 * 'attributes' in data.csv). If there is no location of that category, return
 * (-1, -1). The function should be case-insensitive.
 *
 * @param  {std::string} category         : category name (attribute)
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetAllLocationsFromCategory(
    std::string category) {
  std::vector<std::string> res;
  return res;
}


//phase2
/**
 * GetLocationRegex: Given the regular expression of a location's name, your
 * program should first check whether the regular expression is valid, and if so
 * it returns all locations that match that regular expression.
 *
 * @param  {std::regex} location name      : the regular expression of location
 * names
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetLocationRegex(std::regex location) {
  return {};
}

/**
 * CalculateDistance: Get the distance between 2 nodes.
 * We have provided the code for you. Please do not need to change this function.
 * You can use this function to calculate the distance between 2 nodes.
 * The distance is in mile.
 * The distance is calculated using the Haversine formula.
 * https://en.wikipedia.org/wiki/Haversine_formula
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id,
                                    const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2), 2.0) + cos(a.lat * M_PI / 180.0) *
                                           cos(b.lat * M_PI / 180.0) *
                                           pow(sin(dlon / 2), 2.0);
  double c = 2 * asin(std::min(1.0, sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations
 * inside the vector.
 * We have provided the code for you. Please do not need to change this function.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0; i < int(path.size()) - 1; i++) {
    sum += CalculateDistance(path[i], path[i + 1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path
 * which is a list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
bool TrojanMap::FindLocationName(std::string location)
{
  if (location.size() == 0)
  {
    return false;
  }

  for (auto iter = data.begin(); iter != data.end(); iter++)
  {
    if (iter->second.name == location)
    {
      return true;
    }
  }

  return false;
}


std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  if (!(FindLocationName(location1_name) && FindLocationName(location2_name)))
  {
    return path;
  }


  std::unordered_map<std::string, double> mapDist;        // Store the min distance from root
  std::unordered_map<std::string, std::string> mapParent; // Store the Parent of each node
  std::unordered_map<std::string, bool> mapVisited;       // Store the visted marks for each node
  int visited_size = 0;
  std::string root_id = GetID(location1_name), target_id = GetID(location2_name);

  // Initialize the maps
  for (auto iter = data.begin(); iter != data.end(); iter++)
  {
    mapDist[iter->first] = __INT_MAX__;
    mapParent[iter->first] = "-1";
    mapVisited[iter->first] = false;
  }

  mapDist[root_id] = 0;
  mapParent[root_id] = "-1";

  while (visited_size < data.size())
  {
    std::pair<std::string, double> minVertex = {"-1", __INT_MAX__};

    if (visited_size == 0)
    {
      minVertex.first = root_id;
      minVertex.second = 0;
    }
    else
    {
      for (auto iter = mapVisited.begin(); iter != mapVisited.end(); iter++)
      {
        if (iter->second == true)
        {
          continue;
        }
        else
        {
          if (mapDist[iter->first] < minVertex.second)
          {
            minVertex.first = iter->first;
            minVertex.second = mapDist[iter->first];
          }
        }
      }
    }

    mapVisited[minVertex.first] = true;
    visited_size++;

    if (minVertex.first == target_id)
    {
      break;
    }

    std::vector<std::string> vecNeighbors = GetNeighborIDs(minVertex.first);

    for (auto iter = vecNeighbors.begin(); iter != vecNeighbors.end(); iter++)
    {
      if (mapDist[*iter] != __INT_MAX__ && mapVisited[*iter] == false)
      {
        if (mapDist[minVertex.first] + CalculateDistance(minVertex.first, *iter) <= mapDist[*iter])
        {
          mapDist[*iter] = mapDist[minVertex.first] + CalculateDistance(minVertex.first, *iter);
          mapParent[*iter] = minVertex.first;
        }
      }
      else if (mapDist[*iter] == __INT_MAX__ && mapVisited[*iter] == false)
      {
        mapDist[*iter] = mapDist[minVertex.first] + CalculateDistance(minVertex.first, *iter);
        mapParent[*iter] = minVertex.first;
      }
    }
  }

  std::string tempId = target_id;
  while(tempId != "-1"){
    path.push_back(tempId);
    tempId = mapParent[tempId];
  }

  std::reverse(path.begin(), path.end());

  return path;

}


/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest
 * path which is a list of id. Hint: Do the early termination when there is no
 * change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::unordered_map<std::string, double> mapDist;
  std::unordered_map<std::string, std::string> mapParent;
  

  std::string root_id = GetID(location1_name), target_id = GetID(location2_name);

  if(!(FindLocationName(location1_name) && FindLocationName(location2_name))){
    return path;
  }
  

  for(auto iter = data.begin(); iter!=data.end(); iter++){
    mapDist[iter->first] = __INT_MAX__;
    mapParent[iter->first] = "0";
  }

  mapDist[root_id] = 0;
  mapParent[root_id] = "-1";

  for(int i = 0; i<(data.size()-1); ++i){
    bool bIsChanged = false;

    for(auto iter = data.begin(); iter!=data.end(); iter++){
      std::vector<std::string> vecNeighbors = GetNeighborIDs(iter->first);
      for(auto child = vecNeighbors.begin(); child!=vecNeighbors.end(); child ++){
        if(mapDist[iter->first] == __INT_MAX__)  continue;
        double edge = CalculateDistance(iter->first, *child);
        if(mapDist[iter->first] + edge < mapDist[*child]){
          mapDist[*child] = mapDist[iter->first] + edge;
          mapParent[*child] = iter->first;
          bIsChanged = true;
          //std::cout<<"Modified!"<<std::endl;
        }
      }
    }

    if(!bIsChanged) break;
  }

  std::string tempId = target_id;
  while(tempId != "-1" && path.size() < data.size()){
    path.push_back(tempId);
    tempId = mapParent[tempId];
  }
  std::reverse(path.begin(), path.end());

  return path;
}


//phase 3
/**
 * Traveling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path, 
 *                                                                      for example: {10.3, {{0, 1, 2, 3, 4, 0}, {0, 1, 2, 3, 4, 0}, {0, 4, 3, 2, 1, 0}}},
 *                                                                      where 10.3 is the total distance, 
 *                                                                      and the first vector is the path from 0 and travse all the nodes and back to 0,
 *                                                                      and the second vector is the path shorter than the first one,
 *                                                                      and the last vector is the shortest path.
 */
// Please use brute force to implement this function, ie. find all the permutations.
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

// Please use backtracking to implement this function
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

// Hint: https://en.wikipedia.org/wiki/2-opt
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

// This is optional
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_3opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_locations.csv"
 *   File content:
 *    Name
 *    Ralphs
 *    KFC
 *    Chick-fil-A
 *   Output: ['Ralphs', 'KFC', 'Chick-fil-A']
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(
    std::string locations_filename) {
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, word)) {
    location_names_from_csv.push_back(word);
  }
  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_dependencies.csv"
 *   File content:
 *     Source,Destination
 *     Ralphs,Chick-fil-A
 *     Ralphs,KFC
 *     Chick-fil-A,KFC
 *   Output: [['Ralphs', 'Chick-fil-A'], ['Ralphs', 'KFC'], ['Chick-fil-A', 'KFC']]
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(
    std::string dependencies_filename) {
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    std::vector<std::string> dependency;
    while (getline(s, word, ',')) {
      dependency.push_back(word);
    }
    dependencies_from_csv.push_back(dependency);
  }
  fin.close();
  return dependencies_from_csv;
}



//phase2
/**
 * DeliveringTrojan: Given a vector of location names, it should return a
 * sorting of nodes that satisfies the given dependencies. If there is no way to
 * do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  std::unordered_map<std::string, std::vector<std::string>> graph; //adjacency list
  std::map<std::string, bool> visited;


  for (const auto& location : locations) {
    graph[location] = {};
    visited[location] = false;
  }
  for(auto dependency: dependencies) {
    graph[dependency[0]].push_back(dependency[1]);
  }


  for(auto location: locations)
  {
    if(!visited[location])
      DFS_Helper(location, visited, graph, result);
  }
  std::unique(result.begin(), result.end());
  std::reverse(result.begin(), result.end());
  return result;                                                     
}  

void TrojanMap::DFS_Helper(std::string location, std::map<std::string, bool>& visited, 
                          std::unordered_map<std::string, std::vector<std::string> >&graph, std::vector<std::string>& result)
{
  visited[location] = true; 
  for(const std::string neighbor: graph[location]) 
  {
    if(!visited[neighbor])
    {
      DFS_Helper(neighbor, visited, graph, result);
    }
  }
  result.push_back(location); 
}                                    
                       
//phase2
/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  auto it = data.find(id);
  if (it == data.end()) {
    return false; // If the id is not in the data map, return false.
  }

  Node node = it->second; // Get the Node object associated with the given id safely
  double lon_left = square[0];
  double lon_right = square[1];
  double lat_upper = square[2];
  double lat_lower = square[3];

  // Check if the node's longitude and latitude are within the bounds.
  if (node.lon >= lon_left && node.lon <= lon_right &&
      node.lat >= lat_lower && node.lat <= lat_upper) {
    return true;
  } else {
    return false;
  }

   
 }


//phase2
/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location
 * ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square
 * area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the
 * square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  if (square.size() != 4)
  {
    return subgraph;
  }

  if (square[0] > square[1] || square[2] < square[3])
  {
    return subgraph;
  }
  for (const auto &it : data) {
    std::string id = it.first;
    if (inSquare(id, square)) {
      subgraph.push_back(id);
    }
  }
  return subgraph;
}


//phase2
/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true
 * if there is a cycle path inside the square, false otherwise.
 *
 * @param {std::vector<std::string>} subgraph: list of location ids in the
 * square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
   std::unordered_map<std::string, bool> visited;

  for(auto it:subgraph){
    visited[it] = false;
  }

  for(auto it:subgraph){
    if(visited[it] == false){
      if (dfs(it,visited,"")){
        return true;
      }
    }
  }
  return false;
}


bool TrojanMap::dfs(std::string current_id,std::unordered_map<std::string, bool> &visited, std::string parent_id){
  visited[current_id] = true;
  for(auto n:data[current_id].neighbors){
    if(visited.find(n) != visited.end()){ //to check if the neighbor is in the area
      if(visited[n] == false){
        if(dfs(n,visited,current_id)){
          return true;
        }
      }else if((n!=parent_id) && (visited[n]== true)){
          return true;
      }
    }
  }
  return false;
}



/**
 * FindNearby: Given a class name C, a location name L and a number r,
 * find all locations in class C on the map near L with the range of r and
 * return a vector of string ids
 *
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {double} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  return res;
}

/**
 * Shortest Path to Visit All Nodes: Given a list of locations, return the shortest
 * path which visit all the places and no need to go back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::vector<std::string> }      : the shortest path
 */
std::vector<std::string> TrojanMap::TrojanPath(
      std::vector<std::string> &location_names) {
    std::vector<std::string> res;
    return res;
}

/**
 * Given a vector of queries, find whether there is a path between the two locations with the constraint of the gas tank.
 *
 * @param  {std::vector<std::pair<double, std::vector<std::string>>>} Q : a list of queries
 * @return {std::vector<bool> }      : existence of the path
 */
std::vector<bool> TrojanMap::Queries(const std::vector<std::pair<double, std::vector<std::string>>>& q) {
    std::vector<bool> ans(q.size());
    return ans;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * We have provided the code for you. Please do not need to change this function.
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0])) n.attributes.insert(word);
        if (isdigit(word[0])) n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
