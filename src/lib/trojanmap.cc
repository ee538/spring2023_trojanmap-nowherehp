#include "trojanmap.h"





// class ModifiedUnionFind {
// public:
//     ModifiedUnionFind(int size) {
//         for (int i = 0; i < size; ++i) {
//             parent.push_back(i);
//         }
//     }

//     int find(int x) {
//         if (parent[x] != x) {
//             parent[x] = find(parent[x]);
//         }
//         return parent[x];
//     }

//     void unite(int x, int y) {
//         int root_x = find(x);
//         int root_y = find(y);
//         if (root_x != root_y) {
//             parent[root_x] = root_y;
//         }
//     }

// private:
//     std::vector<int> parent;
// };

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

std::vector<std::string> TrojanMap::GetAllLocationIDs(){
  std::vector<std::string> all_location_ids;

  for (auto it = data.begin(); it != data.end(); it++) {
    all_location_ids.push_back(it->first);
  }

  return all_location_ids;

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
 std::transform(a.begin(),a.end(),a.begin(),::tolower);
 std::transform(b.begin(),b.end(),b.begin(),::tolower);
 if(a == b){
   return 0;
 }
 int m = a.length();
 int n = b.length();
 int distancePossible[m+1][n+1];
 for(int i = 0;i<= m; i++){
   distancePossible[i][0] = i;
 }
 for(int i = 0;i<=n;i++){
   distancePossible[0][i] = i;
 }
 for(int i =1; i<=m;i++){
   for(int j =1 ;j<=n; j++){
     if(a[i-1] == b [j-1]){
       distancePossible[i][j] = distancePossible[i-1][j-1];
     }else{
       distancePossible[i][j] =std::min(distancePossible[i-1][j],std::min(distancePossible[i-1][j-1],distancePossible[i][j-1]))+1;
     }
   }
 }


 return distancePossible[m][n];
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
 int distance = INT_MAX;
 for(auto it  = data.begin(); it != data.end();it++){
   if(it->second.name.empty()){
     continue;
   }
   int distance_between_two_place = CalculateEditDistance(name, it->second.name);
   if(distance_between_two_place < distance){
     distance = distance_between_two_place;
     tmp = it->second.name;
   }
 }
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
 std::set<std::string> CategoriesSet;
 for(auto it = data.begin();it != data.end();it++){
   if(it->second.attributes.empty()){
     continue;
   }else{
     CategoriesSet.insert(*it->second.attributes.begin());
   }
 }
 std::vector<std::string> result;
 for(auto x = CategoriesSet.begin(); x != CategoriesSet.end();x++){
   result.push_back(*x);
 }
 return result;
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
 std::transform(category.begin(),category.end(),category.begin(),::tolower);
 for(auto x = data.begin();x != data.end(); x++){
   if(x->second.attributes.empty()){
     continue;
   }else{
     std::string CategoryName = *x->second.attributes.begin();
     std::transform(CategoryName.begin(),CategoryName.end(),CategoryName.begin(),::tolower);
     if(CategoryName == category){
       res.push_back(x->first);
     }
   }
 }
 if(res.empty()){
   return{"-1"};
 }
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
 std::vector<std::string> result;
 for(auto x = data.begin();x != data.end(); x++){
   if(std::regex_match(x->second.name,location)){
     result.push_back(x->first);
   }
 }
 return result;
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
  records.first = INT_MAX;
  //set first point
  std::string first  = location_ids[0];
  //erase the first pointer
  location_ids.erase(location_ids.begin());
  //use helper
  int num = location_ids.size();
  std::unordered_map<std::string, std::vector<std::vector<std::string>>> memo;
  records.second = TravelingTrojan_Brute_force_helper(location_ids,num,memo);

  for(auto it : records.second){
    it.insert(it.begin(),first);
    it.push_back(first);
    double distance = CalculatePathLength(it);
    if(distance < records.first){
      records.first = distance;
      records.second.push_back(it);
    }
  }
  return records;
}


std::vector<std::vector<std::string>> TrojanMap::TravelingTrojan_Brute_force_helper(
  std::vector<std::string> &location_ids, int m,
  std::unordered_map<std::string, std::vector<std::vector<std::string>>> &memo
) {
  if (location_ids.size() < m) {
    return {};
  }
  if (m == 1) {
    std::vector<std::vector<std::string>> ret;
    for (auto &item : location_ids) {
      ret.push_back({item});
    }
    return ret;
  }
  std::string key = std::to_string(m) + ":" + vectorToString(location_ids);
  if (memo.find(key) != memo.end()) {
    return memo[key];
  }
  std::vector<std::vector<std::string>> ret;
  for (auto it = location_ids.begin(); it != location_ids.end(); it++) {
    std::vector<std::string> vtTemp;
    if (it != location_ids.begin()) {
      vtTemp.insert(vtTemp.end(), location_ids.begin(), it);
    }
    if (it + 1 != location_ids.end()) {
      vtTemp.insert(vtTemp.end(), it + 1, location_ids.end());
    }
    std::vector<std::vector<std::string>> vtArrage = TravelingTrojan_Brute_force_helper(vtTemp, m - 1, memo);
    for (auto &vtSet : vtArrage) {
      vtSet.push_back(*it);
      ret.push_back(vtSet);
    }
  }
  memo[key] = ret;
  return ret;
}



std::string TrojanMap::vectorToString(const std::vector<std::string> &vec) {
  std::ostringstream oss;
  for (const auto &elem : vec) {
    oss << elem << ",";
  }
  std::string result = oss.str();
  // Remove the trailing comma
  result = result.substr(0, result.size() - 1);
  return result;
}




// Please use backtracking to implement this function


std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::string> currentRoad;
  records.first = INT_MAX;
  currentRoad.push_back(location_ids[0]);
  TravelingTrojan_Backtracking_helper(currentRoad,location_ids,records);
  return records;
}




void TrojanMap::TravelingTrojan_Backtracking_helper(std::vector<std::string>& current,std::vector<std::string>& location_ids, std::pair<double, std::vector<std::vector<std::string>>>& records){
  if(current.size()  == location_ids.size()){
    current.push_back(location_ids.front());
    double distance = CalculatePathLength(current);
    if(distance < records.first){
      records.first = distance;
      records.second.push_back(current);
    }
    current.pop_back();
    return;
  }

  for(int i = 1;i<location_ids.size();i++){
    if(std::count(current.begin(),current.end(),location_ids[i])){
      continue;
    }
    current.push_back(location_ids[i]);
    if(CalculatePathLength(current) >= records.first){
      current.pop_back();
      continue;
    }
    TravelingTrojan_Backtracking_helper(current,location_ids,records);
    current.pop_back();
  }
}



// Hint: https://en.wikipedia.org/wiki/2-opt


std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  int size = location_ids.size() + 1;
  int improve = 0;
  location_ids.push_back(location_ids.front());
  records.second.push_back(location_ids);
  records.first = CalculatePathLength(location_ids);


  while (improve < 10) {
    bool improved = false;


    for (int i = 1; i < size - 2; ++i) {
      for (int k = i + 1; k < size - 1; ++k) {
        double old_dist = CalculateDistance(location_ids[i-1], location_ids[i]) + 
                          CalculateDistance(location_ids[k], location_ids[k+1]);
        double new_dist = CalculateDistance(location_ids[i-1], location_ids[k]) + 
                          CalculateDistance(location_ids[i], location_ids[k+1]);


        if (new_dist < old_dist) {
          std::reverse(location_ids.begin() + i, location_ids.begin() + k + 1);
          records.first -= old_dist;
          records.first += new_dist;
          records.second.push_back(location_ids);
          improved = true;
        }
      }
    }
    if (!improved) {
      improve++;
    } else {
      improve = 0;
    }
  }
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
  std::priority_queue<std::pair<double,std::string>> locationName;
  auto id = GetID(name);
  std::vector<std::string> allLocationWithSameAttribute = GetAllLocationsFromCategory(attributesName);

  for(auto it :allLocationWithSameAttribute){
    if(it == id){
      continue;
    }
    double distance = CalculateDistance(it,id);
    if(distance < r){
      locationName.push({distance,it});
      if(int(locationName.size()) > int(k)){
        locationName.pop();
      }
    }
  }

  while(!locationName.empty()){
    res.insert(res.begin(),locationName.top().second);
    locationName.pop();
  }
  return res;
}




/**
 * Shortest Path to Visit All Nodes: Given a list of locations, return the shortest
 * path which visit all the places and no need to go back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::vector<std::string> }      : the shortest path
 */
// std::vector<std::string> TrojanMap::TrojanPath(
//       std::vector<std::string> &location_names) {
//   std::vector<std::string> shortest_path;
//   double min_distance = std::numeric_limits<double>::max();

//   // Get all the permutations of the given locations
//   std::vector<std::vector<std::string>> permutations;
//   std::sort(location_names.begin(), location_names.end());
//   do {
//     permutations.push_back(location_names);
//   } while (std::next_permutation(location_names.begin(), location_names.end()));

//   // Iterate through each permutation and calculate its total distance using Dijkstra's algorithm
//   for (const auto &path : permutations) {
//     double total_distance = 0;
//     std::vector<std::string> full_path;
//     for (size_t i = 0; i < path.size() - 1; ++i) {
//       // Calculate the shortest path between two consecutive locations in the current permutation
//       std::vector<std::string> sub_path = CalculateShortestPath_Dijkstra(path[i], path[i + 1]);
//       // Add the distance of the shortest path to the total distance
//       total_distance += CalculatePathLength(sub_path);
//       // Append the sub_path to the full_path, except for the last element (to avoid duplication)
//       full_path.insert(full_path.end(), sub_path.begin(), sub_path.end() - 1);
//     }
//     // Add the last element of the last sub_path
//     full_path.push_back(path.back());
//     // If the total distance is less than the minimum distance found so far, update the result
//     if (total_distance < min_distance) {
//       min_distance = total_distance;
//       shortest_path = full_path;
//     }
//   }

//   return shortest_path;
// }


// std::vector<std::string> TrojanMap::TrojanPath(std::vector<std::string> &location_names) {
//   std::vector<std::string> shortest_path;
//   double min_distance = std::numeric_limits<double>::max();

//   // Sort the location names
//   std::sort(location_names.begin(), location_names.end());

//   // Iterate through each permutation and calculate its total distance using Dijkstra's algorithm
//   do {
//     double total_distance = 0;
//     for (size_t i = 0; i < location_names.size() - 1; ++i) {
//       // Calculate the shortest path between two consecutive locations in the current permutation
//       std::vector<std::string> sub_path = CalculateShortestPath_Dijkstra(location_names[i], location_names[i + 1]);
//       // Add the distance of the shortest path to the total distance
//       total_distance += CalculatePathLength(sub_path);
//     }

//     // If the total distance is less than the minimum distance found so far, update the result
//     if (total_distance < min_distance) {
//       min_distance = total_distance;
//       shortest_path.clear();

//       // Construct the full path
//       for (size_t i = 0; i < location_names.size() - 1; ++i) {
//         std::vector<std::string> sub_path = CalculateShortestPath_Dijkstra(location_names[i], location_names[i + 1]);
//         // Append the sub_path to the shortest_path, except for the last element (to avoid duplication)
//         shortest_path.insert(shortest_path.end(), sub_path.begin(), sub_path.end() - 1);
//       }
//       // Add the last element of the last sub_path
//       shortest_path.push_back(location_names.back());
//     }
//   } while (std::next_permutation(location_names.begin(), location_names.end()));

//   return shortest_path;
// }

// std::vector<std::string> TrojanMap::TrojanPath(std::vector<std::string> &location_names) {
//   std::vector<std::string> current_path = location_names;

//   // Calculate the initial path length
//   double current_path_length = 0;
//   for (size_t i = 0; i < location_names.size() - 1; ++i) {
//     std::vector<std::string> sub_path = CalculateShortestPath_Dijkstra(location_names[i], location_names[i + 1]);
//     current_path_length += CalculatePathLength(sub_path);
//   }

//   bool improvement = true;
//   while (improvement) {
//     improvement = false;

//     for (size_t i = 1; i < location_names.size() - 1; ++i) {
//       for (size_t j = i + 1; j < location_names.size(); ++j) {
//         // Swap nodes i and j to create a new candidate path
//         std::vector<std::string> candidate_path = current_path;
//         std::reverse(candidate_path.begin() + i, candidate_path.begin() + j + 1);

//         // Calculate the candidate path length
//         double candidate_path_length = 0;
//         for (size_t k = 0; k < location_names.size() - 1; ++k) {
//           std::vector<std::string> sub_path = CalculateShortestPath_Dijkstra(candidate_path[k], candidate_path[k + 1]);
//           candidate_path_length += CalculatePathLength(sub_path);
//         }

//         // If the candidate path is shorter, update the current path
//         if (candidate_path_length < current_path_length) {
//           current_path_length = candidate_path_length;
//           current_path = candidate_path;
//           improvement = true;
//         }
//       }
//     }
//   }

//   return current_path;
// }
// int TrojanMap::CountBits(int n) {
//   int count = 0;
//   while (n) {
//     count++;
//     n &= (n - 1);
//   }
//   return count;
// }



// std::vector<std::string> TrojanMap::TrojanPath(std::vector<std::string> &location_names) {
//   // Check if all locations are valid
//   for (const auto& loc : location_names) {
//     if (!FindLocationName(loc)) {
//       std::cout << loc << " is not a valid location." << std::endl;
//       return {};
//     }
//   }

//   // Initialize the memo table
//   std::vector<std::vector<double>> memo(
//     location_names.size(),        // number of rows = number of locations
//     std::vector<double>(
//         1 << location_names.size(),  // number of columns = 2^number of locations (all possible subsets)
//         -1));                          // initial value of each element is -1 (subproblem not solved yet)

//   // Calculate the shortest path length and path for all subsets of size 2
//   for (size_t i = 0; i < location_names.size(); i++) {
//     memo[i][(1 << i)] = 0;
//   }
//   for (size_t i = 0; i < location_names.size(); i++) {
//     for (size_t j = 0; j < location_names.size(); j++) {
//       if (i != j) {
//         std::vector<std::string> sub_path = CalculateShortestPath_Dijkstra(location_names[i], location_names[j]);
//         memo[i][(1 << i) | (1 << j)] = CalculatePathLength(sub_path);
//       }
//     }
//   }

//   // Calculate the shortest path length and path for all subsets of size k >= 3
//   for (size_t k = 3; k <= location_names.size(); k++) {
//     for (size_t subset = 0; subset < (1 << location_names.size()); subset++) {
//       if (CountBits(subset) == k) {
//         for (size_t i = 0; i < location_names.size(); i++) {
//           if (subset & (1 << i)) {
//             for (size_t j = 0; j < location_names.size(); j++) {
//               if (i != j && (subset & (1 << j))) {
//                 double prev_path_length = memo[j][subset ^ (1 << i) ^ (1 << j)];
//                 if (prev_path_length >= 0) {
//                   std::vector<std::string> sub_path = CalculateShortestPath_Dijkstra(location_names[i], location_names[j]);
//                   double new_path_length = prev_path_length + CalculatePathLength(sub_path);
//                   if (memo[i][subset] < 0 || new_path_length < memo[i][subset]) {
//                     memo[i][subset] = new_path_length;
//                   }
//                 }
//               }
//             }
//           }
//         }
//       }
//     }
//   }

//   // Reconstruct the shortest path
// std::vector<std::pair<std::string, std::vector<std::string>>> full_path;
// size_t subset = (1 << location_names.size()) - 1;
// size_t i = 0;
// while (true) {
//   std::string current_location = location_names[i];
//   std::vector<std::string> sub_path;

//   // Find the next location in the shortest path
//   for (size_t j = 0; j < location_names.size(); j++) {
//     if (i != j && (subset & (1 << j))) {
//       double prev_path_length = memo[j][subset ^ (1 << i)];
//       if (prev_path_length >= 0) {
//         std::vector<std::string> candidate_sub_path = CalculateShortestPath_Dijkstra(current_location, location_names[j]);
//         double new_path_length = prev_path_length + CalculatePathLength(candidate_sub_path);
//         if (memo[i][subset] == new_path_length) {
//           sub_path = candidate_sub_path;
//           i = j;
//           subset ^= (1 << i);
//           break;
//         }
//       }
//     }
//   }

//   // Add the current location and its subpath to the full path
//   full_path.push_back(std::make_pair(current_location, sub_path));

//   if (subset == 0) {
//     break;
//   }
// }

// // Reverse the order of the full path
// std::reverse(full_path.begin(), full_path.end());

// // Convert the full path to a list of location names
// std::vector<std::string> shortest_path;
// for (const auto& loc : full_path) {
//   shortest_path.push_back(loc.first);
//   if (!loc.second.empty()) {
//     shortest_path.insert(shortest_path.end(), loc.second.begin() + 1, loc.second.end());
//   }
// }

// // Return the shortest path
// return shortest_path;

// }
std::vector<std::string> TrojanMap::ClockwiseOrder(std::vector<std::string> location_names) {
  // Check if all locations are valid
  for (const auto& loc : location_names) {
    if (!FindLocationName(loc)) {
      std::cout << loc << " is not a valid location." << std::endl;
      return {};
    }
  }

  // Get the latitude and longitude of each location
  double lat1 = GetLat(GetID(location_names[0]));
  double lon1 = GetLon(GetID(location_names[0]));
  double lat2 = GetLat(GetID(location_names[1]));
  double lon2 = GetLon(GetID(location_names[1]));
  double lat3 = GetLat(GetID(location_names[2]));
  double lon3 = GetLon(GetID(location_names[2]));

  // Calculate the cross product of the vectors from location 1 to locations 2 and 3
  double cross_product = (lat2 - lat1) * (lon3 - lon1) - (lat3 - lat1) * (lon2 - lon1);

  // Determine the order of the locations based on the sign of the cross product
  if (cross_product > 0) {
    // Locations are in clockwise order
    return location_names;
  } else {
    // Locations are in counterclockwise order, swap the second and third locations
    std::vector<std::string> clockwise_order = {location_names[0], location_names[2], location_names[1]};
    return clockwise_order;
  }
}


std::vector<std::string> TrojanMap::TrojanPath(std::vector<std::string> &location_names) {
  // Check if all locations are valid
  for (const auto& loc : location_names) {
    if (!FindLocationName(loc)) {
      std::cout << loc << " is not a valid location." << std::endl;
      return {};
    }
  }
  
  std::vector<std::string> A = ClockwiseOrder(location_names);
  // Calculate the distances between the three locations
  std::vector<std::string> sub_path1 = CalculateShortestPath_Dijkstra(A[0], A[1]);
  double distance_A = CalculatePathLength(sub_path1);
  std::vector<std::string> sub_path2 = CalculateShortestPath_Dijkstra(A[1], A[2]);
  double distance_B = CalculatePathLength(sub_path2);
  std::vector<std::string> sub_path3 = CalculateShortestPath_Dijkstra(A[0], A[2]);
  double distance_C = CalculatePathLength(sub_path3);

  // Compare the distances and select the shortest path
  if (distance_A + distance_B < distance_A + distance_C && distance_A + distance_B < distance_B + distance_C) {
    std::vector<std::string> shortest_path = sub_path1;
    shortest_path.insert(shortest_path.end(), sub_path2.begin() + 1, sub_path2.end());
    // std::reverse(shortest_path.begin(), shortest_path.end());
    return shortest_path;
  } else if (distance_A + distance_C < distance_A + distance_B && distance_A + distance_C < distance_B + distance_C) {
    std::vector<std::string> shortest_path = sub_path3;
    shortest_path.insert(shortest_path.end(), sub_path1.begin() + 1, sub_path1.end());
    // std::reverse(shortest_path.begin(), shortest_path.end());
    return shortest_path;
  } else {
    std::vector<std::string> shortest_path = sub_path2;
    shortest_path.insert(shortest_path.end(), sub_path3.begin() + 1, sub_path3.end());
    // std::reverse(shortest_path.begin(), shortest_path.end());
    return shortest_path;
  }
}


/**
 * Given a vector of queries, find whether there is a path between the two locations with the constraint of the gas tank.
 *
 * @param  {std::vector<std::pair<double, std::vector<std::string>>>} Q : a list of queries
 * @return {std::vector<bool> }      : existence of the path
 */
std::vector<bool> TrojanMap::Queries(const std::vector<std::pair<double, std::vector<std::string>>>& q) {
    std::vector<bool> ans(q.size());

    for (size_t i = 0; i < q.size(); ++i) {
        double max_distance = q[i].first;
        std::string start_name = q[i].second[0];
        std::string end_name = q[i].second[1];

        
        if (!FindLocationName(start_name) || !FindLocationName(end_name)) {
          ans[i] = false;
          continue;
        }

        std::vector<std::string> all_location_ids = GetAllLocationIDs();
        int num_locations = all_location_ids.size();

        ModifiedUnionFind uf(num_locations);
        std::unordered_map<std::string, int> id_to_index;

        for (int j = 0; j < num_locations; ++j) {
            id_to_index[all_location_ids[j]] = j;
        }

        for (int j = 0; j < num_locations; ++j) {
            std::string id1 = all_location_ids[j];
            std::vector<std::string> neighbors = GetNeighborIDs(id1);
            for (const std::string& id2 : neighbors) {
                if (CalculateDistance(id1, id2) <= max_distance) {
                    uf.unite(id_to_index[id1], id_to_index[id2]);
                }
            }
        }

        std::string start_id = GetID(start_name);
        std::string end_id = GetID(end_name);

        ans[i] = uf.find(id_to_index[start_id]) == uf.find(id_to_index[end_id]);
    }

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
