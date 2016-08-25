#include "HexGrid.h"
#include <cmath>
#include <algorithm>
using std::abs;
using std::max;


HexGrid::HexGrid(){
  rows = 10;
  cols = 10;
  size = Vector2(10,10);
  origin = Vector2(0,0);
  grid_layout = "flat";
  setUpConstants();
}

void HexGrid::setUpConstants(){

  //hex directions
  //I'm such a noob I have no clue how to simply create the array. =(
  hex_directions.push_back(Vector3(1,0,-1));
  hex_directions.push_back(Vector3(1,-1,0));
  hex_directions.push_back(Vector3(0,-1,1));
  hex_directions.push_back(Vector3(-1,0,1));
  hex_directions.push_back(Vector3(-1,1,0));
  hex_directions.push_back(Vector3(0,1,-1));

  layout_pointy["f0"] = sqrt(3.0);
  layout_pointy["f1"] = sqrt(3.0) / 2.0;
  layout_pointy["f2"] = 0.0;
  layout_pointy["f3"] = 3.0 / 2.0;
  layout_pointy["b0"] = sqrt(3.0) / 3.0;
  layout_pointy["b1"] = -1.0 / 3.0;
  layout_pointy["b2"] = 0.0;
  layout_pointy["b3"] = 2.0 / 3.0;
  layout_pointy["start_angle"] = 0.5;

  layout_flat["f0"] = 3.0 / 2.0;
  layout_flat["f1"] = 0.0;
  layout_flat["f2"] = sqrt(3.0) / 2.0;
  layout_flat["f3"] = sqrt(3.0);
  layout_flat["b0"] = 2.0 / 3.0;
  layout_flat["b1"] = 0.0;
  layout_flat["b2"] = -1.0 / 3.0;
  layout_flat["b3"] = sqrt(3.0) / 3.0;
  layout_flat["start_angle"] = 0.0;

  setupLayout();

}

void HexGrid::setupLayout(){
  if (grid_layout == "flat"){
    orient["f0"] = layout_flat["f0"];
    orient["f1"] = layout_flat["f1"];
    orient["f2"] = layout_flat["f2"];
    orient["f3"] = layout_flat["f3"];
    orient["b0"] = layout_flat["b0"];
    orient["b1"] = layout_flat["b1"];
    orient["b2"] = layout_flat["b2"];
    orient["b3"] = layout_flat["b3"];
    orient["start_angle"] = layout_flat["start_angle"];
  } else {
    orient["f0"] = layout_pointy["f0"];
    orient["f1"] = layout_pointy["f1"];
    orient["f2"] = layout_pointy["f2"];
    orient["f3"] = layout_pointy["f3"];
    orient["b0"] = layout_pointy["b0"];
    orient["b1"] = layout_pointy["b1"];
    orient["b2"] = layout_pointy["b2"];
    orient["b3"] = layout_pointy["b3"];
    orient["start_angle"] = layout_pointy["start_angle"];
  }
  createMap();
}

//Random things not really related to the calculation of Hex
void HexGrid::set_rows(int r){
  rows = r;
}

void HexGrid::set_cols(int c){
  cols = c;
}

void HexGrid::set_rows_and_cols(int r, int c){
  set_cols(c);
  set_rows(r);
}

void HexGrid::set_hex_size(Vector2 size_){
  size = size_;
}

void HexGrid::set_origin(Vector2 origin_){
  origin = origin_;
}

void HexGrid::set_layout(String layout){
  grid_layout = layout;
  setupLayout();
}

Vector2 HexGrid::point(double x, double y){
  return Vector2(x,y);
}

Vector3 HexGrid::hex(int q, int r, int s)
{
    return Vector3(q,r,s);
}

Vector3 HexGrid::fractionalHex(double q, double r,double s){
  return Vector3(q,r,s);
}

Vector2 HexGrid::offsetCoord(int r, int c){
  return Vector2(r,c);
}

Vector3 HexGrid::hex_add(Vector3 a, Vector3 b)
{
    return Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vector3 HexGrid::hex_subtract(Vector3 a, Vector3 b)
{
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vector3 HexGrid::hex_scale(Vector3 a, int k)
{
    return Vector3(a.x * k, a.y * k, a.z * k);
}

Vector3 HexGrid::hex_direction(int direction)
{
    return hex_directions[direction];
}

Vector3 HexGrid::hex_neighbor(Vector3 hex, int direction)
{
    return hex_add(hex, hex_direction(direction));
}

Array HexGrid::hex_neighbors(Vector3 hex)
{
  Array results;
  for (int i = 0; i < 6; i++)
  {
      results.push_back(hex_neighbor(hex,i));
  }
    return results;
}


int HexGrid::hex_length(Vector3 hex)
{
    return int((abs(hex.x) + abs(hex.y) + abs(hex.z)) / 2);
}

int HexGrid::hex_distance(Vector3 a, Vector3 b)
{
    return hex_length(hex_subtract(a, b));
}

Vector3 HexGrid::hex_round(Vector3 h)
{
    int q = int(round(h.x));
    int r = int(round(h.y));
    int s = int(round(h.z));
    double q_diff = abs(q - h.x);
    double r_diff = abs(r - h.y);
    double s_diff = abs(s - h.z);
    if (q_diff > r_diff and q_diff > s_diff)
    {
        q = -r - s;
    }
    else
        if (r_diff > s_diff)
        {
            r = -q - s;
        }
        else
        {
            s = -q - r;
        }
    return Vector3(q, r, s);
}

Vector3 HexGrid::hex_lerp(Vector3 a, Vector3 b, double t)
{
    return Vector3(a.x * (1 - t) + b.x * t, a.y * (1 - t) + b.y * t, a.z * (1 - t) + b.z * t);
}

Array HexGrid::hex_linedraw(Vector3 a, Vector3 b)
{
    int N = hex_distance(a, b);
    Vector3 a_nudge = fractionalHex(a.x + 0.000001, a.y + 0.000001, a.z - 0.000002);
    Vector3 b_nudge = fractionalHex(b.x + 0.000001, b.y + 0.000001, b.z - 0.000002);
    Array results;
    double step = 1.0 / max(N, 1);
    for (int i = 0; i <= N; i++)
    {
        results.push_back(hex_round(hex_lerp(a_nudge, b_nudge, step * i)));
    }
    return results;
}

Vector2 HexGrid::hex_to_point(Vector3 h)
{
    double x = (double(orient["f0"]) * h.x + double(orient["f1"]) * h.y) * size.x;
    double y = (double(orient["f2"]) * h.x + double(orient["f3"]) * h.y) * size.y;

    return Vector2(x + origin.x, y + origin.y);
}

Vector3 HexGrid::point_to_hex(Vector2 p)
{
  Vector2 pt = Vector2((p.x - origin.x) / size.x, (p.y - origin.y) / size.y);
  double q = double(orient["b0"]) * pt.x + double(orient["b1"]) * pt.y;
  double r = double(orient["b2"]) * pt.x + double(orient["b3"]) * pt.y;

  return hex_round(Vector3(q, r, -q - r));
}

Vector2 HexGrid::hex_corner_offset(int corner)
{
    double angle = 2.0 * M_PI * (corner + double(orient["start_angle"])) / 6;
    return Vector2(size.x * cos(angle), size.y * sin(angle));
}

Array HexGrid::hex_corners(Vector3 h)
{
    Array corners;
    Vector2 center = hex_to_point(h);
    for (int i = 1; i < 7; i++)
    {
        Vector2 offset = hex_corner_offset(i);
        corners.push_back(Vector2(center.x + offset.x, center.y + offset.y));
    }
    return corners;
}

Array HexGrid::hex_edges(Vector3 hex){
	Array edges;
	Array corners = hex_corners(hex);
  for (int i = 0; i < 6; i++){
		int l = i+1;
		if(l == 6){
			l = 0;
    }
    Array edge;
    edge.push_back(corners[l]);
    edge.push_back(corners[i]);
		edges.push_back(edge);
  }
	return edges;
}

//This makes a ring
Array HexGrid::hexes_at_distance(Vector3 hex,int dist)
{
  Array results;
  Vector3 pHex = hex_add(hex,hex_scale(hex_direction(4), dist));
	for (int i = 0; i < 6; i++)
    {
		    for (int j = 0; j < dist; j++){
          results.push_back(pHex);
          pHex = hex_neighbor(pHex,i);
        }
    }
	return results;
}

Array HexGrid::hexes_within_distance(Vector3 hex,int dist)
{
    Array results;
    results.push_back(hex);
    for (int i = 1; i < dist; i++){
      Array subR = hexes_at_distance(hex,i);
      for (int j = 0; j < subR.size(); j++){
        results.push_back(subR[j]);
      }
    }
    return results;
}

bool HexGrid::los_clear_to(Vector3 start, Vector3 finish,Array obstacles){
  bool clear = true;
  Array line;
  line.push_back(hex_to_point(start));
  line.push_back(hex_to_point(finish));

	//This offcenters it just enough to not go through the point GHETTO i know but it sorta works enough
  Vector2 line0 = line[0];
	line0.y = line0.y+0.005;

  for(int i = 0; i < obstacles.size(); i++){
    if(line.size() == 2 && obstacles.size() > i){
      if(line_intersect_hex(obstacles[i],line0,line[1])){
        clear = false;
      }
    }
  }
  return clear;
}

Array HexGrid::los_within_range(Vector3 hex,int dist, Array obstacles, Array checkList){
  Array outList;
  if(checkList.size() == 0){
    checkList = hexes_within_distance(hex, dist);
  }
  for(int i = 0; i < checkList.size(); i++){
    if(los_clear_to(checkList[i], hex,obstacles)){
      outList.push_back(checkList[i]);
    }
  }
  return outList;
}

bool HexGrid::lines_intersect(Vector2 l1Start,Vector2 l1End, Vector2 l2Start, Vector2 l2End){
  bool results = false;

  double line1StartX = l1Start.x;
	double line1StartY = l1Start.y;
	double line1EndX = l1End.x;
	double line1EndY = l1End.y;

	double line2StartX = l2Start.x;
	double line2StartY = l2Start.y;
	double line2EndX = l2End.x;
	double line2EndY = l2End.y;
  double denominator = ((line2EndY - line2StartY) * (line1EndX - line1StartX)) - ((line2EndX - line2StartX) * (line1EndY - line1StartY));

  if(denominator == 0){
    return results;
  }

  double a = line1StartY - line2StartY;
	double b = line1StartX - line2StartX;
	double numerator1 = ((line2EndX - line2StartX) * a) - ((line2EndY - line2StartY) * b);
	double numerator2 = ((line1EndX - line1StartX) * a) - ((line1EndY - line1StartY) * b);
	a = numerator1 / denominator;
	b = numerator2 / denominator;

	if(a > 0 && a < 1 && b > 0 && b < 1){
		results = true;
  }
	return results;
}

bool HexGrid::line_intersect_hex(Vector3 hex,Vector2 lStart, Vector2 lEnd){
	bool crosses = false;
	Array edges = hex_edges(hex);
	for (int i = 0; i < edges.size(); i++){
		if (edges.size() > i){
      Array edge = edges[i];
			if (lines_intersect(lStart,lEnd,edge[0],edge[1])){
				crosses = true;
      }
    }
  }
  return crosses;
}

void HexGrid::createMap(){
  if(grid_layout == "pointy"){
    makePointyMap();
  } else {
    makeFlatMap();
  }
}

void HexGrid::makePointyMap(){
  hex_map.clear();
  for(int r = 0; r<rows; r++){
    double r_offset = floor(r/2);
    for(int q = -1*(r_offset); q < cols-r_offset; q++){
      //String _q;
      //String _r;
      //String _s;
      //_q.num(q);
      //_r.num(r);
      //_s.num(-q-r);
      //String id = _q+"-"+_r+"-"+_s;
      //hex_map[id] = hex(q,r,-q-r);
      hex_map.push_back(hex(q,r,-q-r));
    }
  }
}

void HexGrid::makeFlatMap(){
  hex_map.clear();
  for(int q = 0; q<cols; q++){
    double q_offset = floor(q/2);
    for(int r = -1*(q_offset); r < rows-q_offset; r++){

      //TODO This works if you wanted a dictionary with Ids
      //String _q;
      //String _r;
      //String _s;
      //_q = _q.num(q);
      //_r = _r.num(r);
      //_s = _s.num(-1*(q-r));

      //String id = _q+"-"+_r+"-"+_s;
      //hex_map[id] = hex(q,r,-q-r);
      hex_map.push_back(hex(q,r,-q-r));
    }
  }
}

Array HexGrid::get_map(){
  return hex_map;
}

Array HexGrid::astar_get_path_to(Vector3 startHex, Vector3 endHex, Array obstacles, int dist){
  int lowInd = 0;
  Dictionary currentNode;
  Dictionary curr;
  Dictionary neighbor;
  Dictionary nei;
  Array neighbors;
  Array openList;
  Array closedList;
  Array ret;
//  int n;
  int gScore;
  bool gScoreIsBest;
  bool isClosed;
  bool isObstacle;
  bool visited;
  bool nFound;

  Dictionary tN;
  Dictionary lTN;

  for(int i = 0; i < astar_grid.size(); i++){
    _astar_reset_Nhex(i);
  }

  for(int i = 0; i < astar_grid.size(); i++){
    Dictionary agi = astar_grid[i];
    if(agi["id"] == startHex){
        openList.push_back(agi["id"]);
    }
  }

  int its = 0;
  while(openList.size() > 0){
    its +=1;

    //TODO maybe the low index changes is causing a problem
    lowInd = 0;
    for(int i = 0; i < openList.size(); i++){
      tN = _astar_get_grid_item_from_id(openList[i]);
      lTN = _astar_get_grid_item_from_id(openList[lowInd]);

      if(tN["f"] < lTN["f"]){
        lowInd = i;
      }
    }
    lTN = _astar_get_grid_item_from_id(openList[lowInd]);

    currentNode = lTN;


    if(currentNode["id"] == endHex){

      while(curr["hasParent"]){

        ret.push_back("I am going to loos it");
        return ret;

        ret.push_back(curr);
        curr = _astar_get_grid_item_from_id(currentNode["parent"]);
      //  return ret;
      }

      //i may need to invert the path here
      return ret;
    }

    openList.remove(lowInd);
    ret.push_back(openList.size());
    if(its > 50){
      return ret;
    }

    currentNode["closed"] = true;
    neighbors = currentNode["neighbors"];

    nFound = false;
    for(int i = 0; i < neighbors.size(); i++){

      neighbor = _astar_get_grid_item_from_id(neighbors[i]);
      if(neighbor["found"]){
        nFound = true;
      }

      if(nFound == true){
        isClosed = neighbor["closed"];
        isObstacle = neighbor["isObstacle"];
        visited = neighbor["visited"];

        if(neighbor.size() >=1 && isClosed == false && isObstacle == false ){
          gScore = int(currentNode["g"]) + 1;
          gScoreIsBest = false;
          if(visited == false){
              gScoreIsBest = true;
              neighbor["h"] = hex_distance(neighbor["id"], endHex);
              neighbor["visited"] = true;

              openList.push_back(neighbor["id"]);

          } else if(gScore < int(neighbor["g"])){
            gScoreIsBest = true;
          }

          if(gScoreIsBest){

            neighbor["parent"] = currentNode["id"];
            neighbor["hasParent"] = true;
            neighbor["g"] = gScore;
            neighbor["f"] = double(neighbor["g"])+double(neighbor["h"]);

          }
        }
      }
    }
  }

  return ret;

}

//the ranglist is a list of all the tiles or the possible tiles to get to
void HexGrid::astar_grid_setup(Array obstacles, Array rangeList){
    astar_grid.clear();
    for (int i=0; i < rangeList.size(); i++){
      Dictionary nHex = _astar_gridify_hex(rangeList[i]);
			for(int o = 0; o < obstacles.size(); o++){
        if (obstacles[o] == nHex){
          nHex["isObstacle"] = true;
        }
      }
      astar_grid.push_back(nHex);
    }
}

void HexGrid::_astar_reset_Nhex(int index){
  //this is probably wrong ... I should be using pointers or something
  Dictionary i = astar_grid[index];
  i["f"] = 0;
  i["g"] = 0;
	i["h"] = 0;
	i["debug"] = "";
	i["parent"] = false;
  i["hasParent"] = false;
	i["closed"] = false;
	i["visited"] = false;
  i["found"] = true;

  astar_grid[index] = i;
}

Dictionary HexGrid::_astar_get_grid_item_from_id(Vector3 hex){
  Dictionary t;
  for(int i = 0; i < astar_grid.size(); i++){
    t = astar_grid[i];
    if(t["id"] == hex){
        return t;
    }
  }
  t["found"] = false;
  return t;
}

Dictionary HexGrid::_astar_gridify_hex(Vector3 hex){
  Dictionary nHex;
  //this is ghetto but I cannot seem to use normal ways for converting numbers to strings so I'm just going to use the vector for the id until I figure it out
  nHex["id"] = hex;
  if(hex_map.find(hex) > -1){
    nHex["neighbors"] = hex_neighbors(hex);//engine._neighbors(hGrid.map[hex])
		nHex["q"] = hex.x;
		nHex["r"] = hex.y;
		nHex["s"] = hex.z;
		nHex["f"] = 0;
		nHex["g"] = 0;
		nHex["h"] = 0;
	//	nHex.debug = ""
		nHex["parent"] = false;
    nHex["hasParent"] = false;
		nHex["isObstacle"] = false;
		nHex["closed"] = false;
		nHex["visited"] = false;
    nHex["found"] = true;
  }
  return nHex;
}

void HexGrid::_bind_methods() {
    ObjectTypeDB::bind_method("set_rows",&HexGrid::set_rows);
    ObjectTypeDB::bind_method("set_cols",&HexGrid::set_cols);
    ObjectTypeDB::bind_method("set_layout",&HexGrid::set_layout);

    ObjectTypeDB::bind_method("hex",&HexGrid::hex);
    ObjectTypeDB::bind_method("point",&HexGrid::point);
    ObjectTypeDB::bind_method("hex_add",&HexGrid::hex_add);
    ObjectTypeDB::bind_method("hex_subtract",&HexGrid::hex_subtract);
    ObjectTypeDB::bind_method("hex_scale",&HexGrid::hex_scale);
    ObjectTypeDB::bind_method("hex_distance",&HexGrid::hex_distance);
    ObjectTypeDB::bind_method("hex_neighbor",&HexGrid::hex_neighbor);
    ObjectTypeDB::bind_method("hex_neighbors",&HexGrid::hex_neighbors);
    ObjectTypeDB::bind_method("hex_round",&HexGrid::hex_round);
    ObjectTypeDB::bind_method("hex_linedraw",&HexGrid::hex_linedraw);
    ObjectTypeDB::bind_method("set_hex_size",&HexGrid::set_hex_size);
    ObjectTypeDB::bind_method("set_origin",&HexGrid::set_origin);

    ObjectTypeDB::bind_method("hex_to_point",&HexGrid::hex_to_point);
    ObjectTypeDB::bind_method("point_to_hex",&HexGrid::point_to_hex);
    ObjectTypeDB::bind_method("hex_corners",&HexGrid::hex_corners);
    ObjectTypeDB::bind_method("hex_edges",&HexGrid::hex_edges);

    ObjectTypeDB::bind_method("get_map",&HexGrid::get_map);

    ObjectTypeDB::bind_method("line_intersect_hex",&HexGrid::line_intersect_hex);
    ObjectTypeDB::bind_method("lines_intersect",&HexGrid::lines_intersect);
    ObjectTypeDB::bind_method("los_clear_to",&HexGrid::los_clear_to);
    //los_within_range
    ObjectTypeDB::bind_method("los_within_range",&HexGrid::los_within_range);

    ObjectTypeDB::bind_method("hexes_at_distance",&HexGrid::hexes_at_distance);
    ObjectTypeDB::bind_method("hexes_within_distance",&HexGrid::hexes_within_distance);

    ObjectTypeDB::bind_method("astar_get_path_to",&HexGrid::astar_get_path_to);
    ObjectTypeDB::bind_method("astar_grid_setup",&HexGrid::astar_grid_setup);

}
