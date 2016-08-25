#ifndef HEXGRID_H
#define HEXGRID_H
#include "scene/main/node.h"

class HexGrid : public Node {
	OBJ_TYPE(HexGrid,Node);
  int cols;
  int rows;
	Vector2 size;
	Vector2 origin;
	Dictionary orient;
	Array hex_directions;
	String grid_layout;
	Dictionary _grid_layout;

	Array astar_grid;
	Array hex_map;

	//these may be a waste
	Dictionary layout_pointy;//orientations
	Dictionary layout_flat;//orientaions

protected:
	void setUpConstants();
  static void _bind_methods();

public:
	void setupLayout();
	void createMap();
	void makeFlatMap();
	void makePointyMap();

	Vector3 hex(int q, int r, int s);
	Vector3 fractionalHex(double q, double r, double s);
	Vector2 point(double x, double y);
	Vector2 offsetCoord(int r, int c);
	Vector3 hex_add(Vector3 a, Vector3 b);
	Vector3 hex_subtract(Vector3 a, Vector3 b);
	Vector3 hex_scale(Vector3 a, int k);
	Vector3 hex_direction(int direction);
	Vector3 hex_neighbor(Vector3 hex, int direction);
	Array hex_neighbors(Vector3 hex);
	int hex_length(Vector3 hex);
	int hex_distance(Vector3 a, Vector3 b);
	Vector3 hex_round(Vector3 h);
	Vector3 hex_lerp(Vector3 a, Vector3 b, double t);
	Array hex_linedraw(Vector3 a, Vector3 b);
	Vector2 hex_to_point(Vector3 h);
	Vector3 point_to_hex(Vector2 p);
	Vector2 hex_corner_offset(int corner);
	Array hex_corners(Vector3 h);
	Array hex_edges(Vector3 hex);
	Array hexes_at_distance(Vector3 hex,int dist);
	Array hexes_within_distance(Vector3 hex,int dist);

	bool line_intersect_hex(Vector3 hex,Vector2 lStart, Vector2 lEnd);
	bool lines_intersect(Vector2 l1Start,Vector2 l1End, Vector2 l2Start, Vector2 l2End);
	bool los_clear_to(Vector3 start, Vector3 finish,Array obstacles);
	Array los_within_range(Vector3 hex,int dist, Array obstacles, Array checkList);

	void set_hex_size(Vector2 size_);
	void set_origin(Vector2 origin_);
	Array get_map();

	void set_layout(String layout);
  void set_rows(int r);
  void set_cols(int c);
	void set_rows_and_cols(int c, int r);
  int get_values();

	Array astar_get_path_to(Vector3 startHex, Vector3 endHex, Array obstacles, int dist);
	void astar_grid_setup(Array obstacles, Array rangeList);
	void _astar_reset_Nhex(int index);
	Dictionary _astar_gridify_hex(Vector3 hex);
	Dictionary _astar_get_grid_item_from_id(Vector3 hex);
  HexGrid();
};

#endif
