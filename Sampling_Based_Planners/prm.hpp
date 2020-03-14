#ifndef prm_hpp
#define prm_hpp
#include <random>
#include "tree.hpp"
#include <fstream>

class PRM: public Tree{
public:
    double sf,*map;
    int x_size,y_size,episodes;
    Point start,end;
    bool is_terminal;
    // debug variables
    double min_dist, min_ee_dist;
    std::ofstream nodes,path,joints;
    PRM(unsigned D, double* map, int x_size, int y_size);
    NodeId insert(const Point &pt, NodeId parent = 0) override;
    std::pair<Point,int> new_config(const Point& q_near, const Point& q);
    int connect(Node* node, RRT* tree);
    void backtrack(double*** plan, int* planlength);
    void plan(double* start, double* goal, double*** plan, int* planlength);
    void random_config(Point& q_rand, std::default_random_engine eng);
    std::vector<double> forward_kinematics(const Point& angles);
    bool present(const Point& pt);
};
#endif
