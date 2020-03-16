//
//  main.cpp
//  PRM
//
//  Created by A S KARTHIK SAI VISHNU KUMAR on 09/03/20.
//  Copyright © 2020 cmu. All rights reserved.
//

#include "rrt.hpp"
#include "rrt_connect.hpp"
#include "prm.hpp"
#include <iostream>

int IsValidArmConfiguration(std::vector<double> angles, 
        int numofDOFs, double* map,int x_size, int y_size){
            return true;
        };
int main(int argc, const char * argv[]) {
    
    std::cout << "Hello, World!\n";
    double map[9]={0,0,0,0,0,0,0,0,0};
    PRM rtree(2,map,3,3);
    double start[2]={0,0}; 
    double goal[2]={2,2}; 
    double** plan = NULL;
    int planlength = 0;
    rtree.plan(start,goal,&plan,&planlength);
    // std::cout<<rtree.insert({2.0,3.0});
    // std::cout<<rtree.extend({5.0,5.0});
    // std::cout<<rtree.extend({-5.0,-5.0});
    // std::cout<<rtree.extend({6.0,6.0});
    // std::cout<<rtree.get_nearest_nodes({6.0,6.0})[0]->id;
    return 0;
}
