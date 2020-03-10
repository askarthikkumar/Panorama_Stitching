//
//  Tree.hpp
//  SBM
//
//  Created by A S KARTHIK SAI VISHNU KUMAR on 09/03/20.
//  Copyright © 2020 cmu. All rights reserved.
//

#ifndef Tree_hpp
#define Tree_hpp

// just for output
#include <iostream>

// stl containers
#include <unordered_map>
#include <vector>
#include <utility>
#include <set>

// math
#include <math.h>

#endif /* Tree_hpp */

//typedefs
typedef std::vector<double> Point;
typedef unsigned NodeId;


 class Node{
 public:
     Point point;
     NodeId id;
     Node(){
         this->id=-1;
     }
     Node(Point point,NodeId id=-1){
         this->point=point;
         this->id=id;
     }
};

double distance(Point A, Point B){
    double sum=0.0;
    for(int i=0; i<A.size(); i++){
        sum+=pow(A[i]-B[i],2);
    }
    sum = sqrt(sum);
    return sum;
}


class Comp{
public:
    Point key;
    bool reverse;
    Comp(Point& point, bool reverse=false){
        this->key=point;
        this->reverse=reverse;
    }
    bool operator() (const Node* A, const Node* B){
        if(reverse){//max_heap
            return distance(A->point,this->key) < distance(B->point, this->key);
        }
        //min_heap
        return distance(A->point,this->key) > distance(B->point, this->key);
    }
};


typedef std::set<Node*, Comp> pqueue;


class BoundedPQ{
public:
    pqueue* bpq=NULL;
    int k;
    BoundedPQ(int k, Point pt){
        this->k=k;
        Comp comp(pt);
        this->bpq = new pqueue(comp);
    }
    void push(Node* node){
        this->bpq->insert(node);
        if(this->bpq->size()>=k){
            this->bpq->erase(--this->bpq->end());
        }
    }
    void pop(){
        this->bpq->erase(this->bpq->begin());
    }
    Node* top(){
        return *(this->bpq->begin());
    }
};


class Tree{
public:
    unsigned D, counter;
    std::unordered_map<NodeId, NodeId> parent_map;
    std::unordered_map<NodeId, Node*> node_list;
    Tree(unsigned D){
        this->D=D;
        this->counter=0;
    }
    virtual NodeId insert(Point &pt, NodeId parent=0)=0;
    Node* get_node(NodeId id){
        if(node_list.find(id)==node_list.end()){
            throw "Key not found!";
        }
        return node_list[id];
    }
    std::vector<Node*> get_nearest_nodes(Point &pt, int k){
        BoundedPQ bp(k,pt);
        for(auto& iter:node_list){
            bp.push(iter.second);
        }
        std::vector<Node*> result(bp.bpq->begin(), bp.bpq->end());
        return result;
    }
};
