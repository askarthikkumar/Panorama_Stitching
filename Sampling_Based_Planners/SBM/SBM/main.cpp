//
//  main.cpp
//  SBM
//
//  Created by A S KARTHIK SAI VISHNU KUMAR on 08/03/20.
//  Copyright © 2020 cmu. All rights reserved.
//

// just for output
#include <iostream>

// stl containers
#include <unordered_map>
#include <vector>
#include <utility>
#include <queue>

// math
#include <math.h>

//typedefs
typedef std::vector<double> Point;
typedef unsigned NodeId;

template<typename T>
std::ostream& operator << (std::ostream& os, std::vector<T> list){
    for(auto& item: list){
        os<<item<<" ";
    }
    return os;
}
// create a KD Tree
 class Node{
 public:
     Point point;
     Node *left, *right;
     int level;
     NodeId id;
     Node(){
         this->left=NULL;
         this->right=NULL;
         this->level=0;
         this->id=-1;
     }
     Node(Point point,NodeId id=-1, int level=0){
         this->left=NULL;
         this->right=NULL;
         this->level=level;
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

typedef std::priority_queue<Node*, std::vector<Node*>, Comp> pqueue;

class KDTree{
public:
    unsigned D, counter;
    std::unordered_map<NodeId, NodeId> parent_map;
    std::unordered_map<NodeId, Node*> node_list;
    Node* root;
    KDTree(int D){
        this->D=D;
        this->counter=0;
        root=NULL;
    }
    
    unsigned insert(Point& point){
        unsigned depth=0;
        // search if tree is empty
        if(this->root==NULL){
            NodeId node_id = ++this->counter;
            this->parent_map[node_id] = 0;
            this->root = new Node(point, node_id);
            this->node_list[node_id] = this->root;
            return node_id;
        }
        // iteratively traverse the tree
        unsigned axis = depth%this->D;
        Node* cur = this->root;
        Node* parent = NULL;
        int left = 0;
        while(true){
            if(cur==NULL){
                // not present in the tree create and add new node
                NodeId node_id = ++this->counter;
                this->node_list[node_id] = new Node(point, node_id, depth);
                if(left)parent->left=this->node_list[node_id];
                else parent->right=this->node_list[node_id];
                if(parent!=NULL){
                    this->parent_map[node_id]=parent->id;
                }
                else{
                    throw "Parent not tracked!";
                }
                std::cout<<"Parent is "<<parent->point<<std::endl;
                return node_id;
            }
            if(point==cur->point){
                std::cout<<"Already present!\n";
                return cur->id;
            }
            else if(point[axis] < cur->point[axis]){
                // search left
                left=1;
                depth++;
                axis=depth%this->D;
                parent=cur;
                cur=cur->left;
            }
            else{
                // search right
                left=0;
                depth++;
                axis=depth%this->D;
                parent=cur;
                cur=cur->right;
            }
        }
    }
    
    Node* get_node(NodeId id){
        if(node_list.find(id)==node_list.end()){
            throw "Key not found!";
        }
        return node_list[id];
    }
    
    void nearest_node_(Point& point, Node* cur, pqueue& pq, int depth=0){
        // traverse till we get to a leaf node
        int axis=depth%this->D;
        bool isleft=false;
        
        if(cur->left==NULL && cur->right==NULL){
            // some leaf node reached
            pq.push(cur);
            //pq_r.push(cur);
            return;
        }
        else if(point[axis]<cur->point[axis]){
            //go left
            if(cur->left!=NULL){
                nearest_node_(point, cur->left, pq, depth+1);
                isleft=true;
            }
            else{
                pq.push(cur);
            }
        }
        else{
            //go right
            if(cur->right!=NULL){
                nearest_node_(point, cur->right, pq, depth+1);
                isleft=false;
            }
            else{
                pq.push(cur);
            }
        }
        //check if current node's hyperplane can split current hypersphere
        if(fabs(point[axis]-cur->point[axis])<distance(pq.top()->point,point)){
            if(isleft && cur->right!=NULL)
                nearest_node_(point, cur->right, pq, depth+1);
            else if(cur->left!=NULL)
                nearest_node_(point, cur->left, pq, depth+1);
        }
        return;
    }
    
    Point nearest_node(Point& point){
        Comp comp(point), comp_r(point,true);
        pqueue pq(comp); //pq_r(comp_r);
        this->nearest_node_(point, this->root, pq, 0);
        return pq.top()->point;
    }
    
    std::vector<Point> k_nearest_node(Point& point){
        
        return std::vector<Point>();
    }
};
int main(int argc, const char * argv[]) {
    KDTree tree(2);
    std::vector<Point> p_list = {{30,40}, {5,25}, {10,12}, {70,70}, {50,30}, {35,45}, {60,40}, {5,25}} ;
    for(auto item:p_list){
        std::cout<<tree.insert(item)<<std::endl;
    }
    Point key = {5.0,25.0};
    std::cout<<tree.nearest_node(key);
    return 0;
}
