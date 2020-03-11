#include "rrt.hpp"
#include <fstream>
#include <math.h>

#if !defined(GETMAPINDEX)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)
#endif

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654
#define LINKLENGTH_CELLS 10
#define LOGGING 0

std::vector<double> RRT::forward_kinematics(const Point& angles){
    double x0,y0,x1,y1;
        int i;
        //iterate through all the links starting with the base
        x1 = ((double)this->x_size)/2.0;
        y1 = 0;
        for(i = 0; i < this->D; i++)
        {
            //compute the corresponding line segment
            x0 = x1;
            y0 = y1;
            x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
            y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);
        }
    return {x1,y1};
}

template<typename T>
void log(std::vector<T> pt, std::ofstream& ofs){
    if(LOGGING==0)
        return;
    for(int i=0; i<pt.size(); i++)
        ofs<<pt[i]<<" ";
    ofs<<std::endl;
    return;
}

int IsValidArmConfiguration(std::vector<double> angles, 
        int numofDOFs, double* map,int x_size, int y_size);

template<typename T>
std::ostream& operator << (std::ostream& os, std::vector<T> list){
    for(int i=0; i<list.size(); i++){
        os<<list[i]<<" ";
    }
    return os;
}

Point operator + (Point A, Point B){
    Point res(A.size());
    for(int i=0; i<A.size(); i++)
        res[i]=A[i]+B[i];
    return res;
}

Point operator - (Point A, Point B){
    Point res(A.size());
    for(int i=0; i<A.size(); i++)
        res[i]=A[i]-B[i];
    return res;
}

Point operator * (double k, Point A){
    Point res(A.size());
    for(int i=0; i<A.size(); i++)
        res[i]=k*A[i];
    return res;
}

RRT::RRT(unsigned D, double* map, int x_size, int y_size):Tree(D){
    this->ext_eps=0.5;
    this->sf=20.0;
    this->map=map;
    this->x_size=x_size;
    this->y_size=y_size;
    this->episodes=30000;
    this->term_th=0.01;
    this->is_terminal=false;
    this->exploit_th=0.15;
    //debug variable initialisations
    this->min_dist=1000000;
    this->min_ee_dist=1000000;
    this->nodes=std::ofstream("nodes.txt");
    this->path=std::ofstream("path.txt");
    this->joints=std::ofstream("joints.txt");
}

bool RRT::present(const Point &pt){
    for(auto& item:this->node_list){
        if(item.second->point==pt)
            return true;
    }
    return false;
}

NodeId RRT::insert(const Point &pt, NodeId parent){
    if(this->present(pt)){
        // std::cout<<"Already present\n";
        return 0;
    }
    if(this->counter==0){
        ++this->counter;
        this->node_list[counter]=new Node(pt,counter);
        this->parent_map[counter]=0;
    }
    else{
        ++this->counter;
        // auto result=this->get_nearest_nodes(pt);
        // NodeId parent = result[0]->id;
        this->node_list[this->counter]=new Node(pt,this->counter);
        // std::cout<<"Parent of "<<counter<<"is "<<parent<<std::endl;
        this->parent_map[this->counter]=parent;
    }
    // debug
    auto coord=this->forward_kinematics(pt);
    log(coord,nodes);
    log(pt,joints);
    // end of debug
    return this->counter;
}

std::pair<Point,int> RRT::new_config(const Point& q_near, const Point& q){
    double dist=distance(q_near, q);
    Point unit_vec=(1/dist)*(q-q_near),q_sampled;
    int reachable_flg=dist<this->ext_eps?1:0;
    dist=dist<this->ext_eps?dist:ext_eps;
    int no_samples = (int)(this->sf);
    double step_dist = dist/no_samples;
    int prev=0, break_flg=0,i=0;
    for(i=0; i<no_samples; i++){
        q_sampled = q_near+step_dist*i*unit_vec;
        if(!IsValidArmConfiguration(q_sampled, this->D, this->map, 
                this->x_size, this->y_size)){
                break_flg=1;
                break;
        }
    }
    if(i==1||i==0){//trapped
        // if(IsValidArmConfiguration(q_sampled, this->D, this->map, this->x_size, this->y_size))
        if(i==0 && break_flg==1){
            std::cout<<"False Trap"<<std::endl;
            std::cout<<IsValidArmConfiguration(q_sampled, this->D, this->map, 
                this->x_size, this->y_size)<<" q_near\n";
            // std::cout<<q_sampled<<std::endl;
            std::cout<<q_near<<std::endl;
            std::cout<<q<<std::endl;
        }
        return {q_sampled,0};
    }
    //advanced or reached
    if(break_flg){
        //advanced but did not reach destination
        Point res=q_near+step_dist*(i-1)*unit_vec;
        // std::cout<<"Brk "<<IsValidArmConfiguration(res, this->D, this->map, 
            // this->x_size, this->y_size)<<std::endl;
        if(!IsValidArmConfiguration(res, this->D, this->map, 
            this->x_size, this->y_size)){
                std::cout<<"Invalid config advanced partially\n";
        }
        return {res,1};
    }
    else{
        //for loop completed so completely advanced
        if(reachable_flg){
            // std::cout<<"reachable "<<IsValidArmConfiguration(q, this->D, this->map, 
            // this->x_size, this->y_size)<<std::endl;
            if(!IsValidArmConfiguration(q_sampled, this->D, this->map, 
            this->x_size, this->y_size)){
                std::cout<<"Invalid config reached\n";
            }
            return {q_sampled,2};
        }
        else{
            // Point res=q_near+this->ext_eps*(q-q_near);
            // std::cout<<"Adv "<<IsValidArmConfiguration(q_sampled, this->D, this->map, 
            // this->x_size, this->y_size)<<std::endl;
            if(!IsValidArmConfiguration(q_sampled, this->D, this->map, 
            this->x_size, this->y_size)){
                std::cout<<"Invalid config advanced fully\n";
            }
            return {q_sampled,1};
        }
    }
}

int RRT::extend(const Point &q){
    auto result=this->get_nearest_nodes(q);
    Point q_near;
    if(!result.empty())
        q_near = result[0]->point;
    auto signal = this->new_config(q_near, q);
    if(signal.second==0){
        return 0;
    }
    else{
        //debug statements
        double cur_dist=distance(signal.first, this->end);
        /*
        auto coord=this->forward_kinematics(signal.first);
        double cur_ee_dist=distance(coord, this->forward_kinematics(this->end));
        if(cur_ee_dist<this->min_ee_dist){
            this->min_ee_dist=cur_ee_dist;
            // std::cout<<"Close to Goal EE \n";
            // std::cout<<signal.first<<std::endl;
            // std::cout<<coord<<std::endl;
            // std::cout<<"Min Dist is "<<cur_dist<<std::endl;
        }
        */
        if(cur_dist<this->term_th){
            this->is_terminal=true;
            std::cout<<"Closest Distance is "<<cur_dist<<std::endl;
            std::cout<<signal.first<<std::endl;
        }

        NodeId cur_id=this->insert(signal.first,result[0]->id);
        this->terminal_id=cur_id;

        if(cur_dist<this->min_dist){
            this->min_dist=cur_dist;
            this->closest_id=cur_id;
            // std::cout<<"New config closer\n";
            // std::cout<<signal.first<<std::endl;
        }
        return signal.second;
    }
}

void RRT::backtrack(double*** plan, int* planlength){
    *plan = NULL;
    *planlength = 0;
    std::vector<NodeId> result;
    NodeId cur=this->terminal_id;
    while(cur!=0){
        std::cout<<this->node_list[cur]->point<<std::endl;
        result.push_back(cur);
        cur=this->parent_map[cur];
    }
    int num_samples=(int)result.size();
    *plan = (double**) malloc(num_samples*sizeof(double*));
    int i=0;
    for(auto it=result.rbegin(); it!=result.rend(); it++){
        // debug
        // std::cout<<IsValidArmConfiguration(node_list[*it]->point, this->D, this->map,
        // this->x_size, this->y_size);
        auto coord=this->forward_kinematics(node_list[*it]->point);
        log(coord,path);
        // end of debug
        (*plan)[i] = (double*) malloc(this->D*sizeof(double));
        
        for(int j=0; j<this->D; j++){
            (*plan)[i][j]=this->node_list[*it]->point[j];
        }
        i++;
    }
    *planlength=num_samples;
    
    return;
}

void RRT::random_config(Point& q_rand, std::default_random_engine eng){
    return;
}

void RRT::plan(double* start, 
            double* goal, 
            double*** plan, 
            int* planlength){

    Point q_near;
    int signal;
    // Seeds which work: (seed=10,eps_th=0.5,explt_th=0.15)
    unsigned seed=0,exp_seed=0;
    std::default_random_engine eng(seed);
    std::default_random_engine exp_eng(exp_seed);
    std::uniform_real_distribution<double> u_dist(0,2*PI);
    std::uniform_real_distribution<double> explt(0.0,1.0);
    
    this->start=Point(start,start+(int)this->D);
    this->end=Point(goal,goal+(int)this->D);
    std::cout<<this->forward_kinematics(this->start)<<" Start Position"<<std::endl;
    std::cout<<this->forward_kinematics(this->end)<<" End Position"<<std::endl;
    
    // Initialize
    this->insert(Point(start,start+(int)this->D));
    for(int i=0; i<this->episodes; i++){
        if(!this->is_terminal){
            Point q_rand(this->D);
            // double scale=(double)i/this->episodes;
            // double th=((int)(scale*10)%10)/10.0;
            // double th=0.1;
            // if(scale>0.9)th=1;
            if(explt(exp_eng)>this->exploit_th){
                for(int i=0; i<this->D; i++)q_rand[i]=u_dist(eng);
            }
            else{
                q_rand=this->end;
            }
            // std::cout<<q_rand<<std::endl;
            signal = this->extend(q_rand);
        }
        else{
            std::cout<<"Terminal condition reached\n"<<i<<
            " Nodes sampled\n";
            break;
        }
        if(i%1000==0){
            std::cout<<i<<" Nodes sampled\n";
            std::cout<<"Minimum distance till now is "<<this->min_dist<<std::endl;
            if(i==this->episodes){
                std::cout<<"No path found after sampling "<<i<<" nodes\n";
                break;
            }
        }
    }
    if(this->is_terminal){
        std::cout<<"Path Found!\n";
        this->backtrack(plan,planlength);
    }
    else{
        std::cout<<"No Path found\n";
        this->terminal_id=this->closest_id;
        this->backtrack(plan,planlength);
    }
    log(this->forward_kinematics(this->start),nodes);
    log(this->forward_kinematics(this->end),nodes);
    nodes.close();
    path.close();
    joints.close();
    return;
};
