#include<iostream>
#include<fstream>
#include<vector>
#include<algorithm>
#include<set>
using namespace std;

class Comp{
    public:
    bool operator() (const pair<float,int>& a, const pair<float,int>& b) const{
        return a.first<b.first;
    }
};
int find_hamming(string a, string b){
    // cout<<a<<" "<<b<<endl;
    if(a.size()!=b.size())
        throw "Unequal lengths";
    int n = a.size();
    int total_count=0, count=0;
    int res;
    for(int i=0; i<n; i++){
        res = a[i]^b[i];
        count=0;
        while(res){
            count+=(res&1);
            res = res>>1;
        }
        total_count+=count;
    }
    return total_count;
}

vector<int> find_keysize(string& line){
    int min_dist = INT_MAX;
    float cur_dist;
    int m=1;
    vector<string> temp(4);
    set<pair<float,int>,Comp> dist_set;
    vector<int> dist_list;
    for(int i=2; i<=40; i++){
        cur_dist=0;
        temp[0] = line.substr(0,i);
        temp[1] = line.substr(i,i);
        temp[2] = line.substr(2*i,i);
        temp[3] = line.substr(3*i,i);
        for(int k=0; k<m; k++){
            cur_dist += find_hamming(temp[k],temp[k+1]);
        }
        cur_dist/=(i*m);
        // cout<<cur_dist<<endl;
        dist_set.insert({(float)cur_dist, i});
    }
    int j=0;
    cout<<"Minimum length keys\n";
    for(auto iter:dist_set){
        if(j++<4){
            cout<<iter.second<<" "<<iter.first<<endl;
            dist_list.push_back(iter.second);
        }
    }
    return dist_list;
}

int main(){
    string a = "this is a test";
    string b = "wokka wokka!!!";
    cout<<"Hamming distance is "<<find_hamming(a,b)<<endl;
    ifstream fs;
    fs.open("data6.txt");
    string line;
    string line_cat;
    while(fs>>line){
        line_cat = line_cat+line;
    }
    /* int key_size = */
    cout<<line_cat.size();
    vector<int> key_sizes = find_keysize(line_cat);
    string temp;
    for(int i=0; i<key_sizes.size(); i++){
        temp = line_cat;
        if(temp.size()%key_sizes[i]!=0){
            int diff = key_sizes[i]-temp.size()%key_sizes[i];
            temp = string(diff,'0')+temp;
        }
        vector<string> blocks;
        for(int j=0; j<temp.size()/key_sizes[i]; j++){
            blocks.push_back(temp.substr(j*key_sizes[i], key_sizes[i]));
        }
        vector<string> transposed(key_sizes[i], string(temp.size()/key_sizes[i],'0'));
        for(int j=0; j<temp.size()/key_sizes[i]; j++){
            for(int k=0; k<key_sizes[i]; k++){
                transposed[k][j] = blocks[j][k];
            }
        }
        cout<<blocks[0][1]<<" "<<transposed[1][0]<<endl;
        
    }
}