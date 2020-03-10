#include<iostream>
#include<sstream>

using namespace std;

string repeating_xor(string line, string key){
    string encoded_str,temp;
    int key_ptr = 0, key_len = key.size(), n = line.size();
    stringstream ss;
    int val,xor_prod;
    string hexmap = "0123456789abcdef";
    if(n%2!=0)
        line = "0"+line;
    for(int i=0; i<n; i++){
        val = (int)line[i];
        xor_prod = val^(int)key[key_ptr];
        encoded_str.push_back(hexmap[xor_prod/16]);
        encoded_str.push_back(hexmap[xor_prod%16]);
        key_ptr = (key_ptr+1)%key_len;
    }
    return encoded_str;
}

int main(){
    string val = "Burning 'em, if you ain't quick and nimble\n"
            "I go crazy when I hear a cymbal";
    stringstream ss;
    ss<<val;
    string line;
    cout<<repeating_xor(val,"ICE")<<endl;
    /*
    while(ss.peek()!=EOF){
        getline(ss,line);
        cout<<line<<endl;;
        cout<<repeating_xor(line,"ICE")<<endl;
    }
    */
}