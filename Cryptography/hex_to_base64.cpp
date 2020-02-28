#include <string>
#include <sstream>
#include <iostream>

using namespace std;

string hex_to_base64(string val){
    string temp;
    string base64_str;
    long val_hex;
    std::stringstream ss;
    string base64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    // check if length is odd
    cout<<"Length of original string is "<<val.size()<<endl;
    if(val.size()%6!=0){
        string zeros(6-val.size()%6,'0');
        cout<<"Appending "<<zeros.size()<<" zeros"<<endl;
        val = zeros+val;
    }
    int n = val.size()/2;
    for(int i=0; i<val.size(); i=i+6){
        temp = val.substr(i,6);
        ss<<temp;
        ss>>hex>>val_hex;
        string base64_substr;
        for(int i=0; i<4; i++){
            base64_substr.push_back(base64[val_hex%64]);
            val_hex = val_hex/64;
        }
        reverse(base64_substr.begin(), base64_substr.end());
        base64_str = base64_str+base64_substr;
        ss.clear();
    }
    cout<<"Converted string length is "<<base64_str.size()<<endl;
    return base64_str;
}
int main(){
    
    string val = "49276d206b696c6c696e6720796f757220627261696e206c696b65206120706f69736f6e6f7573206d757368726f6f6d";
    cout<<hex_to_base64(val)<<endl;
}