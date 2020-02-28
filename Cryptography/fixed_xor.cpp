#include<iostream>
#include<sstream>

using namespace std;

string fixed_xor(string val1, string val2){
    string result;
    stringstream ss1,ss2;
    string hex_map = "0123456789abcdef";
    int temp1, temp2, xor_product;
    // if the strings are not of same length throw error
    if(val1.size()!=val2.size())
        throw "Lengths is unequal\n";
    for(int i=val1.size()-1; i>=0; i--){
        ss1<<val1[i];
        ss2<<val2[i];
        ss1>>hex>>temp1;
        ss2>>hex>>temp2;
        ss1.clear();
        ss2.clear();
        xor_product = temp1^temp2;
        result.push_back(hex_map[xor_product]);
    }
    reverse(result.begin(),result.end());
    return result;
}

int main(){
    string val1="1c0111001f010100061a024b53535009181c";
    string val2="686974207468652062756c6c277320657965";
    cout<<fixed_xor(val1,val2)<<endl;
}