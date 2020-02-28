#include<iostream>
#include<sstream>
#include<vector>
#include<math.h>
#include<fstream>
using namespace std;

float get_score(string str){
    // cout<<str<<endl;
    vector<float> exp_freq = {8.167, 1.492, 2.202, 4.253, 12.702, 2.228, 2.015,
                        6.094, 6.966, 0.153, 1.292, 4.025, 2.406, 6.749,
                        7.507, 1.929, 0.095, 5.987, 6.327, 9.356, 2.758,
                        0.978, 2.560, 0.150, 1.994, 0.077};
    float score= 0, exp_score;
    vector<float> obs_freq(26,0);
    char temp;
    int count = 0;
    for(int j=0; j<str.size(); j++){
        if(isalpha(str[j])){
            temp = toupper(str[j]);
            obs_freq[temp-65]++;
            count++;
        }
    }
    // get score
    vector<bool> visited(26,false);
    for(int j=0; j<str.size(); j++){
        // get current_freq
        if(isalpha(str[j])){
            temp = toupper(str[j]);
            temp = temp-65;
            if(!visited[temp]){
                exp_score = exp_freq[temp]*count/100;
                // chi squared distribution
                score+= (pow(obs_freq[temp]-exp_score,2)/(exp_score));
                visited[temp] = true;
            }
        }
    }
    return score;
}
string get_most_prob_msg(vector<string> results){
    string result;
    int arg_min=0;
    float min_score = numeric_limits<float>::max(), score;
    for(int i=0; i<results.size(); i++){
        score = get_score(results[i]);
        // cout<<score<<endl;
        // cout<<results[i]<<endl;
        if(score<min_score){
            arg_min = i;
            min_score = score;
        }
    }
    cout<<"Min score is "<<min_score<<endl;
    return results[arg_min];
}

bool valid(char c){
    // cout<<"Here with "<<c<<endl;
    // cout<<c<<endl;
    if(isalpha(c)||isspace(c))
        return true;
    string spl_chars = "\'\".?!;,";
    for(int i=0; i<spl_chars.size(); i++){
        // cout<<spl_chars[i];
        if(c==spl_chars[i]){
            return true;
        }
    }
    return false;

    return isprint(c)||isspace(c);
}
string single_byte_xor(string code){
    string result;
    stringstream ss;
    int hex_val;
    char temp_char;
    string message;
    vector<string> results;
    int flag = 0;
    // if string is not even in length
    if(code.size()%2!=0)
        code = "0"+code;
    for(int k=0; k<256; k++){
        flag=0;
        for(int i=0; i<code.size(); i+=2){
            // cout<<i<<" "<<k<<" ";
            string temp = code.substr(i,2);
            // cout<<temp<<endl;
            ss.clear();
            ss<<temp;
            ss>>hex>>hex_val;
            temp_char = hex_val^k;
            if(!valid(temp_char)){
                flag = 1;
                continue;
            }
            // cout<<"Finished\n";
            message.push_back(temp_char);
        }
        if(!message.empty() && flag==0){
            // cout<<message<<" "<<(char)k<<endl;
            results.push_back(message);
        }
        message.clear();
    }
    if(results.size()==0){
        // cout<<"No valid string found\n";
        return result;
    }
    result = get_most_prob_msg(results);
    return result;
}

int main(){
    string  code = "1b37373331363f78151b7f2b783431333d78397828372d363c78373e783a393b3736";
    cout<<single_byte_xor(code)<<endl;
}
