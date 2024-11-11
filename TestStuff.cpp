#include <iostream>
#include <utility>
#include <vector>
#include <queue>


using namespace std;

pair<double, int> dum;

void pq(){
    priority_queue<pair<double, int> > q;
    dum.first = 23.3;
    q.push(make_pair(25.5, 1));
    q.push(make_pair(27.5, 2));
    q.push(make_pair(22.5, 3));
    q.push(make_pair(28.5, 4));

    while(q.size() != 0){
        dum = q.top();
        cout << dum.first << " " << dum.second << "size " << q.size() << endl;
        q.pop();
    }
}

void arrays(double* path){
    path[0] = 5;
}

int main(){
    
    double path[5];

    path[0] = 25;

    arrays(path);

    cerr << path[0] << endl;

}