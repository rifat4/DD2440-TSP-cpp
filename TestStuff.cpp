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

void arrays2(){
    double path[5];

    path[0] = 25;

    arrays(path);

    cerr << path[0] << endl;
}


void conditions(){
    int condition = -1;
    
    switch(condition){
        case 1:
            cout << "one" << endl;
        case -1:
            cout << "two" << endl;
        case 3:
            cout << "i dont want this to be written" << endl;
            break;
        case 4:
            cout << "this 100\% shouldnt be written" << endl;
            break;
        default:
            condition = -1;
    }
}

void declare(){
    double a = 5;
    //double a = 25;

    cout << a << endl;
}

void moduloTesting(){
    int i = 5;
    int b = i % 5;
    cerr << b << endl;
}




int main(){
    moduloTesting();
}