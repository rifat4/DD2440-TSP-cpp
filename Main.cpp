#include <iostream>
#include <stdio.h>
#include <vector>
#include <utility>
#include <queue>
#include <chrono>
#include <cmath>

using namespace std;

//number of points
int N;

double distMat[1000][1000];

const int NEIGHBOURS = 20;

chrono::high_resolution_clock::time_point start;

int nearestNeighbours[1000][NEIGHBOURS];
//priority_queue<pair<double, int> > nearestNeighbours[1000];

//pair<double, double> cords[1000];

//std::vector<std::vector<double> > distances;
std::vector<std::pair<double, double> > cords;

void readPoints(){
    for(int i = 0; i < N; i++){
        double x, y;
        std::cin >> x >> y;
        cords.push_back(std::make_pair(x, y));
    }
}

void printPoints(){
    cerr << N << std::endl;
    for(int i = 0; i < N; i++){
        double x, y;
        cerr << cords[i].first << " " << cords[i].second << std::endl;
    }
}

//calculates euclidean distance
double dist(pair<double, double> a, pair<double, double > b){
    double xdiff = a.first - b.first;
    double ydiff = a.second - b.second;
    return sqrt(xdiff * xdiff + ydiff * ydiff);
}

void calculateDist(){
    vector<double> disti;

    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            if(i == j) continue;
            distMat[i][j] = dist(cords[i], cords[j]);
        }
    }
}

void printdistances(){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            cerr << distMat[i][j] << endl;
        }
    }
}

void calculateNearestNeighbours(){
    for(int i = 0; i < N; i++){
        double longestDist = numeric_limits<double>::max();
        priority_queue<pair<double, int> > nearestNeighboursQ;

        for(int j = 0; j < N; j++){
            if(i == j) continue;
            if(nearestNeighboursQ.size() >= NEIGHBOURS){
                if(distMat[i][j] < nearestNeighboursQ.top().first){
                    nearestNeighboursQ.pop();
                    nearestNeighboursQ.push(make_pair(distMat[i][j], j));
                }
            } else {
                nearestNeighboursQ.push(make_pair(distMat[i][j], j));
            }
        }
        int index = 0;
        while(!nearestNeighboursQ.empty()){
            nearestNeighbours[i][index] = nearestNeighboursQ.top().second;
            nearestNeighboursQ.pop();
            index++;
        }
    }
}

/*
 * if(counter < NEIGHBOURS){
                nearestNeighbours[i][counter] = j;
                if(distances[i][j] < longestDist){
                    distances[i][j] = longestDist;
                    index = counter;
                }
                counter++;
            } else {
                if(distances[i][j] < longestDist){
                    nearestNeighbours[i][index];
                }
            }
 */

void printNearestNeighbours(){
    int LIMIT = NEIGHBOURS;
    if(N < NEIGHBOURS) LIMIT = N - 1;
    for(int i = 0; i < N; i++){
        for(int j = 0; j < LIMIT; j++){
            cerr << nearestNeighbours[i][j] << " ";
        }
        cerr << endl;
    }
}

void solveNearestNeighbour(int* path){
    //pair<double, double> start = cords[0];

    bool used[N];
    for(int i = 0; i < N; i++){
        used[i] = false;
    }

    path[0] = 0;
    used[0] = true;

    int index = 0;

    int previous = 0;

    for(int i = 0; i < N - 1; i++){
        double shortestDist = numeric_limits<double>::max();
        for(int j = 0; j < N; j++){
            if(used[j]) continue;
            if(distMat[previous][j] < shortestDist){
                shortestDist = distMat[previous][j];
                index = j;
            }
        }
        previous = index;
        path[i + 1] = index;
        used[index] = true;
        //cout << used[9] << endl;
    }

}

void printPath(int* path){
    for(int i = 0; i < N; i++){
        cout << path[i] << endl;
    }
}


//could potentially be improved to O(n/2) instead of O(n)
//tried to implement in java but was slower than just doing O(n)
void swap(int i, int j, int* path){
    i++;
    while(i < j){
        int temp = path[i];
        path[i] = path[j];
        path[j] = temp;
        i++;
        j--;
    }
}

bool twoOPT(int i, int j, int* path){
    double a = -(distMat[path[i]][path[i + 1]] + distMat[path[j]][path[(j + 1)] % N]); //TODO mod only used for 0, so maybe change handle it in cheaper way.
    double b = distMat[path[i]][path[j]] + distMat[path[i + 1]][path[(j + 1)] % N];
    double lengthDelta = a + b;
    if(lengthDelta >= 0) return false;
    swap(i, j, path);
    //cerr << "swapped" << endl;
    return true;
}

void twoOPT(int* path){
    bool changed = true;
    while(changed){
        changed = false;
        
        for(int i = 0; i < N - 1; i++){
            for(int j = i + 2; j < N; j++){
                if(twoOPT(i, j, path)) changed = true;
            }
        }
        chrono::high_resolution_clock::time_point end_time = chrono::high_resolution_clock::now();
        chrono::milliseconds elapsed_time = chrono::duration_cast<chrono::milliseconds>(end_time - start);
        if(elapsed_time.count() > 1800) break;
    }

}

int main(){
    //read in number of points
    cin >> N;

    start = chrono::high_resolution_clock::now();

    //initiate the distance size of our 2d vector to NxN.
    //distances.resize(N, std::vector<double>(N));
    //cords.resize(N);

    //cerr << "reading points" << endl;
    readPoints();
    //cerr << "printing points" << endl;
    //printPoints();

    //cerr << "calculating distances" << endl;
    calculateDist();
    //cerr << "printing distances" << endl;
    //printdistances();

    //cerr << "calculating nearest neighbours" << endl;
    calculateNearestNeighbours();
    //cerr << "printing nearest neighbours" << endl;
    //printNearestNeighbours();

    int path[N];

    solveNearestNeighbour(path);

    twoOPT(path);

    printPath(path);

    return 0;
}
