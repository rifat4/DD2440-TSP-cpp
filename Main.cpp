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


//worst case could potentially be improved to O(n/2) instead of O(n) and average to O(1/4)?
//tried to implement in java but was slower than just this
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


//When we want to reverse the list from j to i, where j > i
//That is reversing from ex index 4 to 2, so if N = 6 then 5<->2, 0<->1
//3-4 should remain unchanged then because the cut is being made between i and i+1 and j and j+1
//so i+1 to j should remain unchanged
void swapLoopOverZero(int* path, int i, int j){
    j++;
    j -= N;
    while(j < i){
        int temp = path[i];
        path[i] = path[j + N];
        path[j + N] = temp;
        i--;
        j++;
    }
}

void twoOPTUnderZero(int* path, int i, int j){
    j++;
    j -= N;
    while(j < i){

    }
}

double distance(int* path, int i, int j){
    return distMat[path[i]][path[j]];
}


//decides what paths to reverse
void swap(int i, int j, int k, int OPTCase, int* path){
    switch (OPTCase){
        case 1:
            swap(i, k, path);
            break;
        case 2:
            swap(i, j, path);
            break;
        case 3:
            swap(j, k, path);
            break;
        case 4:
            swap(i,j, path);
            swap(j,k, path);
            break;
        case 5:
            swap(i,j, path);
            swap(j,k, path);
            swap(i,k, path);
            break;
        case 6:
            swap(i,j, path);
            swap(i,k, path);
            break;
        case 7:
            swap(j,k, path);
            swap(i,k, path);
            break;
        
        
    default:
        cerr << "error reached default in swap()" << endl;
    }
}



void threeOPT(int* path){
    bool changed = true;
    while(changed){
        
        int OPTCase = -1;
        double longestDelta = 0;
        double delta;
        changed = false;
        for(int i = 0; i < N - 2; i++){
            for(int j = i + 2; j < N - 1; j++){
                for(int k = j + 2; k < N; k++){

                    longestDelta = 0;

                    //cerr << "segfault? here? " << endl;

                    //distance between i and i+1
                    double iDist = distMat[path[i]][path[i+1]];
                    double jDist = distMat[path[j]][path[j+1]];
                    double kDist = distMat[path[k]][path[(k+1) % N]];

                    //cerr << "yes here" << endl;

                    
                    //permutation 3OPT #1 2OPT
                    double a = -(iDist + kDist);
                    double b = distMat[path[i]][path[k]] + distMat[path[i+1]][path[(k+1) % N]];
                    delta = a + b;
                    if(delta < longestDelta){
                        longestDelta = delta;
                        OPTCase = 1;
                    }

                    //permutation 3OPT #2 2OPT
                    a = -(iDist + jDist);
                    b = distMat[path[i]][path[j]] + distMat[path[i+1]][path[j+1]];
                    delta = a + b;
                    if(delta < longestDelta){
                        longestDelta = delta;
                        OPTCase = 2;
                    }

                    //permutation 3OPT #3 2OPT
                    a = -(jDist + kDist);
                    b = distMat[path[j]][path[k]] + distMat[path[j+1]][path[(k+1) % N]];
                    delta = a + b;
                    if(delta < longestDelta){
                        longestDelta = delta;
                        OPTCase = 3;
                    }


                    //permutation 3OPT #4
                    a = -(iDist + jDist + kDist);
                    b = distMat[path[i]][path[j]] + distMat[path[i+1]][path[k]] + distMat[path[j+1]][path[(k+1) % N]];
                    delta = a + b;
                    if(delta < longestDelta){
                        longestDelta = delta;
                        OPTCase = 4;
                    }


                    //permutation 3OPT #5
                    b = distMat[path[i]][path[j+1]] + distMat[path[i+1]][path[k]] + distMat[path[j]][path[(k+1) % N]];
                    delta = a + b;
                    if(delta < longestDelta){
                        longestDelta = delta;
                        OPTCase = 5;
                    }


                    //permutation 3OPT #6
                    b = distMat[path[i]][path[k]] + distMat[path[i+1]][path[j+1]] + distMat[path[j]][path[(k+1) % N]];
                    delta = a + b;
                    if(delta < longestDelta){
                        longestDelta = delta;
                        OPTCase = 6;
                    }


                    //permutation 3OPT #7
                    b = distMat[path[i]][path[j+1]] + distMat[path[i+1]][path[(k+1) % N]] + distMat[path[j]][path[k]];
                    delta = a + b;
                    if(delta < longestDelta){
                        longestDelta = delta;
                        OPTCase = 7;
                    }


                    if(longestDelta < 0){
                        changed = true;
                        //cerr << "i " << i << " j " << j << " k " << k << " OPTCase " << OPTCase << " longestdelta " << longestDelta << endl;
                        swap(i, j, k, OPTCase, path);
                    }

                }
            }
            chrono::high_resolution_clock::time_point end_time = chrono::high_resolution_clock::now();
            chrono::milliseconds elapsed_time = chrono::duration_cast<chrono::milliseconds>(end_time - start);
            if(elapsed_time.count() > 1980) return;

        }

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
    //calculateNearestNeighbours();
    //cerr << "printing nearest neighbours" << endl;
    //printNearestNeighbours();

    int path[N];

    solveNearestNeighbour(path);

    //twoOPT(path);

    threeOPT(path);

    printPath(path);

    return 0;
}
