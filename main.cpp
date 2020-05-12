#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <stack>
#include <ctime>



class Graph{
private:
    int nVertices, source, destination, k;
    //index is node1, pair<node2, weight>
    std::vector<std::pair<int, float>> * adj;

public:
    Graph(int vertices): nVertices{vertices}, adj{new std::vector<std::pair<int, float>>[vertices]}{}


    void addEdge(int node1, int node2, float weight){
        adj[node1].emplace_back(node2, weight);
    }

    void addPathGoals(int s, int d, int kx){
        source = s;
        destination = d;
        k = kx;

    }

    int getSource(){
        return source;
    }

    int getTarget(){
        return destination;
    }

    std::vector<std::pair<float, int>> dijkstraAlg(int start){

        std::vector<std::pair<float, int>> distances(nVertices, std::make_pair(INT_MAX, -1));
        std::priority_queue<std::pair<float, int>> pQueue;
        pQueue.push(std::make_pair(0, start));
        distances[start].first = 0;

        while(!pQueue.empty()){
            auto j = pQueue.top().second;

            pQueue.pop();
            for(auto i = adj[j].begin(); i != adj[j].end(); i++){
                int v = (*i).first;
                float weight = (*i).second;

                if(distances[v].first > distances[j].first + weight){

                    distances[v].first = distances[j].first + weight;
                    distances[v].second = j;
                    pQueue.push(std::make_pair(distances[v].second, v));
                }

            }
        }

        return distances;

    }


    void findKShortest(int s, int d){
        std::vector<float> kShortest(k, INT_MAX);
        auto distances = dijkstraAlg(s);
        kShortest.push_back(0);
        std::stack<int> shortestPath;
        int n = distances[getTarget()].second;

        while(n != -1){
            shortestPath.push(n);
            n = distances[n].second;
        }

        std::vector<int> visited;

        float min = INT_MAX;
        while(!shortestPath.empty()){
            auto current = shortestPath.top();

            shortestPath.pop();
            for(auto & i : adj[current]){
                int v = i.first;
                float weight = i.second;

                bool found = false;
                for(auto j : distances){
                    if(v == getTarget() || j.second == v || (std::find(visited.begin(),visited.end(), v) != visited.end())){
                        found = true;
                        break;
                    }
                }
                if(!found){
                    visited.push_back(v);
                    float temp = (distances[v].first - distances[getTarget()].first) + dijkstraAlg(v)[getTarget()].first;
                    if(temp < kShortest[k - 1]){
                        kShortest[k - 1] = temp;
                        std::sort(kShortest.begin(), kShortest.end());

                    }
                }
            }
        }
        for(int i = 0; i < k; i++){
            if(i == 0){
                std::cout << distances[getTarget()].first + kShortest[i];
            } else {
                std::cout << ", " << distances[getTarget()].first + kShortest[i];
            }
        }
        std::cout << std::endl;
    }
};

Graph setup(const std::string &fileName);

int main(int argc, char *argv[]) {
    Graph graph = setup(argv[1]);
    clock_t start, end;

    start = clock();
    graph.findKShortest(graph.getSource(), graph.getTarget());
    end = clock();

    std::cout << ((double) (end - start)) << std::endl;

    return 0;
}

Graph setup(const std::string &fileName){
    std::ifstream inputFile;
    inputFile.open(fileName);
    std::string s;
    std::getline(inputFile, s);
    std::string::size_type mark;

    // Get number of edges and vertices from the top of the input file
    int nVertices = stoi(s, &mark);
    int nEdges = stoi(s.substr(mark));

    Graph graph(nVertices);

    int a, b;
    float w;

    // get bulk of input (node1, node2, weightBetween)
    for(int i = 0; i < nEdges; i++){
        std::getline(inputFile, s);
        a = stoi(s, &mark);
        s = s.substr(mark);
        b = stoi(s, &mark);
        w = stof(s.substr(mark));

        graph.addEdge(a, b, w);
    }

    std::getline(inputFile, s);
    int source = stoi(s, &mark);
    s = s.substr(mark);
    int destination = stoi(s, &mark);
    int k = stoi(s.substr(mark));

    graph.addPathGoals(source, destination, k);


    return graph;
}
