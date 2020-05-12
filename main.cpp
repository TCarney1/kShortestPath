#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>




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

    void dijkstraAlg(){

        std::vector<float> distances(nVertices, INT_MAX);
        std::priority_queue<std::pair<float, int>> pQueue;
        std::cout << source << std::endl;
        pQueue.push(std::make_pair(0, source));
        distances[source] = 0;


        while(!pQueue.empty()){
            auto j = pQueue.top().second;
            pQueue.pop();
            for(auto i = adj[j].begin(); i != adj[j].end(); i++){
                int v = (*i).first;
                float weight = (*i).second;
                if(distances[v] > distances[j] + weight){
                    distances[v] = distances[j] + weight;
                    pQueue.push(std::make_pair(distances[v], v));
                }

            }
        }
        /*
        for(int i = 0; i < nVertices; i++){
            std::cout << "Vertex: " << i << " Distance: " << distances[i] << std::endl;
        }
        */
        std::cout << "Source to Destination = " << distances[destination] << std::endl;
    }

};

Graph setup(const std::string &fileName);

int main(int argc, char *argv[]) {
    Graph graph = setup(argv[1]);

    graph.dijkstraAlg();

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
