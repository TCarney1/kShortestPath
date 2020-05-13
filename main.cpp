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
        // stores the current distance, and which node it came from.
        std::vector<std::pair<float, int>> distances(nVertices, std::make_pair(INT_MAX, -1));
        // so we can use the shortest values first.
        std::priority_queue<std::pair<float, int>> pQueue;
        pQueue.push(std::make_pair(0, start));
        // to store minimum distances to get to each node.
        distances[start].first = 0;

        while(!pQueue.empty()) {
            auto j = pQueue.top().second;
            pQueue.pop();

            for (auto i = adj[j].begin(); i != adj[j].end(); i++) {
                // v is current vertex
                int v = (*i).first;
                float weight = (*i).second;

                // compares current route vs route through new vertex + the shortest distance it takes to get to that node.
                if (distances[v].first > distances[j].first + weight) {
                    distances[v].first = distances[j].first + weight;
                    distances[v].second = j;
                    pQueue.push(std::make_pair(distances[v].second, v));
                }

            }
        }
        return distances;
    }


    void findKShortest(int s, int dest){
        // store the value to be added to the shortest path for k shortest paths.
        std::vector<float> kShortest(k, INT_MAX);
        // init the first value with 0 since it will be our shortest path and we dont need to add anything.
        kShortest.push_back(0);

        // get the distances for the shortest path.
        auto distances = dijkstraAlg(s);
        // to store the shortest path (reversing it as it is backwards now).
        std::stack<int> shortestPath;
        // start at the last NON goal node of the shortest path.
        int n = distances[dest].second;

        // no point calculating the already visited nodes. since it doesnt matter in the case which node we come from.
        std::vector<int> visited;

        // reverses and stores the shortest path of nodes.
        // we will use these nodes to check for other routes to find the k shortest paths.
        while(n != -1){
            shortestPath.push(n);
            n = distances[n].second;
        }




        float min = INT_MAX;
        // go through every node in the shortest path (Not including the goal node)
        while(!shortestPath.empty()){
            auto current = shortestPath.top();
            shortestPath.pop();
            // go through all the adjacent nodes to this node (check the nodes next to the path)
            for(auto & i : adj[current]){
                // v is vertex
                int v = i.first;
                float weight = i.second;

                bool found = false;
                //*****************************************************************************************************
                // this checks if the current node is either: on the shortest path, is the goal node or has
                // already been visited. I found that doing it this way is faster on average as any true case results
                // in breaking out of the loop. Where as if we search for the false cases we have to search the entire
                // list every time.
                //*****************************************************************************************************
                for(auto j : distances){
                    if(j.second == v || v == dest || (std::find(visited.begin(),visited.end(), v) != visited.end())){
                        found = true;
                        break;
                    }
                }
                if(!found){
                    visited.push_back(v);
                    // temp = (weight to get here - best total weight) + weight left.
                    float temp = (distances[v].first - distances[dest].first) + dijkstraAlg(v)[dest].first;
                    // keep track of the kShortest routes.
                    if(temp < kShortest[k - 1]){
                        // replace the longest one if we find one shorter.
                        kShortest[k - 1] = temp;
                        // sort to make sure the longest one gets replaced next time.
                        std::sort(kShortest.begin(), kShortest.end());

                    }
                }
            }
        }
        // output shortest k distances. (adds the extra distance onto the shortest path)
        for(int i = 0; i < k; i++){
            if(i == 0){
                std::cout << distances[dest].first + kShortest[i];
            } else {
                std::cout << ", " << distances[dest].first + kShortest[i];
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
