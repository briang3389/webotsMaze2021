//bfs.h: runs bfs to search for next tile

//some helper structures to run BFS
queue<pair<int, int>> bfsQueue;
pair<int, int> boardParents[boardSize][boardSize];
bool traveled[boardSize][boardSize];

pair<int, int> bfs(pair<int, int> node) {
    //bfs with queue, search for closest unvisited node
    if (!board[node.first][node.second].visited) {
        return node;
    }
    if (traveled[node.first][node.second]) return make_pair(-1, -1);
    traveled[node.first][node.second] = true;

    for (int i = 0; i < 4; i++) {
        if (board[node.first][node.second].open[i]) {
            pair<int, int> neighbor = neighborTile(node, i);
                if (!traveled[neighbor.first][neighbor.second] && !board[neighbor.first][neighbor.second].isHole) {
                bfsQueue.push(neighbor);
                boardParents[neighbor.first][neighbor.second] = node;
            }
        }
    }

    return make_pair(-1, -1);
}

pair<int, int> runBFS(pair<int, int> startNode) {
    //runs bfs after changing a few variables
    while (!bfsQueue.empty()) bfsQueue.pop();
    for (int i = 0; i < boardSize; i++) {
        for (int j = 0; j < boardSize; j++) {
            boardParents[i][j] = make_pair(-1, -1);
            traveled[i][j] = false;
        }
    }

    bfs(startNode);

    pair<int, int> inp;
    pair<int, int> result = startNode;
    while (!bfsQueue.empty()) {
        inp = bfsQueue.front();
        result = bfs(inp);
        if (result.first != -1) break;
        bfsQueue.pop();
    }
    return result;
}