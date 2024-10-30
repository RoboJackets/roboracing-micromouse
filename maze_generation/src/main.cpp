#include "vendors/GLFW/glfw3.h"
#include <iostream>
#include <stack>
#include <queue>
#include <vector>
#include <set>
#include <map>
#include <random>
#include <unistd.h>
#include <algorithm>



// Define Constants
int dim = 16;
int max = dim * dim;
int interval = 20000;
int seed = 0;
std::string algo;
std::string gen_algo;
std::vector<std::vector<int> > adj_matrix;
std::vector<int> start_pos(140, 40);
int cell_size = 20;

struct Node {
    int pos;
    int cost;
    int manhattan_distance;
};

struct LeastF { 
  int operator()(const Node& n1, const Node& n2) { 
    return (n1.cost + n1.manhattan_distance) -  (n2.cost + n2.manhattan_distance);
  } 
}; 

void print_stack(std::stack<int> s) {
    while (!s.empty()) {
        std::cout << s.top() << " ";
        s.pop();
    }
    std::cout << std::endl;
}

void print_priority_queue(std::priority_queue<Node, std::vector<Node>, LeastF > pq) {
    while (!pq.empty()) {
        std::cout << pq.top().pos << " : " << pq.top().cost + pq.top().manhattan_distance << ", ";
        pq.pop();
    }
    std::cout << std::endl;
}

void print_queue(std::queue<int> q) {
    while (!q.empty()) {
        std::cout << q.back() << " ";
        q.pop();
    }
    std::cout << std::endl;
}

void print_vector(const std::vector<int>& vec) {
    for (const int& element : vec) {
        std::cout << element << " ";
    }
    std::cout << std::endl;
}

static void draw_line(double x1, double y1, double x2, double y2) {
    glBegin(GL_LINES);
    glColor3f(1.f, 1.f, 1.f);
    glVertex3f(x1, y1, 0.f);
    glVertex3f(x2, y2, 0.f);
    glEnd();
}

static void draw_rect(double x, double y, double width, double height, std::string color) {
    glBegin(GL_QUADS);
    if (color == "red")
        glColor3f(1.f, 0.f, 0.f);
    else
        glColor3f(0.f, 1.f, 0.f);
    glVertex3f(x,y,0.f);
    glVertex3f(x + width,y,0.f);
    glVertex3f(x + width,y + height,0.f);
    glVertex3f(x,y + height,0.f);
    glEnd();
}


static std::vector<int> get_neighbors(int i) {
    std::vector<int> neighbors;
    if (i - dim >= 0) neighbors.push_back(i - dim);
    if (i + dim < (dim * dim)) neighbors.push_back(i + dim);
    if ((i - 1 >= 0) && (i - 1) / dim == i / dim) neighbors.push_back(i - 1);
    if ((i + 1) / dim == i / dim) neighbors.push_back(i + 1);
    return neighbors;
}

static std::vector<int> get_connected_neighbors(int i) {
    std::vector<int> connected_neighbors;
    std::vector<int> neighbors = get_neighbors(i);
    for (int neighbor : neighbors) {
        if (adj_matrix[i][neighbor] == 1)
            connected_neighbors.push_back(neighbor);
    }
    return connected_neighbors;
}

static std::vector<int> get_unvisited_neighbors(int i, std::set<int> visited) {
    std::vector<int> unvisited_neighbors;
    std::vector<int> neighbors = get_neighbors(i);
    for (int neighbor : neighbors) {
        if (visited.count(neighbor) == 0)
            unvisited_neighbors.push_back(neighbor);
    }
    return unvisited_neighbors;
}

static std::vector<int> pos_of_cell(int i) {
    std::vector<int> pos;
    pos.push_back(start_pos[0] + (i / dim) * cell_size);
    pos.push_back(start_pos[1] + (i % dim) * cell_size);
    return pos;
}

static void draw_maze() {
    int max = adj_matrix.size();
    int dim = sqrt(max);
    for (int i = 0; i < max; ++i) {
        std::vector<int> pos = pos_of_cell(i);
        if ((i - dim < 0 || adj_matrix[i][i - dim] == 0))
            draw_line(pos[0], pos[1], pos[0], pos[1] + cell_size);
        if ((i + dim >= max || adj_matrix[i][i + dim] == 0))
            draw_line(pos[0] + cell_size, pos[1], pos[0] + cell_size, pos[1] + cell_size);
        if (i != 0 && ((i - 1) / dim != i / dim || adj_matrix[i][i - 1] == 0))
            draw_line(pos[0], pos[1], pos[0] + cell_size, pos[1]);
        if (i != max -1 && ((i + 1) / dim != i / dim || adj_matrix[i][i + 1] == 0))
            draw_line(pos[0], pos[1] + cell_size, pos[0] + cell_size, pos[1] + cell_size);
    }
}

static std::vector<std::vector<int> > generate_maze_rdfs() {
    std::cout<< "Generating maze..." << std::endl;
    int max = dim * dim;
    std::vector<std::vector<int> > adj_matrix;
    for (int i = 0; i < dim * dim; ++i) {
        std::vector<int> row;
        for (int j = 0; j < dim * dim; ++j) {
            row.push_back(0);
        }
        adj_matrix.push_back(row);
    }
    std::uniform_real_distribution<> distr(0, 3);
    std::stack<int> path;

    std::set<int> visited;
    int next, current = 0;
    path.push(current);
    visited.insert(current);
    do {    
        current = path.top();
        path.pop();
        std::vector<int> unvisited;
        if ((unvisited = get_unvisited_neighbors(current, visited)).size() != 0) {
            path.push(current);
            next = unvisited[(int) rand() % unvisited.size()];
            adj_matrix[current][next] = 1;
            adj_matrix[next][current] = 1;
            current = next;
            path.push(current);
        }
        visited.insert(current);
    } while (path.size() != 0);
    return adj_matrix;
}

static void fill_cell(std::vector<int> start_pos, int cell_size,int i, int dim, std::string color) {
    std::vector<int> pos = pos_of_cell(i);
    int offset = cell_size / 3;
    draw_rect(pos[0] + offset, pos[1] + offset, cell_size - 2 * offset, cell_size - 2 * offset, color);
}

static int manhattan_distance_from_goal(int i) {
    return 2 * (dim - 1) - (i % dim) - (i / dim);
}

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

int main(int argc, char* argv[])
{
    if (!glfwInit()) 
        return EXIT_FAILURE;

    if ((argc != 5 && argc != 6)
     || ((algo = std::string(argv[2])) != "dfs" && (algo = std::string(argv[2])) != "dfs_greedy" && (algo = std::string(argv[2])) != "bfs_greedy" && std::string(argv[2]) != "bfs" && std::string(argv[2]) != "astar" && std::string(argv[2]) != "random")
     || ((algo = std::string(argv[1])) != "rdfs")) return -1;

    algo = std::string(argv[2]);
    gen_algo = std::string(argv[1]);
    dim = std::stoi(std::string(argv[3]));
    max = dim * dim;
    start_pos = std::vector<int>(25, 25);
    interval = std::stoi(std::string(argv[4]));
    seed = argc == 5 ? 0 : std::stoi(std::string(argv[5]));
    srand(seed);
    cell_size = 20;

    time_t now = time(0);
    tm* localTime = localtime(&now);

    int hour = localTime->tm_hour;
    int min = localTime->tm_min; 
    int sec = localTime->tm_sec;

    std::string title = gen_algo + " " + algo + " " + std::to_string(dim) + "x" + std::to_string(dim) + " " + std::to_string(seed);

    int window_dim = 50 + dim * cell_size;
    if (window_dim > 800) {
        cell_size = 750 / dim;
        window_dim = 800;
    }

    GLFWwindow* window;
    window = glfwCreateWindow(window_dim, window_dim, title.c_str(), NULL, NULL);
    glfwSetErrorCallback(error_callback);
    glfwSetKeyCallback(window, key_callback);

    if (!window) {
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glfwMakeContextCurrent(window);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, window_dim, window_dim, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    adj_matrix = generate_maze_rdfs();
    int i = 0;
    long distance = 0;
    std::set<int> visited;
    std::set<int> visited_bad;
    std::vector<long> costs(max);
    std::vector<int> manhattan_distances(max);
    std::queue<int> path_queue;
    std::stack<int> path_stack;
    std::priority_queue<Node, std::vector<Node>, 
                           LeastF > frontier; 
    path_queue.push(i);
    Node first;
    first.pos = i;
    first.manhattan_distance = manhattan_distance_from_goal(0);
    first.cost = 0;
    frontier.push(first);
    manhattan_distances[0] = manhattan_distance_from_goal(i);
    costs[0] = -1;
    bool done = false;
    while (!glfwWindowShouldClose(window))
    { 
        glClear(GL_COLOR_BUFFER_BIT);
        if (algo == "random") {
            std::vector<int> neighbors = get_connected_neighbors(i);
            visited.insert(i);
            for (int neighbor : neighbors)
                if (visited.count(neighbor) == 0) {
                    i = neighbor;
                    break;
                }
        } else if (algo == "dfs" && i != max-1) {
            visited.insert(i);
            path_stack.push(i);
            bool found = false;
            while (!path_stack.empty() && i == path_stack.top()) {
                std::vector<int> neighbors = get_connected_neighbors(i);
                for (int neighbor : neighbors){
                    if (visited.count(neighbor) == 0) {
                        i = neighbor;
                        visited.insert(i);
                        found = true;
                        break;
                    } 
                } 
                if (!found) {
                    path_stack.pop();
                    visited_bad.insert(i);
                    i = path_stack.top();
                }
            }
        } else if (algo == "dfs_greedy" && i != max-1) {
            visited.insert(i);
            path_stack.push(i);
            bool found = false;
            while (!path_stack.empty() && i == path_stack.top()) {
                std::vector<int> neighbors = get_connected_neighbors(i);
                long min = LONG_MAX;
                for (int neighbor : neighbors) {
                    if (visited.count(neighbor) == 0 && manhattan_distance_from_goal(neighbor) < min) {
                        min = manhattan_distance_from_goal(neighbor);
                        i = neighbor;
                        found = true;
                    }
                }
                if (found) visited.insert(i);
                else {
                    path_stack.pop();
                    visited_bad.insert(i);
                    i = path_stack.top();
                }
            }
        } else if (algo == "bfs" && i != max-1 && !path_queue.empty()) {
            ++distance;
            i = path_queue.front();
            visited_bad.insert(i);
            std::vector<int> neighbors = get_connected_neighbors(i);
            int valid_neighbor_count = 0;
            for (int neighbor : neighbors)
                if (visited_bad.count(neighbor) == 0) {
                        path_queue.push(neighbor);
                        ++valid_neighbor_count;
                        costs[neighbor] = distance;
                    }
            if (valid_neighbor_count == 0) costs[i] = LONG_MAX;
            if (i == max - 1) {
                visited_bad.erase(i);
                visited.insert(i);
                while (i != 0) {
                    std::vector<int> neighbors = get_connected_neighbors(i);
                    long min = LONG_MAX;
                    for (int neighbor : neighbors) {
                        if (costs[neighbor] < min) {
                            min = costs[neighbor];
                            i = neighbor;
                        }
                    }
                    visited_bad.erase(i);
                    visited.insert(i);
                }
                i = max - 1;
            }
            
            path_queue.pop();
        } else if (algo == "bfs_greedy" && i != max-1 && !path_queue.empty()) {
           
        }
        else if (algo == "astar" && i != max-1) {
            while (visited_bad.count(frontier.top().pos) != 0) frontier.pop();
            i = frontier.top().pos;
            distance = frontier.top().cost;
            ++distance;
            visited_bad.insert(i);
            std::vector<int> neighbors = get_connected_neighbors(i);
            int valid_neighbor_count = 0;
            for (int neighbor : neighbors)
                if (visited_bad.count(neighbor) == 0) {
                    Node n;
                    n.pos = neighbor;
                    n.cost = distance;
                    n.manhattan_distance = manhattan_distance_from_goal(neighbor);
                    frontier.push(n);
                    costs[n.pos] = distance;
                    std::cout <<  (n.cost + n.manhattan_distance) << std::endl;
                    }
            if (i == max - 1) {
                visited_bad.erase(i);
                visited.insert(i);
                while (i != 0) {
                    std::vector<int> neighbors = get_connected_neighbors(i);
                    long min = LONG_MAX;
                    for (int neighbor : neighbors) {
                        if (costs[neighbor] < min) {
                            min = costs[neighbor];
                            i = neighbor;
                        }
                    }
                    visited_bad.erase(i);
                    visited.insert(i);
                }
                i = max - 1;
            }
        }
        for (int cell : visited)
            fill_cell(start_pos, cell_size, cell, sqrt(adj_matrix.size()), "green");
        for (int cell : visited_bad)
            fill_cell(start_pos, cell_size, cell, sqrt(adj_matrix.size()), "red");
        draw_maze();
        if (i == max - 1 && !done) {
            std::cout <<  "SOLVING COMPLETE" << std::endl;
            done = true;
        }
        usleep(interval * 1000);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
