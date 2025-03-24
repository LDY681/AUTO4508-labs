#include "./myheap.h"
#include "eyebot.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

char *FILENAME = "nodes.txt";
#define MAX_NODES 20
#define LINE_SIZE 100
#define POSITION_OFFSET 100
#define MATH_INF 9999

typedef struct {
    int x;
    int y;
    int num_neighbors;
    int neighbors[MAX_NODES];
} Node;

// Connection matrix to store distances (-1 = no connection)
double connection_matrix[MAX_NODES][MAX_NODES];

// Nodes map
Node nodes[MAX_NODES];
int num_nodes = 0;

// Shortest path
int shortest_path[MAX_NODES];
int path_length = 0;

// Euclidean distance
double euclidean_distance(int x1, int y1, int x2, int y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void set_bot() {
    if (num_nodes == 0) {
        printf("Please load nodes first (KEY1).\n");
        return;
    }
    SIMSetRobot(0, nodes[0].x + POSITION_OFFSET, nodes[0].y + POSITION_OFFSET, 0, 0);
	VWSetPosition(nodes[0].x, nodes[0].y, 0);
    printf("\nRobot set to %d %d.\n", nodes[0].x, nodes[0].y);
}

void load_nodes(char *FILENAME) {
    FILE *file = fopen(FILENAME, "r");
    if (!file) {
        printf("Failed to open file \n");
        return;
    }

    char line[LINE_SIZE];

    // Initialize connection matrix
    for (int i = 0; i < MAX_NODES; i++)
        for (int j = 0; j < MAX_NODES; j++)
            connection_matrix[i][j] = -1;

    // Populate nodes map
    while (num_nodes < MAX_NODES && fgets(line, sizeof(line), file)) {
        char *token;
        int count = 0;

        // Populate x and y of current node
        token = strtok(line, " ");
        if (!token) continue;
        nodes[num_nodes].x = atoi(token);

        token = strtok(NULL, " ");
        if (!token) continue;
        nodes[num_nodes].y = atoi(token);

        while ((token = strtok(NULL, " ")) && count < MAX_NODES - 1) {
            int neighbor = atoi(token) - 1;
            if (neighbor >= 0 && neighbor < MAX_NODES) {
                nodes[num_nodes].neighbors[count++] = neighbor;
            }
        }
        nodes[num_nodes].num_neighbors = count;
        num_nodes++;
    }
    fclose(file);

    // Calculate euclidean distances and populate connection matrix
    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < nodes[i].num_neighbors; j++) {
            connection_matrix[i][nodes[i].neighbors[j]] = euclidean_distance(nodes[i].x, nodes[i].y, nodes[nodes[i].neighbors[j]].x, nodes[nodes[i].neighbors[j]].y);

            // Print edges on the LCD
            LCDLineAlign(nodes[i].x, nodes[i].y, nodes[nodes[i].neighbors[j]].x, nodes[nodes[i].neighbors[j]].y, 'w');
        }
    }
    
    // Print Connection Matrix
    printf("\nConnection Matrix:\n");
    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < num_nodes; j++) {
            printf("%.1f ", (connection_matrix[i][j] == -1) ? (i == j) ? 0.0: -1.0 : connection_matrix[i][j]);
        }
        printf("\n");
    }
}

// Drive the robot along the computed shortest path
void drive_along_path(Node nodes[]) {
    if (path_length == 0) {
        printf("No path available. Please run A-star first (KEY2).\n");
        return;
    } else if (path_length == 1) {
        printf("No path found!\n");
        return;
    }

    int x, y, phi;
    VWGetPosition(&x, &y, &phi); // Get initial position

    printf("\nStart driving...\n");
    for (int i = path_length - 1; i >= 0; i--) {
        int node_index = shortest_path[i];
        int target_x = nodes[node_index].x;
        int target_y = nodes[node_index].y;

        // Compute relative angle 
        int dx = target_x - x;
        int dy = target_y - y;
        double target_theta = atan2(dy, dx) * 180 / M_PI;

        // Compute euclidean distance
        double distance = euclidean_distance(x, y, target_x, target_y);

        // Compute turning angle
        double diff = target_theta - phi;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;

        // Turn the robot
        VWTurn(diff, 50);
        VWWait();
        // printf("Turned %.2f degrees\n", diff);

        // Move straight
        VWStraight(distance, 100);
        VWWait();

        // Update position and print the path
        int prev_x = x, prev_y = y;
        VWGetPosition(&x, &y, &phi);
        printf("Arrived at node %d, current position %d, %d \n", node_index + 1, x, y);
        LCDLineAlign(prev_x, prev_y, x, y, 'r');
    }
    printf("Reached the goal!\n");
}

// A-star Algorithm (Finds shortest path and stores it)
void a_star(Node nodes[], int num_nodes) {
    int start = 0, goal = num_nodes - 1;
    double g_score[MAX_NODES];   // Currently known score of the cheapest path from start to n
    double f_score[MAX_NODES];   // Our current best guess f_score = g_score + heuristic(n), this is the one to be stored in priority queue
    int from_node[MAX_NODES];   // An independent node list that indicates the traversed path

    for (int i = 0; i < MAX_NODES; i++) {
        g_score[i] = MATH_INF;
        f_score[i] = MATH_INF;
        from_node[i] = -1;
    }

    g_score[start] = 0;
    f_score[start] = euclidean_distance(nodes[start].x, nodes[start].y, nodes[goal].x, nodes[goal].y);

    // Start at the start node
    PriorityNode open_set[MAX_NODES];
    int open_size = 0;
    push(open_set, &open_size, start, f_score[start]);

    while (open_size > 0) {
        // Travel to the next node in line as the  current node
        int current = pop(open_set, &open_size);
        // Stop at goal
        if (current == goal) break;

        // Traverse through the neighbors of the current node
        for (int i = 0; i < nodes[current].num_neighbors; i++) {
            int neighbor = nodes[current].neighbors[i];
            if (connection_matrix[current][neighbor] == -1) continue;

            // Could-be score if we take this path
            double tentative_g_score = g_score[current] + connection_matrix[current][neighbor];

            // If the could-be path is shorter, update the path
            if (tentative_g_score < g_score[neighbor]) {
                // Update traversed path
                from_node[neighbor] = current;
                // Update the g and f scores
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = g_score[neighbor] + euclidean_distance(nodes[neighbor].x, nodes[neighbor].y, nodes[goal].x, nodes[goal].y);
                // Proceed to the next node
                push(open_set, &open_size, neighbor, f_score[neighbor]);
            }
        }
    }

    path_length = 0;
    for (int curr_node = goal; curr_node != -1; curr_node = from_node[curr_node]) {
        shortest_path[path_length++] = curr_node;
    }

     // Print the shortest path
     printf("\nShortest Path from Node 1 to Node %d:\n", num_nodes);
     if (path_length == 1) {
         printf("No path found!\n");
     } else {
         for (int i = path_length - 1; i >= 0; i--) {
             printf("%d%s", shortest_path[i] + 1, (i == 0) ? "\n" : " -> ");
         }
         printf("Total Distance: %.2f\n", g_score[goal]);
     }
}

// Main function with menu system
int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: ./astar.x filename.txt \n");
        return EXIT_FAILURE;
    }
    FILENAME = argv[1];

    LCDMenu("Load", "Calculate", "Set", "Run");

    while (1) {
        int key = KEYRead();
        if (key == KEY1) {
            load_nodes(FILENAME);
        } else if (key == KEY2) {
            a_star(nodes, num_nodes);
        } else if (key == KEY3) {
            set_bot();
        } else if (key == KEY4) {
            drive_along_path(nodes);
        } else {
            OSWait(100);
        }
    }
    return 0;
}
