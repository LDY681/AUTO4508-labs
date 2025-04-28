/* brushfire.c - Final with Voronoi, Node Reduction, and A* Run */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "image.h"
#include "eyebot.h"

#define IMAGE_SIZE 128
#define WORLD_SIZE 4000
#define MAX_QUEUE_SIZE (IMAGE_SIZE * IMAGE_SIZE)
#define DRIVE_SPEED 300
#define INF 1000000
#define NODE_FILE "nodes.txt"

/* Types and directions */
typedef struct {
    int x, y;
} Point;

const int dirX[4] = {-1, 0, 1, 0};
const int dirY[4] = {0, 1, 0, -1};
const int dir8X[8] = {-1, 0, 1, 0, -1, -1, 1, 1};
const int dir8Y[8] = {0, 1, 0, -1, -1, 1, -1, 1};
const int colors[] = {RED, GREEN, BLUE, YELLOW, CYAN, TEAL, MAGENTA, PURPLE, MAROON, ORANGE, OLIVE};

Point queue[MAX_QUEUE_SIZE];
int front = 0, rear = 0;

void enqueue(int x, int y) { queue[rear++] = (Point){x, y}; }
Point dequeue() { return queue[front++]; }
int is_queue_empty() { return front == rear; }

Point pbm_to_wld(Point pbm) {
    int wld_x = (int)(pbm.x * WORLD_SIZE / IMAGE_SIZE);
    int wld_y = (int)(pbm.y * WORLD_SIZE / IMAGE_SIZE);
    return (Point){wld_y, WORLD_SIZE - wld_x};
}

int reduce_path(Point* full_path, int full_length, Point* reduced_path, int stride) {
    int reduced_length = 0;
    for (int i = 0; i < full_length; i++) {
        if (i % stride == 0 || i == full_length - 1) {
            reduced_path[reduced_length++] = full_path[i];
        }
    }
    return reduced_length;
}

void label_obstacles(BYTE* image, int label[IMAGE_SIZE][IMAGE_SIZE], int* next_id) {
    memset(label, 0, sizeof(int) * IMAGE_SIZE * IMAGE_SIZE);
    *next_id = 1;

    for (int i = 0; i < IMAGE_SIZE; i++) {
        for (int j = 0; j < IMAGE_SIZE; j++) {
            if (i == 0 || j == 0 || i == IMAGE_SIZE-1 || j == IMAGE_SIZE-1) {
                image[i * IMAGE_SIZE + j] = 1;
            }
        }
    }

    for (int i = 0; i < IMAGE_SIZE; i++) {
        for (int j = 0; j < IMAGE_SIZE; j++) {
            if (image[i * IMAGE_SIZE + j] == 1 && label[i][j] == 0) {
                front = rear = 0;
                enqueue(i, j);
                label[i][j] = (*next_id);
                while (!is_queue_empty()) {
                    Point p = dequeue();
                    for (int d = 0; d < 4; d++) {
                        int ni = p.x + dirX[d];
                        int nj = p.y + dirY[d];
                        if (ni >= 0 && ni < IMAGE_SIZE && nj >= 0 && nj < IMAGE_SIZE) {
                            if (image[ni * IMAGE_SIZE + nj] == 1 && label[ni][nj] == 0) {
                                label[ni][nj] = (*next_id);
                                enqueue(ni, nj);
                            }
                        }
                    }
                }
                (*next_id)++;
            }
        }
    }
}

void create_colored_image(int label[IMAGE_SIZE][IMAGE_SIZE], BYTE* color_img) {
    memset(color_img, 255, IMAGE_SIZE * IMAGE_SIZE * 3);
    for (int i = 0; i < IMAGE_SIZE; i++) {
        for (int j = 0; j < IMAGE_SIZE; j++) {
            int id = label[i][j];
            if (id > 0) {
                int color = colors[(id - 1) % (sizeof(colors)/sizeof(colors[0]))];
                int idx = (i * IMAGE_SIZE + j) * 3;
                color_img[idx] = (color >> 16) & 0xFF;
                color_img[idx + 1] = (color >> 8) & 0xFF;
                color_img[idx + 2] = color & 0xFF;
            }
        }
    }
}

void brushfire_voronoi(BYTE* image, int label[IMAGE_SIZE][IMAGE_SIZE], int voronoi[IMAGE_SIZE][IMAGE_SIZE]) {
    int distance[IMAGE_SIZE][IMAGE_SIZE];
    front = rear = 0;
    memset(voronoi, 0, sizeof(int) * IMAGE_SIZE * IMAGE_SIZE);

    for (int i = 0; i < IMAGE_SIZE; i++) {
        for (int j = 0; j < IMAGE_SIZE; j++) {
            if (label[i][j] > 0) {
                voronoi[i][j] = 1;
                distance[i][j] = 0;
                enqueue(i, j);
            } else {
                distance[i][j] = INF;
            }
        }
    }

    while (!is_queue_empty()) {
        Point p = dequeue();
        for (int d = 0; d < 4; d++) {
            int ni = p.x + dirX[d];
            int nj = p.y + dirY[d];
            if (ni >= 0 && ni < IMAGE_SIZE && nj >= 0 && nj < IMAGE_SIZE) {
                if (distance[ni][nj] == INF) {
                    distance[ni][nj] = distance[p.x][p.y] + 1;
                    label[ni][nj] = label[p.x][p.y];
                    enqueue(ni, nj);
                } else if (label[ni][nj] != label[p.x][p.y]) {
                    if (abs(distance[ni][nj] - distance[p.x][p.y]) <= 1) {
                        voronoi[p.x][p.y] = 2;
                    }
                }
            }
        }
    }

    // Post-processing
    for (int i = 1; i < IMAGE_SIZE-1; i++) {
        for (int j = 1; j < IMAGE_SIZE-1; j++) {
            if (voronoi[i][j] == 0) {
                int seen[256] = {0};
                int unique_labels = 0;
                for (int d = 0; d < 8; d++) {
                    int ni = i + dir8X[d];
                    int nj = j + dir8Y[d];
                    if (ni >= 0 && ni < IMAGE_SIZE && nj >= 0 && nj < IMAGE_SIZE) {
                        int lbl = label[ni][nj];
                        if (lbl > 0 && seen[lbl] == 0) {
                            seen[lbl] = 1;
                            unique_labels++;
                        }
                    }
                }
                if (unique_labels >= 2) {
                    voronoi[i][j] = 2;
                }
            }
        }
    }

        // Manual injection from 4 corners
        int corners[4][2] = {{0,0}, {0,IMAGE_SIZE-1}, {IMAGE_SIZE-1,0}, {IMAGE_SIZE-1,IMAGE_SIZE-1}};
        int step_i[4] = {1, 1, -1, -1};
        int step_j[4] = {1, -1, 1, -1};
    
        for (int c = 0; c < 4; c++) {
            int i = corners[c][0];
            int j = corners[c][1];
            while (i >= 0 && i < IMAGE_SIZE && j >= 0 && j < IMAGE_SIZE) {
                if (voronoi[i][j] == 2) break;
                if (voronoi[i][j] == 0 || voronoi[i][j] == 1) voronoi[i][j] = 2;
                i += step_i[c];
                j += step_j[c];
            }
        }
}

void draw_and_export_voronoi(int voronoi[IMAGE_SIZE][IMAGE_SIZE]) {
    FILE *file = fopen(NODE_FILE, "w");
    if (!file) {
        LCDPrintf("Failed to open nodes.txt\n");
        return;
    }

    Point nodes[MAX_QUEUE_SIZE];
    int node_idx[IMAGE_SIZE][IMAGE_SIZE];
    int node_count = 0;

    for (int i = 0; i < IMAGE_SIZE; i++) {
        for (int j = 0; j < IMAGE_SIZE; j++) {
            node_idx[i][j] = -1;
            if (voronoi[i][j] == 2) {
                int count = 0;
                for (int d = 0; d < 8; d++) {
                    int ni = i + dir8X[d];
                    int nj = j + dir8Y[d];
                    if (ni >= 0 && ni < IMAGE_SIZE && nj >= 0 && nj < IMAGE_SIZE && voronoi[ni][nj] == 2) {
                        count++;
                    }
                }
                if (count != 2) {
                    node_idx[i][j] = node_count;
                    nodes[node_count++] = (Point){i, j};
                }
            }
        }
    }

    // Reduce nodes
    Point reduced_nodes[MAX_QUEUE_SIZE];
    int reduced_count = reduce_path(nodes, node_count, reduced_nodes, 1);   // FIXME do not reduce otherwise breaks connectivity

    memset(node_idx, -1, sizeof(node_idx));
    for (int i = 0; i < reduced_count; i++) {
        node_idx[reduced_nodes[i].x][reduced_nodes[i].y] = i;
    }

    for (int i = 0; i < reduced_count; i++) {
        Point wld = pbm_to_wld(reduced_nodes[i]);
        fprintf(file, "%d %d", wld.x, wld.y);
        for (int d = 0; d < 8; d++) {
            int ni = reduced_nodes[i].x + dir8X[d];
            int nj = reduced_nodes[i].y + dir8Y[d];
            if (ni >= 0 && ni < IMAGE_SIZE && nj >= 0 && nj < IMAGE_SIZE) {
                if (node_idx[ni][nj] != -1) {
                    fprintf(file, " %d", node_idx[ni][nj] + 1);
                }
            }
        }
        fprintf(file, "\n");
    }

    fclose(file);

    for (int i = 0; i < IMAGE_SIZE; i++) {
        for (int j = 0; j < IMAGE_SIZE; j++) {
            printf("%d", voronoi[i][j]);
            if (j == IMAGE_SIZE-1) printf("\n");
            if (voronoi[i][j] == 2) {
                LCDCircle(j, i, 2, RED, 1);
            }
        }
    }
}

void run_astar() {
    system("../lab4/astar.x nodes.txt");
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("Usage: brushfire map.pbm\n");
        return 1;
    }

    BYTE* image;
    read_pbm(argv[1], &image);

    int label[IMAGE_SIZE][IMAGE_SIZE];
    int voronoi[IMAGE_SIZE][IMAGE_SIZE];
    BYTE color_img[IMAGE_SIZE * IMAGE_SIZE * 3];
    int next_id;

    label_obstacles(image, label, &next_id);
    create_colored_image(label, color_img);
    brushfire_voronoi(image, label, voronoi);

    LCDMenu("Binary", "Color", "Voronoi", "Drive");

    while (1) {
        int key = KEYRead();
        switch (key) {
            case KEY1:
                LCDClear();
                LCDMenu("Binary", "Color", "Voronoi", "Drive");
                LCDImageStart(0, 0, IMAGE_SIZE, IMAGE_SIZE);
                LCDImageBinary(image);
                break;
            case KEY2:
                LCDClear();
                LCDMenu("Binary", "Color", "Voronoi", "Drive");
                LCDImageStart(0, 0, IMAGE_SIZE, IMAGE_SIZE);
                LCDImage(color_img);
                break;
            case KEY3:
                draw_and_export_voronoi(voronoi);
                break;
            case KEY4:
                run_astar();
                break;
        }
    }

    free(image);
    return 0;
}
