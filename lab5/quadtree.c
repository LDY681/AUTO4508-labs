#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "eyebot.h"
#include "image.h"

#define INITIAL_X 500
#define INITIAL_Y 3500
#define INITIAL_ANGLE 0
#define MINIMUM_QUAD_SIZE_DISPLAY 8  // Minimum size of a square to **DISPLAY**
#define MINIMUM_QUAD_SIZE 8  // Minimum size of a square to **SPLIT**
#define MINIMUM_QUAD_FOR_EDGE 16  // Minimum size of a square to be considered as a valid edge for the Astar, otherwise bot will scratch wall
#define MAX_SQUARES 16384
#define MAX_NEIGHBORS 128

typedef struct {
    int x, y, size; // x, y is the starting cornor of the square
} Square;

typedef struct {
    int x, y;   // x, y is the center of the square
} Point;

// Quadtree squares
Square free_squares[MAX_SQUARES];
Square occupied_squares[MAX_SQUARES];
int free_square_count = 0;
int occupied_square_count = 0;

// Node map
Point nodes[MAX_SQUARES];
Point edges[MAX_SQUARES][MAX_NEIGHBORS];
int edge_count[MAX_SQUARES];    // current number of edges for each node
int node_count = 0;

// Euclidean distance
double euclidean_distance(int x1, int y1, int x2, int y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

int get_or_add_node(Point p) {
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].x == p.x && nodes[i].y == p.y) {
            return i;
        }
    }
    nodes[node_count] = p;
    return node_count++;
}

void add_edge(Point a, Point b) {
    int a_idx = get_or_add_node(a);
    int b_idx = get_or_add_node(b);

    edges[a_idx][edge_count[a_idx]] = b;
    edge_count[a_idx]++;

    edges[b_idx][edge_count[b_idx]] = a;
    edge_count[b_idx]++;
}

void quad(int x, int y, int size, BYTE *image) {
    printf("quad %d %d (%d)\n", x, y, size);
    bool all_free = true, all_occupied = true;
    for (int i = x; i < x + size; i++) {
        for (int j = y; j < y + size; j++) {
            // 0 is free, 1 is occupied
            if (image[i * IMAGE_SIZE + j]) {
                all_free = false;
            } else {
                all_occupied = false;
            }
        }
    }

    if (all_free) {
        printf("free %d %d (%d)\n", x, y, size);
        free_squares[free_square_count++] = (Square){x, y, size};
        // Do not display if size smaller than 16
        if (size >= MINIMUM_QUAD_SIZE_DISPLAY) {
            LCDArea(y, x, y + size, x + size, GREEN, 0);
        }
    } else if (all_occupied || size <= MINIMUM_QUAD_SIZE) { // Treat smallest allowed square as occupied
        printf("occupied %d %d (%d)\n", x, y, size);
        occupied_squares[occupied_square_count++] = (Square){x, y, size};
        // Do not display if size smaller than 16
        if (size >= MINIMUM_QUAD_SIZE_DISPLAY) {
            LCDArea(y, x, y + size, x + size, RED, 0);
        }
    } else {
        int s2 = size / 2;
        printf("split %d %d (%d)\n", x, y, size);
        quad(x, y, s2, image);
        quad(x + s2, y, s2, image);
        quad(x, y + s2, s2, image);
        quad(x + s2, y + s2, s2, image);
    }
}

void compute_paths() {
    for (int i = 0; i < free_square_count; i++) {
        for (int j = i + 1; j < free_square_count; j++) {
            // printf("free_square %d %d (%d)\n", free_squares[i].x, free_squares[i].y, free_squares[i].size);
            int Ax = free_squares[i].x + free_squares[i].size / 2;
            int Ay = free_squares[i].y + free_squares[i].size / 2;
            int Bx = free_squares[j].x + free_squares[j].size / 2;
            int By = free_squares[j].y + free_squares[j].size / 2;

            bool has_collided = false;
            for (int k = 0; k < occupied_square_count; k++) {
                Square occupied_square = occupied_squares[k];
                int Rx = occupied_square.x, Ry = occupied_square.y;
                int Sx = Rx + occupied_square.size, Sy = Ry;
                int Tx = Rx, Ty = Ry + occupied_square.size;
                int Ux = Rx + occupied_square.size, Uy = Ry + occupied_square.size;

                int F[4];
                F[0] = (By - Ay) * Rx + (Ax - Bx) * Ry + (Bx * Ay - Ax * By);
                F[1] = (By - Ay) * Sx + (Ax - Bx) * Sy + (Bx * Ay - Ax * By);
                F[2] = (By - Ay) * Tx + (Ax - Bx) * Ty + (Bx * Ay - Ax * By);
                F[3] = (By - Ay) * Ux + (Ax - Bx) * Uy + (Bx * Ay - Ax * By);

                int pos = 0, neg = 0;
                for (int f = 0; f < 4; f++) {
                    if (F[f] >= MINIMUM_QUAD_SIZE) pos++;
                    else if (F[f] < -MINIMUM_QUAD_SIZE) neg++;
                }

                if (pos == 4 || neg == 4) continue;

                if (!((Ax > Ux && Bx > Ux) || (Ax < Rx && Bx < Rx) ||
                      (Ay > Uy && By > Uy) || (Ay < Ry && By < Ry))) {
                    has_collided = true;
                    break;
                }
            }

            if (!has_collided) {
                LCDLine(Ay, Ax, By, Bx, BLUE);

                // FIXME so we can't really use to smaller edge when sending to Astar, otherwise the bot will scratch wall
                if (free_squares[i].size >= MINIMUM_QUAD_FOR_EDGE) {
                    add_edge((Point){Ax, Ay}, (Point){Bx, By});
                    add_edge((Point){Bx, By}, (Point){Ax, Ay});
                }
            }
        }
    }
}

Point pbm_to_wld(Point pbm) {
    int wld_x = (int) (pbm.x * WORLD_SIZE / IMAGE_SIZE);
    int wld_y = (int) (pbm.y * WORLD_SIZE / IMAGE_SIZE);

    // return pbm;
    // return (Point){wld_x, wld_y};
    return (Point){wld_y, WORLD_SIZE - wld_x};
}

void move_node_to_index(Point target, int new_index) {
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].x == target.x && nodes[i].y == target.y) {
            if (i == new_index) return;

            // Swap nodes
            Point temp_node = nodes[new_index];
            nodes[new_index] = nodes[i];
            nodes[i] = temp_node;

            // Swap edges
            Point temp_edges[MAX_NEIGHBORS];
            memcpy(temp_edges, edges[new_index], sizeof(Point) * MAX_NEIGHBORS);
            memcpy(edges[new_index], edges[i], sizeof(Point) * MAX_NEIGHBORS);
            memcpy(edges[i], temp_edges, sizeof(Point) * MAX_NEIGHBORS);

            // Swap edge counts
            int temp_count = edge_count[new_index];
            edge_count[new_index] = edge_count[i];
            edge_count[i] = temp_count;

            break;
        }
    }
}

void export_nodes() {
    FILE *file = fopen("nodes.txt", "w");
    if (!file) {
        perror("Could not open nodes.txt");
        return;
    }

    move_node_to_index((Point){16, 16}, 0);         // Set start to always top left
    move_node_to_index((Point){112, 112}, node_count - 1); // Set end to always bottom right

    for (int i = 0; i < node_count; i++) {
        // Temporary array to mark seen neighbors
        bool seen[MAX_SQUARES] = {false};
        int neighbor_indices[MAX_NEIGHBORS];
        int neighbor_count = 0;

        for (int j = 0; j < edge_count[i]; j++) {
            for (int k = 0; k < node_count; k++) {
                if (edges[i][j].x == nodes[k].x &&
                    edges[i][j].y == nodes[k].y) {
                    if (!seen[k]) {
                        neighbor_indices[neighbor_count++] = k + 1;
                        seen[k] = true;
                    }
                    break;
                }
            }
        }

        fprintf(file, "%d %d", pbm_to_wld(nodes[i]).x, pbm_to_wld(nodes[i]).y);
        for (int n = 0; n < neighbor_count; n++) {
            fprintf(file, " %d", neighbor_indices[n]);
        }
        fprintf(file, "\n");
    }

    fclose(file);
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: quadtree map.pbm\n");
        return EXIT_FAILURE;
    }

    BYTE *image;
    read_pbm(argv[1], &image);
    if (!image) {
        fprintf(stderr, "Image loading failed. NULL pointer received.\n");
        return EXIT_FAILURE;
    }

    LCDImageStart(0, 0, IMAGE_SIZE, IMAGE_SIZE);
    LCDImageBinary(image);

    SIMSetRobot(0, INITIAL_X, INITIAL_Y, 0, INITIAL_ANGLE);
    LCDMenu("QUADTREE", "PATHS", "EXPORT", "RUN A*");

    while (1) {
        switch (KEYRead()) {
        case KEY1:
            quad(0, 0, IMAGE_SIZE, image);
            break;
        case KEY2:
            compute_paths();
            break;
        case KEY3:
            export_nodes();
            break;
        case KEY4:
            system("../lab4/astar.x nodes.txt");
            break;
        default:
            break;
        }
    }

    return EXIT_SUCCESS;
}
