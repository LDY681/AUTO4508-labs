#include "eyebot.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

char *FILENAME = "nodes.txt";
#define MAX_NODES 20
#define LINE_SIZE 100

typedef struct
{
    int x;                    // x position
    int y;                    // y position
    int num_neighbors;        // number of neighbors
    int neighbors[MAX_NODES]; // list of neighbors (node number)
} Node;

double euclidean_distance(int dx, int dy) {
    return sqrt(dx * dx + dy * dy);
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage: ./astar.x filename.txt \n");
        return EXIT_FAILURE;
    }
    FILENAME = argv[1];

    LCDMenu("Task1", "Task2", "Task3", "END");

    while (1)
    {
        int key = KEYRead();
        if (key == KEY1)
        {
            FILE *file = fopen(FILENAME, "r");
            if (!file) {
                perror("Failed to open file");
                return EXIT_FAILURE;
            }

            Node nodes[MAX_NODES];
            int num_nodes = 0;
            char line[LINE_SIZE];

            while (num_nodes < MAX_NODES && fgets(line, sizeof(line), file))
            {
                char *token;
                int count = 0;

                // Read node's x and y coordinates
                token = strtok(line, " ");
                if (!token) continue;
                nodes[num_nodes].x = atoi(token);

                token = strtok(NULL, " ");
                if (!token) continue;
                nodes[num_nodes].y = atoi(token);

                // Read node's neighbors (need to deduct 1 to convert to 0-based index)
                while ((token = strtok(NULL, " ")) && count < MAX_NODES - 1)
                {
                    int neighbor = atoi(token) - 1;
                    if (neighbor >= 0 && neighbor < MAX_NODES) {
                        nodes[num_nodes].neighbors[count++] = neighbor;
                    }
                }

                nodes[num_nodes].num_neighbors = count;
                num_nodes++;
            }
            fclose(file);

            // Print Euclidean Distance Matrix
            printf("\nConnection Matrix:\n");
            for (int i = 0; i < num_nodes; i++)
            {
                for (int j = 0; j < num_nodes; j++)
                {
                    int is_connected = 0;
                    for (int k = 0; k < nodes[i].num_neighbors; k++)
                    {
                        if (nodes[i].neighbors[k] == j)
                        {
                            is_connected = 1;
                            break;
                        }
                    }

                    if (i == j)
                        printf("0.0 ");
                    else if (is_connected)
                        printf("%.1f ", euclidean_distance(nodes[i].x - nodes[j].x, nodes[i].y - nodes[j].y));
                    else
                        printf("-1.0 ");
                }
                printf("\n");
            }
        }
        else if (key == KEY2)
        {
            // A-star algorithm
            // TODO read page 123
        }
        else if (key == KEY3)
        {
        }
        else if (key == KEY4)
        {
            break;
        }
        else
        {
            OSWait(100);
        }
    }
    return 0;
}
