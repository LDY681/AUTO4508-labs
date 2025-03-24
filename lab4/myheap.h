#define SHRINK_RATIO 10
#include "eyebot.h"

// Min-heap for priority queue (A* open list)
typedef struct {
    int index;
    double cost;
} PriorityNode;

void swap(PriorityNode *a, PriorityNode *b) {
    PriorityNode temp = *a;
    *a = *b;
    *b = temp;
}

void heapify(PriorityNode heap[], int size, int i) {
    int smallest = i, left = 2 * i + 1, right = 2 * i + 2;
    if (left < size && heap[left].cost < heap[smallest].cost) smallest = left;
    if (right < size && heap[right].cost < heap[smallest].cost) smallest = right;
    if (smallest != i) {
        swap(&heap[i], &heap[smallest]);
        heapify(heap, size, smallest);
    }
}

int pop(PriorityNode heap[], int *size) {
    if (*size == 0) return -1;
    int index = heap[0].index;
    heap[0] = heap[--(*size)];
    heapify(heap, *size, 0);
    return index;
}

void push(PriorityNode heap[], int *size, int index, double cost) {
    int i = (*size)++;
    heap[i].index = index;
    heap[i].cost = cost;
    while (i > 0 && heap[(i - 1) / 2].cost > heap[i].cost) {
        swap(&heap[i], &heap[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

void LCDLineAlign(int prev_x, int prev_y, int x, int y, char color) {
    int lcd_x, lcd_y;
    LCDGetSize(&lcd_x, &lcd_y);
    prev_x = (prev_x / SHRINK_RATIO) + (lcd_x / 2);
    x = (x / SHRINK_RATIO) + (lcd_x / 2);
    prev_y = (lcd_y / 2) - (prev_y / SHRINK_RATIO);
    y = (lcd_y / 2) - (y / SHRINK_RATIO);
    LCDLine(prev_x, prev_y, x, y, color == 'r' ? RED : WHITE);
    LCDCircle(prev_x, prev_y, 5, color == 'r' ? RED : WHITE, 1);
    LCDCircle(x, y, 5, color == 'r' ? RED : WHITE, 1);
}