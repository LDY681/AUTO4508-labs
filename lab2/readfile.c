#include <stdio.h>

int main() {
    FILE* file = fopen("way.txt", "r");
    int points[20][2];
    int i = 0;
    while (fscanf(file,"%d %d", &points[i][0], &points[i][1]) == 2) {
    	printf("x:%d y:%d\n", points[i][0], points[i][1]);
	    i++;
	}
    fclose(file);
}