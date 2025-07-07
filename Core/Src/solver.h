#ifndef SOLVER_H
#define SOLVER_H

typedef enum Heading {NORTH, WEST, SOUTH, EAST} Heading;
typedef enum Action {LEFT, FORWARD, RIGHT, IDLE, TURNAROUND} Action;

typedef struct coord {
    int x;
    int y;
} coord;

typedef struct neighbor {
    coord coord;
    Heading heading;
    int streak;
} neighbor;

Action solver();
void generateInitialWalls();

#endif
