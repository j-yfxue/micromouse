#include "solver.h"
#include "queues.h"
#include "constants.h"
#include <stdio.h>
#include "IR_driver.h"
#include "main.h"
#include <stdlib.h>

/* Initialize starting values */

// define starting position
unsigned char target = STARTING_TARGET; // initially 1 since going to center, 0 going to start
coord currentXY = {STARTING_X, STARTING_Y}; // coordinate position of the mouse
Heading currentHeading = STARTING_HEADING; //Potentially initialized to North

//NEW
int originalFloodfill[MAZE_WIDTH][MAZE_HEIGHT];  // Stores initial floodfill values
int returning;
/* Arrays and Array Helper Functions */

// keeps track of the known vertical walls in the maze
int verticalWalls[MAZE_WIDTH+1][MAZE_HEIGHT] = {{0}};
// keeps track of horizontal walls in the maze
int horizontalWalls[MAZE_WIDTH][MAZE_HEIGHT+1] = {{0}};
// keeps track of current floodfill values
int floodArray[MAZE_WIDTH][MAZE_HEIGHT];
// keeps track of the path the car should take after a floodfill iteration for each cell of the maze
Heading pathArray[MAZE_WIDTH][MAZE_HEIGHT] = {{NORTH}};
// keeps track of all of the cells that the mouse has visited
int travelArray[MAZE_WIDTH][MAZE_HEIGHT] = {{0}};

// given a coord, checks to see if the mouse has visited a certain cell before
int checkTravelArray(coord c) 
{
    return travelArray[c.x][c.y];
}
// given a coord, updates the travel array to mark that the mouse has visited that cell before
void updateTravelArray(coord c) 
{
    travelArray[c.x][c.y] = 1;
}
// given coordinate, updates the respective cell's floodfill value
void updateFloodArray(coord c, int val) 
{
    floodArray[c.x][c.y] = val;
}
// given coordinate, gets the respective cell's floodfill value
int getFloodArray(coord c) 
{

    return floodArray[c.x][c.y];
}
// given coordinate, updates the respective cell's path heading
void updatePathArray(coord c, Heading h) 
{
    pathArray[c.x][c.y] = h;
}
// given cordinate, gets the respective cell's path heading
Heading getPathArray(coord c) 
{

    return pathArray[c.x][c.y];
    
}
/* Floodfill Functions */

// resets the floodfill array to target the center as destination
void resetFloodArray()
{
    // set the entire flood array to blank values (-1)
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            if (x == LOWER_X_GOAL && y == UPPER_Y_GOAL && target == 1)
            {
                floodArray[x][y] = 0;
            } else if (x == STARTING_X && y == STARTING_Y && !target)
            {
                floodArray[x][y] = 0; 
            } else {
                floodArray[x][y] = -1;
            }

            
            
            
        }
    }
    //Following determines the whether to set the target or the starting point to 0

    // set desired goal values 
    // if (target) // target is goal (center)
    //     for (int x = LOWER_X_GOAL; x <= UPPER_X_GOAL; x++)
    //         for (int y = LOWER_Y_GOAL; y <= UPPER_Y_GOAL; y++)
    //             floodArray[x][y] = 0;
    // else // target is starting cell
    //     floodArray[STARTING_X][STARTING_Y] = 0;
}
/******************************************************** */
// given heading and coordinate, check if there is a wall on that side of the cell
int checkWall(Heading heading, coord c) {
    switch (heading) {
        case NORTH: return horizontalWalls[c.x][c.y+1]; //tracking the uppermost cells
        case WEST: return verticalWalls[c.x][c.y];
        case SOUTH: return horizontalWalls[c.x][c.y];
        case EAST: return verticalWalls[c.x+1][c.y]; // tracking the rightmost walls
    }
}

//advancing locations
// Increments coord in the direction of the heading by input integer, then returns updated coord
coord incrementCoord(Heading heading, coord c, int n)
{
    switch (heading) {
        case NORTH: return (coord){ c.x       , c.y + n };
        case WEST : return (coord){ c.x - n   , c.y     };
        case SOUTH: return (coord){ c.x       , c.y - n };
        case EAST : return (coord){ c.x + n   , c.y     };
    }
}

// turns currentHeading global variable to the left based on the mouse's current heading,
// then returns LEFT action
Action API_turnLeft() {
    currentHeading = (currentHeading+1)%4;
    return LEFT;
}

// turns currentHeading global variable to the right based on the mouse's current heading,
// then returns RIGHT action
Action API_turnRight() {
    currentHeading = (currentHeading-1)%4;
    return RIGHT;
}

// returns whether the mouse is in the target
unsigned char mouseInGoal() {
    return (target == 1 &&
            currentXY.x == LOWER_X_GOAL &&
            currentXY.y == LOWER_Y_GOAL);
}

// given heading and coordinates, returns the floodfill value of the corresponding neighbor cell.
// if the neighbor is off of the maze (argument cell is on the boundary of the maze), return -2
int getNeighbor(Heading heading, coord c)
{
    switch (heading) {
        case NORTH:
            if (c.y >= MAZE_HEIGHT-1) return OUT_OF_BOUNDS;
            else return floodArray[c.x][c.y+1];
        case WEST:
            if (c.x <= 0) return OUT_OF_BOUNDS;
            else return floodArray[c.x-1][c.y];
        case SOUTH:
            if (c.y <= 0) return OUT_OF_BOUNDS;
            else return floodArray[c.x][c.y-1];
        case EAST:
            if (c.x >= MAZE_WIDTH-1) return OUT_OF_BOUNDS;
            else return floodArray[c.x+1][c.y];
    }
}

void generateNeighbor(queue q, neighbor current, int currentVal) {
    //if there is no wall in the given direction
    //check wall checks whether a wall is blocking movement
 
 
    /*
    this is used for floodfill when calculating the costs of the cells
    algorithm starts at the target point
 
    1. need to check if the wall exists in the heading provided given the current direction
 
    2. if the wall does not exist then based on the value of the current position, increment the
    value of the cell cost for the neighbor
    */
 
    static const Heading directions[4] = {NORTH, EAST, SOUTH, WEST};
 
    for (int i = 0; i < 4; i++) {
        Heading heading = i;
        if (!checkWall(heading, current.coord)){
 
            coord nextCoord = incrementCoord(heading, current.coord, 1);
            int nextVal = currentVal + TILE_SCORE;
            int neighborVal = getNeighbor(heading, current.coord);
 
            if (neighborVal == NOT_YET_SET || nextVal < neighborVal) {
                neighbor n;
                n.coord = nextCoord;
                n.heading = heading;
                queue_push(q, n);
                updateFloodArray(nextCoord, nextVal);
                updatePathArray(nextCoord, (heading + 2) % 4);
            }
        }
    }
   
 
 
    
}

// updates the floodfill array based on known walls
void floodFill() {

    // set non-goal values to blank so that the floodfill array can be recalculated
    resetFloodArray();

    // declare/initialize relevant variables for queue for floodfill algorithm
    queue q = queue_create();

    // iterate through the 2D array, find goal values and add them to the queue
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            if (floodArray[x][y] == 0) {
                // for the starting goal values, it doesn't matter which direction you approach them from.
                // as such, they should be oriented from all directions
                queue_push(q,(neighbor){(coord){x,y},NORTH,0});
                queue_push(q,(neighbor){(coord){x,y},WEST,0});
                queue_push(q,(neighbor){(coord){x,y},SOUTH,0});
                queue_push(q,(neighbor){(coord){x,y},EAST,0});
            }
        }
    }

    // adds available neighbors to queue and updates their floodfill values
    while (!queue_is_empty(q)) {
        // initializes values for calculating floodfills for neighbors
        neighbor current = queue_pop(q);
        int currentVal = getFloodArray(current.coord);

        // prints the current cell's floodfill number to the simulation screen
     
        generateNeighbor(q,current,currentVal);       
    }
}

// places a wall in respective arrays and API at the given heading and coordinate
void placeWall(Heading heading, coord c) {
    // sets a wall in the wall arrays
    switch (heading) {
        case NORTH:
            horizontalWalls[c.x][c.y+1] = 1;
            return;
        case WEST:
            verticalWalls[c.x][c.y] = 1;
            return;
        case SOUTH:
            horizontalWalls[c.x][c.y] = 1;
            return;
        case EAST:
            verticalWalls[c.x+1][c.y] = 1;
            return;
    }
}

void generateInitialWalls() {
    //placing walls along the southern and northern edges for each column
    for (int x = 0; x < MAZE_WIDTH; x++) {
        placeWall(SOUTH,(coord){x,0});
        placeWall(NORTH,(coord){x,MAZE_HEIGHT-1});
    }
    //plaicing walls along the western and eastern edges for each row
    for (int y = 0; y < MAZE_HEIGHT; y++) {
        placeWall(WEST,(coord){0,y});
        placeWall(EAST,(coord){MAZE_WIDTH-1,y});
    }
}

// checks for and then updates the walls for the current cell
//WILL NEED TO READ IN SENSOR DATA HERE
void updateWalls()
{
    // based on the current heading, places walls at the respective locations
    //can potentially change the API functions with our own for final implementation
    if (wallFront()) placeWall(currentHeading,currentXY);
    if (wallLeft()) placeWall((currentHeading+1)%4,currentXY);
    if (wallRight()) placeWall((currentHeading-1)%4,currentXY);
}

// based on updated wall and floodfill information, return the next action that the mouse should do
Action nextAction() {
    // 1) If we are already in the goal and we want to stay there, do nothing
    if (mouseInGoal()) {
        return IDLE;
    }

    // 2) Determine the heading we *want* to move toward from flood-fill (path array)
    Heading desiredHeading = getPathArray(currentXY);

    // 3) If our current heading is already correct, move forward one cell if no wall
    if (currentHeading == desiredHeading) {
        if (!checkWall(desiredHeading, currentXY)) {
            // Move forward one cell
            // Update our internal position & mark it visited
            currentXY = incrementCoord(desiredHeading, currentXY, 1);
            updateTravelArray(currentXY);
            return FORWARD;
        } else {
            // There is a wall in front even though the flood-fill suggests forward.
            // For a simple fallback, just turn left. (You can decide any fallback you like.)
            return API_turnLeft();
        }
    }

    // 4) Otherwise, we need to turn until we face the desired heading.

    // Turning left: currentHeading + 1 == desiredHeading (mod 4).
    if ((currentHeading + 1) % 4 == desiredHeading) {
        return API_turnLeft();
    }
    // Turning right: currentHeading - 1 == desiredHeading (mod 4), 
    // or equivalently currentHeading + 3 == desiredHeading (mod 4).
    if ((currentHeading + 3) % 4 == desiredHeading) {
        return API_turnRight();
    }

    // If the desired heading is 180 degrees behind us, just do two lefts (or two rights).
    API_turnLeft();
    API_turnLeft();
    return TURNAROUND;
}


// checks if the mouse has reached its target
void checkDestination()
{
    
    if (target) {
        if (mouseInGoal()) {
            if (RESET_AT_CENTER) {
                currentXY = (coord){0,0};
                currentHeading = NORTH;
            }
            else if (!STAY_AT_CENTER) {
                target = 0;       
                //returning = 1;
            }
        }
    } else if (currentXY.x == STARTING_X && currentXY.y == STARTING_Y) {
        target = 1;
    }
        //returning = 0;
    
}


// sends the mouse's recommended next action back to main
Action solver() {
    checkDestination();
    updateWalls();    
    floodFill();
    return nextAction();
}
