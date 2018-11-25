#include <vector>
#include <chrono>
#include "CleaningRobot.hpp"
using namespace std;
constexpr auto &&now = std::chrono::high_resolution_clock::now;

CleaningRobot::CleaningRobot() {};
CleaningRobot::~CleaningRobot() {
    if (map) delete map;
};

void CleaningRobot::readFloorData(int argc, char* argv[]) {
    if (argc < 2) {
        errorFlag = true;
        errorMessage = "Missing command argument. Please specify the path to floor.data.";
        return;
    }

    ifstream readInFile(string(argv[1]) + "/floor.data");
    if (!readInFile.is_open()) {
        errorFlag = true;
        errorMessage = "Unable to open \"" + string(argv[1]) + "/floor.data\". Either file does not exist or wrong path.";
        return;
    }

    short rows = 0 , columns = 0;
    readInFile >> rows >> columns >> maxBattery;

    map = new RobotMap(rows, columns);
    map->readFloorData(readInFile);
    map->processData();

    finalPath.reserve(map->totalCells*2);
}; 

bool CleaningRobot::hasError() {
    return errorFlag;
};

int CleaningRobot::printError() {
    cout << "Error: " << errorMessage << endl;
    return 1;
};

void CleaningRobot::clean() {

    int unvisitedCount = map->totalCells;
    bool isGoingHome = false;
    bool setPathToRecharger = false;
    int battery = 0;
    Cell* current = map->recharger;
    Cell* next = nullptr;
    Cell* lastIncToR = nullptr;
    Cell* destination = nullptr;
    vector<Cell*> path;

    while(current != map->recharger || unvisitedCount) {

        if (!current->visited) {
            current->visited = true;
            --unvisitedCount;
        }

        // 1. determine next movement
        // 1.1 robot is at recharger
        if (current == map->recharger) {
            battery = maxBattery;
            isGoingHome = false;

            path.clear();
            if (lastIncToR == nullptr) {
                path.push_back(map->randomStart());
            } else {
                map->findClosestUnvisited(path, map->recharger, lastIncToR);
            }

            if (path.back() == current) path.pop_back();
            next = path.back();

        // 1.2 robot is not at recharger
        } else {

            // TODO: need to check battery for current step to reduce some process.

            if (next != nullptr) {

                // robot is on the way to the closest unvisited cell
                if (!isGoingHome) {
                // (using normal dijkstra distance)
                    setPathToRecharger = !enoughBattery(next, battery-1);
                
                // robot is homing
                } else {
                    if (next == map->recharger) {
                        if (lastIncToR != current) {
                            lastIncToR = current;
                        }
                    }			
                }

            // robot has no goal now, robot is roaming
            } else {
                
                // all the cells are visited
                if (unvisitedCount == 0) {
                    setPathToRecharger = true;

                // still some cells not visited
                } else {
                    // has unvisited neighbor?
                    next = map->unvisitedAdjacent(current);
                    if (next) {
                        setPathToRecharger = !enoughBattery(next, battery-1);
                    
                    } else {
                        path.clear();
                        map->findClosestUnvisited(path, current);
                        // map->printPath(path);
                        if (path.back() == current) path.pop_back();
                        next = path.back();
                        setPathToRecharger = !enoughBattery(next, battery-1);
                    }
                }
            }
        } 

        if (setPathToRecharger) {
            map->randomShortestWayHome(path, current);
            if (path.back() == current) path.pop_back();
            next = path.back();
            isGoingHome = true;
            setPathToRecharger = false;
        }

        // 2. move !
        if (next == map->recharger) lastIncToR = current;
        current = next;
        if (path.back() == current) path.pop_back();
        if (path.empty()) {
            next = nullptr;
        } else {
            next = path.back();
        }
        
        // record current cell into final path.
        finalPath.emplace_back(current->i, current->j);
        
        ++usingSteps;
        --battery;
    }
};

void CleaningRobot::outputPath(int argc, char* argv[]) {
    ofstream o(string(argv[1]) + "/final.path");
    o << usingSteps << endl;

    // vector<Cell*> &cells = map->cells->cells;
    // for (int k = 0 ; k < pathIndices.size() ; ++k) {
    //     o << cells[pathIndices[k]]->i << ' ' << cells[pathIndices[k]]->j << endl;
    // }

    for (int k = 0 ; k < finalPath.size() ; ++ k) {
        o << finalPath[k].i << ' ' << finalPath[k].j << endl;
    }
};

inline bool CleaningRobot::enoughBattery(Cell* cell, int battery) {
    return battery >= cell->distance;
};

void CleaningRobot::test() {

    // map->calculateDistanceToR();
    map->printAllDistance();

};