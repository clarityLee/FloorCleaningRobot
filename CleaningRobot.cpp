#include <vector>
#include <chrono>
#include <functional>
#include "CleaningRobot.hpp"
using namespace std;

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

    filePath = string(argv[1]);
    ifstream readInFile(filePath + "/floor.data");
    if (!readInFile.is_open()) {
        errorFlag = true;
        errorMessage = "Unable to open \"" + filePath + "/floor.data\". Either file does not exist or wrong path.";
        return;
    }


    short rows = 0 , columns = 0;
    readInFile >> rows >> columns >> maxBattery;

    map = new RobotMap(rows, columns);
    map->readFloorData(readInFile);
    map->processData();

    finalPath.reserve(map->totalCells*3);
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
    Cell* recharger = map->recharger;
    Cell* next = nullptr;
    Cell* lastIncToR = nullptr;
    Cell* destination = nullptr;
    vector<Cell*> path;
    Cell*(RobotMap::*unvisitedAdj)(Cell*);

    if (map->edges > 498000) {
        unvisitedAdj = map->unvisitedAdj_fatest;
        cout << endl << "   switching to algorithm: unvisitedAdj_fatest" << endl;
    } else {
        unvisitedAdj = map->unvisitedAdj_v2;
        cout << endl << "   use default algorithm: unvisitedAdj_v2" << endl;
    }
    // std::function<Cell*(Cell*)> &unvisitedAdj = estTime > 20 ? map->unvisitedAdj_fatest : map->unvisitedAdj_v2;
    
    path.reserve(battery);

    recharger->visited = true;
    --unvisitedCount;
    while(unvisitedCount || current != recharger) {

        if (!current->visited) {
            current->visited = true;
            --unvisitedCount;
        }

        // 1. determine next movement
        // 1.1 robot is at recharger
        if (current == recharger) {
            battery = maxBattery;
            isGoingHome = false;

            path.clear();
            if (lastIncToR == nullptr) {
                path.push_back(map->randomStart());

            } else {
                map->findClosestUnvisitedToR(path, lastIncToR->index);
            }

            while(path.back() == current) {path.pop_back();}
            next = path.back();

        // 1.2 robot is not at recharger
        } else {

            // if robot has no goal now, robot is roaming
            if (next == nullptr) {

                // all the cells are visited
                if (unvisitedCount == 0)  setPathToRecharger = true;

                // still some cells not visited
                else {
                    // has unvisited neighbor?
                    next = (map->* unvisitedAdj)(current);
                    // next = map->unvisitedAdj_v2(current);
                    // next = map->unvisitedAdj_fatest(current);
                    

                    if (next) setPathToRecharger = !enoughBattery(next, battery-1);
                    else {
                        path.clear();
                        map->findClosestUnvisited(path, current);
                        
                        // check if battery is enough to go to destination
                        if (enoughBattery(path.front(), battery-path.size()+1)) {
                            while(path.back() == current) {path.pop_back();}
                            next = path.back();
                        } else setPathToRecharger = true;
                    }
                }
            }
        }

        if (setPathToRecharger) {
            map->randomShortestWayHome(path, current);
            while(path.back() == current) {path.pop_back();}
            next = path.back();
            isGoingHome = true;
            setPathToRecharger = false;
            lastIncToR = path[1];
        }

        // 2. move !
        if (next == map->recharger) lastIncToR = current;
        current = next;

        while(!path.empty() && path.back() == current) {path.pop_back();}
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
    return battery >= map->distToR[cell->index];
};

void CleaningRobot::test() {

    // map->calculateDistanceToR();
    // map->printAllDistance();

};