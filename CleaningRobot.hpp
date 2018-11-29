#include <iostream>
#include <fstream>
#include <string>
#include "RobotMap.hpp"
using namespace std;

class CleaningRobot {
public:
    CleaningRobot();
    ~CleaningRobot();
    bool hasError();
    int printError();
    void readFloorData(int argc, char* argv[]);
    void clean();
    void outputPath(int argc, char* argv[]);
    void test();
private:
    bool errorFlag = false;
    int maxBattery = 0;
    int usingSteps = 0;
    string errorMessage;
    RobotMap* map;
    string filePath;
    int estTime = 0;

    class coordinate {
    public:
        coordinate(short _i, short _j) : i(_i), j(_j) {};
        short i, j;
    };
    vector<coordinate> finalPath;
    
    inline bool enoughBattery(Cell* cell, int battery);
};