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
    void refine();
    void outputPath();
    void cleanTmpFile();
    void test();
    void analysis();
    int totalSteps();
private:
    bool isRefine = false;
    bool errorFlag = false;
    int maxBattery = 0;
    int usingSteps = 0;
    string errorMessage;
    RobotMap* rmap;
    string filePath;

    double sAtRecharger = 0;
    double sDoingPath = 0;
    double sRoaming = 0;
    double sGoingAdj = 0;
    double sGoingNoneAdj = 0;
    double sElse = 0;
    int cFCUTR = 0;
    int noUnvClosestCount = 0;
    int unvClosestCount = 0;
    vector<int> finalPath;

    void saveTmp();
    void resetForRefine();
    inline bool enoughBattery(Cell* cell, int battery);
    inline bool enoughBattery(int cellIndex, int battery);
};