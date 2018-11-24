#include <iostream>
#include <chrono>
#include "CleaningRobot.hpp"
using namespace std;

int main(int argc, char* argv[]) {
    constexpr auto &&now = std::chrono::high_resolution_clock::now;

    auto programStartTime = now();

    CleaningRobot robot;
    cout << "Robot is starting to readFloorData() ....";
    auto phaseStartTime = now();
    robot.readFloorData(argc, argv);
    chrono::duration<double> elapsed = now() - phaseStartTime;
    cout << "completed in : " << elapsed.count() * 1000 << " ms." << endl;
    

    cout << "Robot is generating path ....";
    phaseStartTime = now();
    robot.clean();
    elapsed = now() - phaseStartTime;
    cout << "completed in : " << elapsed.count() * 1000 << " ms." << endl;
    

    cout << "Robot is saving path file.....";
    phaseStartTime = now();
    robot.outputPath(argc, argv);
    elapsed = now() - phaseStartTime;
    cout << "completed in : " << elapsed.count() * 1000 << " ms." << endl;

    elapsed = now() - programStartTime;
    cout << "Total used time :" << elapsed.count() * 1000 << " ms." << endl;
    
    return 0;
};