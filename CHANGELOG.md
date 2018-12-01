# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## 0.0.5 - 2018-12-01
## Added
- The Robot now can do multiple round of refinement within 25 secs up to 100 rounds.

## 0.0.4 - 2018-12-01
## Changed
- Set limit to path result buffer. Once the limitation is reached, append the data to a tmp file. The program will later convert the tmp file into a formal output result.
- CleaningRobot: Add analysis() function to CleaningRobot.hpp to show detail execution time.
- Robotmap: Use vector<Cell> instead of vector<Cell*> to re-arrange all cells in contiguous memory.
- Robotmap : Split infrequently used coordinate of cells to a separate container to reduce the size of vector<Cell>.
- Robotmap : combine original adjacent list (in vector form) into each cell (in array form).
- Robotmap : Introduce BFS algorithm to replace Dijkstra when finding closest unvisited cell when roaming.
- Robotmap : Introduce unvisitedAdj_min to find unvisited adjacent cell which prefer minimum distance to recharger to reduce fragment of the unvisited in the map.

## 0.0.3 - 2018-11-30
## Added
- When robot is roaming, dynamically choose different algorithm of finding unvisited adjacent cell according to total edges.
- Verify bot.
## Changed
- Dijkstra's data from any cell to reacharger incoming from any direction is now calaulte only once and cached for later use.
- When robot is at recharger, finding way to closest unvisited is replaced by reversing the result of finding way to recharger from closest unvisited.
- When robot is roaming, unvisited cell with distance to R equal or greater than current has high priority. Unvisited cell with adjacent visited cell that is different to current also as slight higher priority.
- When robot is roaming, if there is no adjacnt unvisited cell, before going to the next non-adjacent destination, the robot will check if the battery is enough. If the battery is not enough, it will go back to recharger instead.
- 5 set of unvisited cells with respect to recharger are stored in queue.
### Fixed
- Fixed the bug that RobotMap.cpp can only read square matrix. It now can read rectangular matrix.
- Fixed the bug that openSet is not work properly when deleting entry by using priority_queue instead of multiset.

## 0.0.2 - 2018-11-25
### Changed
- Introduce boolean array to mark cells in openSet and closedSet.
- Change the structure of "cameFrom" from map to array.
- Remove use of closedSet (not storing actual cell object anymore).
- Change path coordinate class definition from RobotMap.hpp into CleaningRobot.hpp.

## 0.0.1 - 2018-11-24
### Added
- A basic structure to traverse all the cells in a given map.