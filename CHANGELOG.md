# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## 0.0.2 - 2018-11-25
### Changed
- Introduce boolean array to mark cells in openSet and closedSet.
- Change the structure of "cameFrom" from map to array.
- Remove use of closedSet (not storing actual cell object anymore).
- Change path coordinate class definition from RobotMap.hpp into CleaningRobot.hpp.

## 0.0.1 - 2018-11-24
### Added
- A basic structure to traverse all the cells in a given map.