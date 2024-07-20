# 8BallGameAnalyzer

![GitHub repo size](https://img.shields.io/github/repo-size/Alessio240593/8BallGameAnalyzer)
![GitHub contributors](https://img.shields.io/github/contributors/Alessio240593/8BallGameAnalyzer)
![GitHub stars](https://img.shields.io/github/stars/Alessio240593/8BallGameAnalyzer?style=social)
![GitHub forks](https://img.shields.io/github/forks/Alessio240593/8BallGameAnalyzer?style=social)
![GitHub issues](https://img.shields.io/github/issues/Alessio240593/8BallGameAnalyzer)
![GitHub pull requests](https://img.shields.io/github/issues-pr/Alessio240593/8BallGameAnalyzer)
![GitHub](https://img.shields.io/github/license/Alessio240593/8BallGameAnalyzer)
![GitHub Workflow Status](https://img.shields.io/github/workflow/status/Alessio240593/8BallGameAnalyzer/CMake?label=build%20on%20Linux)
![C++](https://img.shields.io/badge/C++-11-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5.4-green)

## Description
8BallGameAnalyzer is a tool to analyze 8-ball pool games, providing insights into strategies and statistics to improve gameplay.

## Video Demo
![Demo](billard.gif)

## Features
- Track the trajectory (BETA version)
- Analyze ball position and patterns
- Provide strategic suggestions based on game state

## Installation

### Prerequisites
- [CMake](https://cmake.org/download/)
- [OpenCV](https://opencv.org/releases/)

### Build Instructions

1. Clone the repository
    ```bash
    git clone https://github.com/Alessio240593/8BallGameAnalyzer.git
    ```

2. Navigate to the project directory
    ```bash
    cd 8BallGameAnalyzer
    ```

3. Create a build directory and navigate into it
    ```bash
    mkdir build
    cd build
    ```

4. Configure the project with CMake, specifying the path to your OpenCV installation if necessary
    ```bash
    cmake ..
    ```

5. Build the project
    ```bash
    make
    ```

## Usage
Command line usage:
```cpp
    #include <iostream>
    #include <cstdlib>
    #include <fstream>
    #include "billiardAnalyzer.hpp"
    
    int main(int argc, char** argv)
    {
        try {
            billiardAnalyzer::EightBallPoolGame::analyzingEightBallPoolGame(argc, argv);
        } catch (const std::invalid_argument & e) {
            std::cerr << e.what() << std::endl;
            exit(EXIT_FAILURE);
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }
    
        std::exit(EXIT_SUCCESS);
    }
  ```
  ```bash
    ./main -path path/to/image_or_video -showTable true/false -showBalls true/false -showSegmentation true/false -showMinimap true/false
  ```

Usage in the cpp program:
  ```cpp
    #include <iostream>
    #include <cstdlib>
    #include <fstream>
    #include "billiardAnalyzer.hpp"
    
    int main(int argc, char** argv)
    {
        try {
            billiardAnalyzer::EightBallPoolGame::analyzingEightBallPoolGame("../Resources/videos/game2_clip2.mp4", true, true, false, true);
        } catch (const std::invalid_argument & e) {
            std::cerr << e.what() << std::endl;
            exit(EXIT_FAILURE);
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }
    
        std::exit(EXIT_SUCCESS);
    }
  ```

## Contributing
If you'd like to contribute to this project, please fork the repository and create a pull request. You can also open issues for feature requests or bug reports.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
