#include <iostream>
#include <cstdlib>
#include <fstream>
#include "billiardAnalyzer.hpp"

int main(int argc, char** argv)
{
    try {
        billiardAnalyzer::EightBallPoolGame::analyzingEightBallPoolGame("../Resources/videos/game1_clip2.mp4",
                                                                        true,
                                                                        true,
                                                                        false,
                                                                        true);
    } catch (const std::invalid_argument & e) {
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    std::exit(EXIT_SUCCESS);
}