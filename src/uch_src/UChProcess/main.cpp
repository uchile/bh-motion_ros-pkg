#include "process.h"

int main(int argc, char *argv[])
{
    std::cout << "antes robotProcess " << std::endl;
    process robotProcess;
    robotProcess.cognitionThread.join();
    robotProcess.motionThread.join();
    std::cout << "despues robotProcess " << std::endl;
};

