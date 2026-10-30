//===========================================================================
//  main.cpp  --  entry point.
//
//  Deliberately thin: parse arguments, construct the Application, run it.
//  All wiring lives in Application so the startup path is testable and the
//  headless mode uses exactly the same construction sequence as the windowed
//  mode.
//===========================================================================

#include <exception>
#include <iostream>

#include "application/Application.h"
#include "config/Config.h"

int main(int argc, char** argv) {
    // Record argv[0] so config/ and assets/ can be found relative to the
    // executable as well as relative to the working directory.
    vsim::setExecutablePath(argc > 0 ? argv[0] : nullptr);

    vsim::LaunchOptions options;
    if (!vsim::parseCommandLine(argc, argv, options)) {
        return 0;   // --help, or a reported argument error
    }

    try {
        vsim::Application app;
        if (!app.initialize(options)) {
            std::cerr << "[main] initialisation failed\n";
            return 1;
        }
        return app.run();
    } catch (const std::exception& e) {
        std::cerr << "[main] fatal: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[main] fatal: unknown exception\n";
        return 1;
    }
}
