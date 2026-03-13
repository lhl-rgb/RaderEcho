#include "RadarSimulator.h"
#include "Logger.h"
#include <iostream>
#include <string>
#include <csignal>

static PhasedArrayRadarSimulator* g_simulator = nullptr;

void signalHandler(int signum) {
    LOG_WARN("Interrupt signal received: " + std::to_string(signum));
    if (g_simulator) {
        g_simulator->stop();
    }
}

void printWelcome() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Phased Array Radar Echo Simulator" << std::endl;
    std::cout << "  Version 3.0" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    printWelcome();

    Logger::getInstance().setLogFile("../out/logs/simulation.log");
    Logger::getInstance().setLevel(Logger::Level::INFO);

    std::string configPath = "../config/simulation_config.txt";
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [config_file]" << std::endl;
            return 0;
        }
        configPath = arg;
    }

    LOG_INFO("Starting Phased Array Radar Echo Simulator V3.0");
    LOG_INFO("Config file: " + configPath);

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    PhasedArrayRadarSimulator simulator;
    g_simulator = &simulator;

    LOG_INFO("Initializing simulator...");
    if (!simulator.initialize(configPath)) {
        LOG_ERROR("Failed to initialize simulator");
        std::cerr << "Error: Failed to initialize simulator" << std::endl;
        return 1;
    }

    std::cout << "Simulator initialized successfully!" << std::endl;
    std::cout << "  - Radar frequency: " << simulator.getRadarParams().frequency / 1e9 << " GHz" << std::endl;
    std::cout << "  - Bandwidth: " << simulator.getRadarParams().bandwidth / 1e6 << " MHz" << std::endl;
    std::cout << "  - PRF: " << simulator.getRadarParams().prf << " Hz" << std::endl;
    std::cout << "  - Max range: " << simulator.getRadarParams().maxRange / 1e3 << " km" << std::endl;
    std::cout << "  - Pulses per CPI: " << simulator.getRadarParams().numPulsesPerCPI << std::endl;
    std::cout << std::endl;

    TargetParams target1;
    target1.id = 1;
    target1.position = Position3D(10000, 0, 0);
    target1.velocity = Velocity3D(-50, 0, 0);
    target1.rcs_mean = 10.0;
    target1.swerlingModel = SwerlingModel::Swerling1;
    target1.motionModel = MotionModel::ConstantVelocity;
    simulator.addTarget(target1);
    std::cout << "Added target 1 at 10km, moving at -50 m/s" << std::endl;

    TargetParams target2;
    target2.id = 2;
    target2.position = Position3D(15000, 5000, 0);
    target2.velocity = Velocity3D(0, -30, 0);
    target2.rcs_mean = 100.0;
    target2.swerlingModel = SwerlingModel::Swerling3;
    target2.motionModel = MotionModel::ConstantVelocity;
    simulator.addTarget(target2);
    std::cout << "Added target 2 at 15.8km, moving at -30 m/s" << std::endl;
    std::cout << std::endl;

    std::cout << "Starting full simulation..." << std::endl;
    simulator.runFullSimulation();

    std::cout << "Simulation completed!" << std::endl;
    LOG_INFO("Simulation completed");

    g_simulator = nullptr;

    return 0;
}
