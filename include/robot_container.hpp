#pragma once

#include "vex.h"
#include "superstructure/superstructure.hpp"
#include "neblib/xdrive.hpp"

namespace neblib {
    class RobotContainer {
    public:   /// PUBLIC ///

        RobotContainer(uint32_t& robotTime);

        void periodic();

    private:   /// PRIVATE ///

        Superstructure superstructure;
        XDrive xDrive;

        vex::controller driverController;

        void configureButtonBindings();
    };
}