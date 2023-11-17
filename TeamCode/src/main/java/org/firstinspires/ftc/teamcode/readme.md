# FTC 19342 - Clever Core (RO103) - TEAM CODE for season #8 CenterStage
# ALL RIGHTS RESERVED (tho you can use our code as inspiration <3)

Implements:
* DifferentActivities: Activity programs, made specifically for a custom robotized system (NOT USED IN THE ACTUAL COMPETITION)

* Localizer: three type of localizers:
        - custom Threaded IMU (used only for orientation in space, not for travelled distance); was developed for testing field centric drive (UNSTABLE)
        - custom 3 wheel odometry localizier. Still testing..
        - roadrunner implementation for 3 wheel odometry; comparing the performance with the custom one

* OpModes: used in the actual competition

* RR: roadrunner util

* Swerve:
        -custom swerve module hardware (MODULES / DRIVETRAIN); also we modified roadrunner's path following ``DriveSignal`` to be compatible with the swerve 
        -swerve 1st order kinematics; working on figuring out how to implement 2nd order to limit skew

* TestOpModes: pretty self explanatory

* Util: util classes (all custom):
        -motor util (initialization, pid, resetting encoders etc.)
        -Point/Pose classes used in the custom odometry; needs ``Transformations`` to be converted in roadrunner parameters
        -``Transformations`` class; needed to do all types of conversions (ex: our 'Pose' to roadrunner's 'Pose2d')
        -Bulking Cache implementation trough ``PhotonCore`` class; Still testing...
