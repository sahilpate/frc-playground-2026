// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.SwerveDrive;

public class Robot extends TimedRobot {
    XboxController controller = new XboxController(0);
    SwerveDrive swerve = new SwerveDrive();

    public Robot() {}

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {
        swerve.stopModules();
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        swerve.stopModules();
    }
}

/*
    Unused funcion headers
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
*/
