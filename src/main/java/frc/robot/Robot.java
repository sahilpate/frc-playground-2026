// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.SwerveDrive;
import frc.robot.systems.SwerveModule;
import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;

import edu.wpi.first.wpilibj.XboxController;

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
        // swerve.stopModules();
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        // swerve.stopModules();
		// swerve.setWheelsToAngle(0, controller.getLeftY() * 0.8);

		double xSpeed = controller.getLeftX() * (DriveConsts.maxMetersPerSecToMotorSpeed / 10);
		double ySpeed = controller.getLeftY() * (DriveConsts.maxMetersPerSecToMotorSpeed / 10);
		double rotSpeed = controller.getRightX() * (DriveConsts.maxRadPerSecToMotorSpeed / 5);

		swerve.setModules(ySpeed, xSpeed, rotSpeed);
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
