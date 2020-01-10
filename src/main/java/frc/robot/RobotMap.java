/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// Motor Map
	// I = inverse, S = steer

	// Northwest Wheel
	public static int northwestS = 15;
	public static int northwest = 16;
	public static boolean northwestI = false;
	public static boolean northwestSI = false;

	// Northeast Wheel
	public static int northeastS = 3;
	public static int northeast = 4;
	public static boolean northeastI = true;
	public static boolean northeastSI = false;

	// Southeast Wheel
	public static int southeastS = 7;
	public static int southeast = 8;
	public static boolean southeastI = true;
	public static boolean southeastSI = false;

	// Southwest Wheel
	public static int southwestS = 11;
	public static int southwest = 12;
	public static boolean southwestI = false;
	public static boolean southwestSI = false;

	// SECOND SWERVE WHEELBASE

	// West Wheel
	public static int westS = 13;
	public static int west = 14;
	public static boolean westI = false;
	public static boolean westSI = false;

	// North Wheel
	public static int northS = 1;
	public static int north = 2;
	public static boolean northI = true;
	public static boolean northSI = false;

	// East wheel
	public static int eastS = 5;
	public static int east = 6;
	public static boolean eastI = true;
	public static boolean eastSI = false;

	// South Wheel

	public static int southS = 9;
	public static int south = 10;
	public static boolean southI = false;
	public static boolean southSI = false;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;

}
