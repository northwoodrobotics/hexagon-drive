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

	// Frontleft
	public static int FrontleftS = 1;
	public static int Frontleft = 2;
	public static boolean FrontleftI = false;
	public static boolean FrontleftSI = false;

	// Rightfront
	public static int FrontrightS = 3;
	public static int Frontright = 4;
	public static boolean FrontrightI = true;
	public static boolean FrontrightSI = false;

	// Rightback
	public static int BackrightS = 5;
	public static int Backright = 6;
	public static boolean BackrightI = true;
	public static boolean BackrightSI = false;

	// Leftback
	public static int  BackleftS = 7;
	public static int Backleft = 8;
	public static boolean  BackleftI = false;
	public static boolean  BackleftSI = false;
	
	// Front
	public static int FrontS = 9;
	public static int Front = 10;
	public static boolean FrontI = true;
	public static boolean FrontSI = false;

	// Back
	public static int BackS = 11;
	public static int Back = 12;
	public static boolean BackI = false;
	public static boolean BackSI = false;



	

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;

}
