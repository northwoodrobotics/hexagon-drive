/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.button.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static final XboxController driveController = new XboxController(0);
	public static final JoystickButton driveA = new JoystickButton(driveController, 1);
	public static final JoystickButton driveB = new JoystickButton(driveController, 2);
	public static final JoystickButton driveX = new JoystickButton(driveController, 3);
	public static final JoystickButton driveY = new JoystickButton(driveController, 4);
	public static final JoystickButton driveLTrigger = new JoystickButton(driveController, 5);
	public static final JoystickButton driveRTrigger = new JoystickButton(driveController, 6);


	public OI(Robot robot) {
		driveA.whenPressed(new ToggleFrontDirection(robot.drivetrain));
		driveX.whenPressed(new ToggleCentricMode(robot.drivetrain));
	}

	/**
	 * Adds a deadzone to, for example, a joystick input that does not completely
	 * zero itself mechanically.
	 * 
	 * @param input  the input value (any double between -1 and 1, inclusively).
	 * @param radius how far from zero the input can be for the output to still be
	 *               zero. This must be greated than 0 and less than 1.
	 * @return the value after application of the deadzone (between -1 and 1,
	 *         inclusively).
	 */
	public static double deadBand(double input) {
		double output;
		double radius = 0.2;
		assert (-1 <= input && input <= 1) : "input is less than -1 or greater than 1";
		assert (radius < 1) : "deadband radius is greater than or equal to the maximum output";

		if (input >= radius) {
			output = ((1 * (input - 1)) / (1 - radius)) + 1;
		} else if (input < -radius) {
			output = ((1 * (input + 1)) / (1 - radius)) - 1;
		} else {
			output = 0;
		}

		assert (Math.abs(output) <= 1) : "expected to output a smaller number than the 1 of " + 1;
		return output;
	}

	/**
	 * @return a double[] with the x output in [0] and the y output in [1]
	 */
	public static double[] circularExpDeadBand(double x, double y) {
		assert (-1 <= x && x <= 1) : "a is less than -1 or greater than 1";
		assert (-1 <= y && y <= 1) : "b is less than -1 or greater than 1";
		double[] output = { 0, 0 };

		double radius = 0.15;
		double exp = 1.5;
		double inputVectMag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
		if (inputVectMag > 1) {
			inputVectMag = 1;
		}

		if (inputVectMag > radius) {
			double outputVectMag = Math.pow(((inputVectMag - 1) / (1 - radius)) + 1, exp);
			if (x == 0) {
				output[0] = 0;
				output[1] = outputVectMag * Math.copySign(1.0, y);
			} else {
				output[0] = outputVectMag * Math.cos(Math.atan(y / x)) * Math.copySign(1.0, x);
				output[1] = outputVectMag * Math.sin(Math.atan(y / x)) * Math.copySign(1.0, x);
			}
		}

		assert (Math.abs(output[0]) <= 1) : "x output is too large! (expected less than 1; got " + output[0] + ")";
		assert (Math.abs(output[1]) <= 1) : "y output is too large! (expected less than 1; got " + output[1] + ")";
		return output;
	}
}
