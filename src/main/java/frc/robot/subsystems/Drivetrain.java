/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


/*            --- LAYOUT ---
*
*                 Front
*      Wheel 2 ------------ Wheel 1    ---
*          |                 |           |
*          |                 |           |
*          |                 |           |
*    Left  |                 |  Right    |-- Length
*          |                 |           |
*          |                 |           |
*          |                 |           |
*      Wheel 3 ------------ Wheel 4    ---
*                 Back
*
*          |                 |
*          |----- Width -----|
*/


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import frc.robot.swerve.drive.SwerveDrive;
import frc.robot.swerve.math.CentricMode;
import frc.robot.RobotMap;
import frc.robot.commands.teleop.TeleDrive;
import frc.robot.swerve.drive.CanTalonSwerveEnclosure;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Drivetrain extends Subsystem {

	//Default
	private CanTalonSwerveEnclosure swerveEnclosureNorthwest;
	private CanTalonSwerveEnclosure swerveEnclosureNortheast;
	private CanTalonSwerveEnclosure swerveEnclosureSoutheast;
	private CanTalonSwerveEnclosure swerveEnclosureSouthwest;
	
	//Plus
	private CanTalonSwerveEnclosure swerveEnclosureWest;
	private CanTalonSwerveEnclosure swerveEnclosureNorth;
	private CanTalonSwerveEnclosure swerveEnclosureEast;
	private CanTalonSwerveEnclosure swerveEnclosureSouth;
	private SwerveDrive swerveDrive;

	public static final double GEAR_RATIO = (1024d);
	private static final double L_Default = 35;
	private static final double W_Default = 35;
	private static final double L_Plus = 35;
	private static final double W_Plus = 35;

	private static final double P = 10.0;
	private static final double I = 0.0;
	private static final double D = 0.0;
	private static final double F = 0.0;

	private WPI_TalonSRX driveMotorNorthwest;
	private WPI_TalonSRX driveMotorNortheast;
	private WPI_TalonSRX driveMotorSoutheast;
	private WPI_TalonSRX driveMotorSouthwest;

	private WPI_TalonSRX driveMotorWest;
	private WPI_TalonSRX driveMotorNorth;
	private WPI_TalonSRX driveMotorEast;
	private WPI_TalonSRX driveMotorSouth;

	private WPI_TalonSRX steerMotorNorthwest;
	private WPI_TalonSRX steerMotorNortheast;
	private WPI_TalonSRX steerMotorSoutheast;
	private WPI_TalonSRX steerMotorSouthwest;

	private WPI_TalonSRX steerMotorWest;
	private WPI_TalonSRX steerMotorNorth;
	private WPI_TalonSRX steerMotorEast;
	private WPI_TalonSRX steerMotorSouth;

	double[] wheelAngles = new double[8];

	private Gyro gyro = new ADXRS450_Gyro();
	private CentricMode centricMode = CentricMode.ROBOT;
	private boolean SouthIsFront = false;

	public Drivetrain() {

	}

	public void init() {
		
		driveMotorNorthwest = new WPI_TalonSRX(RobotMap.northwest);
		driveMotorNortheast = new WPI_TalonSRX(RobotMap.northeast);
		driveMotorSoutheast = new WPI_TalonSRX(RobotMap.southeast);
		driveMotorSouthwest = new WPI_TalonSRX(RobotMap.southwest);
		driveMotorWest = new WPI_TalonSRX(RobotMap.west);
		driveMotorNorth = new WPI_TalonSRX(RobotMap.north);
		driveMotorEast = new WPI_TalonSRX(RobotMap.east);
		driveMotorSouth = new WPI_TalonSRX(RobotMap.south);

		driveMotorNorthwest.setInverted(RobotMap.northwestI);
		driveMotorNortheast.setInverted(RobotMap.northwestI);
		driveMotorSoutheast.setInverted(RobotMap.southeastI);
		driveMotorSouthwest.setInverted(RobotMap.southwestI);
		driveMotorWest.setInverted(RobotMap.westI);
		driveMotorNorth.setInverted(RobotMap.northI);
		driveMotorEast.setInverted(RobotMap.eastI);
		driveMotorSouth.setInverted(RobotMap.southI);

		driveMotorNorthwest.setNeutralMode(NeutralMode.Brake);
		driveMotorNortheast.setNeutralMode(NeutralMode.Brake);
		driveMotorSoutheast.setNeutralMode(NeutralMode.Brake);
		driveMotorSouthwest.setNeutralMode(NeutralMode.Brake);
		driveMotorWest.setNeutralMode(NeutralMode.Brake);
		driveMotorNorth.setNeutralMode(NeutralMode.Brake);
		driveMotorEast.setNeutralMode(NeutralMode.Brake);
		driveMotorSouth.setNeutralMode(NeutralMode.Brake);

		steerMotorNorthwest = new WPI_TalonSRX(RobotMap.northwestS);
		steerMotorNortheast = new WPI_TalonSRX(RobotMap.northeastS);
		steerMotorSoutheast = new WPI_TalonSRX(RobotMap.southeastS);
		steerMotorSouthwest = new WPI_TalonSRX(RobotMap.southwestS);
		steerMotorWest = new WPI_TalonSRX(RobotMap.westS);
		steerMotorNorth = new WPI_TalonSRX(RobotMap.northS);
		steerMotorEast = new WPI_TalonSRX(RobotMap.eastS);
		steerMotorSouth = new WPI_TalonSRX(RobotMap.southS);

		steerMotorNorthwest.setInverted(RobotMap.northwestSI);
		steerMotorNortheast.setInverted(RobotMap.northeastSI);
		steerMotorSoutheast.setInverted(RobotMap.southeastSI);
		steerMotorSouthwest.setInverted(RobotMap.southwestSI);
		steerMotorWest.setInverted(RobotMap.westSI);
		steerMotorNorth.setInverted(RobotMap.northSI);
		steerMotorEast.setInverted(RobotMap.eastSI);
		steerMotorSouth.setInverted(RobotMap.southSI);

		steerMotorNorthwest.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorNortheast.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorSoutheast.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorSouthwest.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorWest.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorNorth.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorEast.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorSouth.configSelectedFeedbackSensor(FeedbackDevice.Analog);

		steerMotorNorthwest.selectProfileSlot(0, 0);
		steerMotorNortheast.selectProfileSlot(0, 0);
		steerMotorSoutheast.selectProfileSlot(0, 0);
		steerMotorSouthwest.selectProfileSlot(0, 0);
		steerMotorWest.selectProfileSlot(0, 0);
		steerMotorNorth.selectProfileSlot(0, 0);
		steerMotorEast.selectProfileSlot(0, 0);
		steerMotorSouth.selectProfileSlot(0, 0);

		steerMotorNorthwest.config_kP(0, P);
		steerMotorNortheast.config_kP(0, P);
		steerMotorSoutheast.config_kP(0, P);
		steerMotorSouthwest.config_kP(0, P);
		steerMotorWest.config_kP(0, P);
		steerMotorNorth.config_kP(0, P);
		steerMotorEast.config_kP(0, P);
		steerMotorSouth.config_kP(0, P);
		
		steerMotorNorthwest.config_kI(0, I);
		steerMotorNortheast.config_kI(0, I);
		steerMotorSoutheast.config_kI(0, I);
		steerMotorSouthwest.config_kI(0, I);
		steerMotorWest.config_kI(0, I);
		steerMotorNorth.config_kI(0, I);
		steerMotorEast.config_kI(0, I);
		steerMotorSouth.config_kI(0, I);

		steerMotorNorthwest.config_kD(0, D);		
		steerMotorNortheast.config_kD(0, D);
		steerMotorSoutheast.config_kD(0, D);
		steerMotorSouthwest.config_kD(0, D);
		steerMotorWest.config_kD(0, D);
		steerMotorNorth.config_kD(0, D);
		steerMotorEast.config_kD(0, D);
		steerMotorSouth.config_kD(0, D);

		steerMotorNorthwest.config_kF(0, F);
		steerMotorNortheast.config_kF(0, F);
		steerMotorSoutheast.config_kF(0, F);
		steerMotorSouthwest.config_kF(0, F);
		steerMotorWest.config_kF(0, F);
		steerMotorNorth.config_kF(0, F);
		steerMotorEast.config_kF(0, F);
		steerMotorSouth.config_kF(0, F);

		swerveEnclosureNorthwest = new CanTalonSwerveEnclosure("enc NW", driveMotorNorthwest, steerMotorNorthwest, GEAR_RATIO);
		swerveEnclosureNortheast = new CanTalonSwerveEnclosure("enc NE", driveMotorNortheast, steerMotorNortheast, GEAR_RATIO);
		swerveEnclosureSoutheast = new CanTalonSwerveEnclosure("enc SE", driveMotorSoutheast, steerMotorSoutheast, GEAR_RATIO);
		swerveEnclosureSouthwest = new CanTalonSwerveEnclosure("enc SW", driveMotorSouthwest, steerMotorSouthwest, GEAR_RATIO);
		swerveEnclosureWest = new CanTalonSwerveEnclosure("enc W", driveMotorWest, steerMotorWest, GEAR_RATIO);
		swerveEnclosureNorth = new CanTalonSwerveEnclosure("enc N", driveMotorNorth, steerMotorNorth, GEAR_RATIO);
		swerveEnclosureEast = new CanTalonSwerveEnclosure("enc E", driveMotorEast, steerMotorEast, GEAR_RATIO);
		swerveEnclosureSouth = new CanTalonSwerveEnclosure("enc S", driveMotorSouth, steerMotorSouth, GEAR_RATIO);

		swerveEnclosureNorthwest.setReverseSteerMotor(true);
		swerveEnclosureNortheast.setReverseSteerMotor(true);
		swerveEnclosureSoutheast.setReverseSteerMotor(true);
		swerveEnclosureSouthwest.setReverseSteerMotor(true);
		swerveEnclosureWest.setReverseSteerMotor(true);
		swerveEnclosureNorth.setReverseSteerMotor(true);
		swerveEnclosureEast.setReverseSteerMotor(true);
		swerveEnclosureSouth.setReverseSteerMotor(true);

		swerveEnclosureNorthwest.setReverseEncoder(true);
		swerveEnclosureNortheast.setReverseEncoder(true);
		swerveEnclosureSoutheast.setReverseEncoder(true);
		swerveEnclosureSouthwest.setReverseEncoder(true);
		swerveEnclosureWest.setReverseEncoder(true);
		swerveEnclosureNorth.setReverseEncoder(true);
		swerveEnclosureEast.setReverseEncoder(true);
		swerveEnclosureSouth.setReverseEncoder(true);


		swerveDrive = new SwerveDrive(swerveEnclosureNorthwest, swerveEnclosureNortheast, swerveEnclosureSoutheast, swerveEnclosureSouthwest, swerveEnclosureWest, swerveEnclosureNorth, swerveEnclosureEast, swerveEnclosureSouth, W_Default, L_Default, W_Plus, L_Plus);
		swerveDrive.setCentricMode(centricMode);

		resetEncoders();
		calibrateGyro();
	}
	
	public void drive(double fwd, double strafe, double rotateCW) {
		if (centricMode == CentricMode.ROBOT) {
			if (SouthIsFront) {
				swerveDrive.move(fwd, strafe, rotateCW, getHeading());
			} else {
				swerveDrive.move(-fwd, -strafe, rotateCW, getHeading());
			}
		} else {
			swerveDrive.move(-fwd, -strafe, rotateCW, getHeading());
		}
	}

	public double[] getWheelAngles() {
		wheelAngles[0] = steerMotorNorthwest.getSelectedSensorPosition();
		wheelAngles[1] = steerMotorNortheast.getSelectedSensorPosition();
		wheelAngles[2] = steerMotorSoutheast.getSelectedSensorPosition();
		wheelAngles[3] = steerMotorSouthwest.getSelectedSensorPosition();
		wheelAngles[4] = steerMotorWest.getSelectedSensorPosition();
		wheelAngles[5] = steerMotorNorth.getSelectedSensorPosition();
		wheelAngles[6] = steerMotorEast.getSelectedSensorPosition();
		wheelAngles[7] = steerMotorSouth .getSelectedSensorPosition();
		return wheelAngles;
	}

	public double getHeading() {
		return gyro.getAngle() % 360;
	}

	public void calibrateGyro() {
		System.out.println("Calibrating the gyro...");
		gyro.calibrate();
		System.out.println("Done calibrating the gyro.");
	}

	public void initDefaultCommand() {
		setDefaultCommand(new TeleDrive(this));
	}

	public void resetEncoders() {
		swerveEnclosureNorthwest.setEncPosition(0);
		swerveEnclosureNortheast.setEncPosition(0);
		swerveEnclosureSoutheast.setEncPosition(0);
		swerveEnclosureSouthwest.setEncPosition(0);
		swerveEnclosureWest.setEncPosition(0);
		swerveEnclosureNorth.setEncPosition(0);
		swerveEnclosureEast.setEncPosition(0);
		swerveEnclosureSouth.setEncPosition(0);
		System.out.println("Drivetrain encoders have been reset.");
	}

	public void setCentricMode(CentricMode mode) {
		swerveDrive.setCentricMode(mode);
		centricMode = mode;
	}

	public void setSouthAsFront() {
		SouthIsFront = true;
	}

	public void setNorthAsFront() {
		SouthIsFront = false;
	}

	public boolean SouthIsFront() {
		return SouthIsFront;
	}

	public CentricMode getCentricMode() {
		return centricMode;
	}

}