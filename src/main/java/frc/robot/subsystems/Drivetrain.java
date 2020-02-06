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
	private CanTalonSwerveEnclosure swerveEnclosureFrontleft;
	private CanTalonSwerveEnclosure swerveEnclosureFrontright;
	private CanTalonSwerveEnclosure swerveEnclosureBackright;
	private CanTalonSwerveEnclosure swerveEnclosureBackleft;
	
	//Plus
	private CanTalonSwerveEnclosure swerveEnclosureBack;
	
	private SwerveDrive swerveDrive;

	public static final double GEAR_RATIO = (1024d);
	private static final double L_Default = 18;
	private static final double W_Default = 31.18;
	private static final double L_Plus = 25.5;
	private static final double W_Plus = 25.5;

	private static final double P = 10.0;
	private static final double I = 0.0;
	private static final double D = 0.0;
	private static final double F = 0.0;

	private WPI_TalonSRX driveMotorFrontleft;
	private WPI_TalonSRX driveMotorFrontright;
	private WPI_TalonSRX driveMotorBackright;
	private WPI_TalonSRX driveMotorBackleft;
	private WPI_TalonSRX driveMotorBack;

	private WPI_TalonSRX steerMotorFrontleft;
	private WPI_TalonSRX steerMotorFrontright;
	private WPI_TalonSRX steerMotorBackright;
	private WPI_TalonSRX steerMotorBackleft;
	private WPI_TalonSRX steerMotorBack;

	double[] wheelAngles = new double[5];

	private Gyro gyro = new ADXRS450_Gyro();
	private CentricMode centricMode = CentricMode.ROBOT;
	private boolean backIsForward = false;
	private boolean limitSpeed = false;

	public Drivetrain() {

	}

	public void init() {
		
		driveMotorFrontleft = new WPI_TalonSRX(RobotMap.Frontleft);
		driveMotorFrontright = new WPI_TalonSRX(RobotMap.Frontright);
		driveMotorBackright = new WPI_TalonSRX(RobotMap.Backright);
		driveMotorBackleft = new WPI_TalonSRX(RobotMap.Backleft);
		driveMotorBack = new WPI_TalonSRX(RobotMap.Back);

		driveMotorFrontleft.setInverted(RobotMap.FrontleftI);
		driveMotorFrontright.setInverted(RobotMap.FrontrightI);
		driveMotorBackright.setInverted(RobotMap.BackrightI);
		driveMotorBack.setInverted(RobotMap.BackI);

		driveMotorFrontleft.setNeutralMode(NeutralMode.Brake);
		driveMotorFrontright.setNeutralMode(NeutralMode.Brake);
		driveMotorBackright.setNeutralMode(NeutralMode.Brake);
		driveMotorBackleft.setNeutralMode(NeutralMode.Brake);
		driveMotorBack.setNeutralMode(NeutralMode.Brake);

		steerMotorFrontleft = new WPI_TalonSRX(RobotMap.FrontleftS);
		steerMotorFrontright = new WPI_TalonSRX(RobotMap.FrontrightS);
		steerMotorBackright = new WPI_TalonSRX(RobotMap.BackrightS);
		steerMotorBackleft = new WPI_TalonSRX(RobotMap.BackleftS);
		steerMotorBack = new WPI_TalonSRX(RobotMap.BackS);

		steerMotorFrontleft.setInverted(RobotMap.FrontleftSI);
		steerMotorFrontright.setInverted(RobotMap.FrontrightSI);
		steerMotorBackright.setInverted(RobotMap.BackrightSI);
		steerMotorBackleft.setInverted(RobotMap.BackleftSI);
		steerMotorBack.setInverted(RobotMap.BackSI);

		steerMotorFrontleft.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorFrontright.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorBackright.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorBackleft.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotorBack.configSelectedFeedbackSensor(FeedbackDevice.Analog);

		steerMotorFrontleft.selectProfileSlot(0, 0);
		steerMotorFrontright.selectProfileSlot(0, 0);
		steerMotorBackright.selectProfileSlot(0, 0);
		steerMotorBackleft.selectProfileSlot(0, 0);
		steerMotorBack.selectProfileSlot(0, 0);

		steerMotorFrontleft.config_kP(0, P);
		steerMotorFrontright.config_kP(0, P);
		steerMotorBackright.config_kP(0, P);
		steerMotorBackleft.config_kP(0, P);
		steerMotorBack.config_kP(0, P);
		
		steerMotorFrontleft.config_kI(0, I);
		steerMotorFrontright.config_kI(0, I);
		steerMotorBackright.config_kI(0, I);
		steerMotorBackleft.config_kI(0, I);
		steerMotorBack.config_kI(0, I);

		steerMotorFrontleft.config_kD(0, D);		
		steerMotorFrontright.config_kD(0, D);
		steerMotorBackright.config_kD(0, D);
		steerMotorBackleft.config_kD(0, D);
		steerMotorBack.config_kD(0, D);

		steerMotorFrontleft.config_kF(0, F);
		steerMotorFrontright.config_kF(0, F);
		steerMotorBackright.config_kF(0, F);
		steerMotorBackleft.config_kF(0, F);
		steerMotorBack.config_kF(0, F);

		swerveEnclosureFrontleft = new CanTalonSwerveEnclosure("enc NW", driveMotorFrontleft, steerMotorFrontleft, GEAR_RATIO);
		swerveEnclosureFrontright = new CanTalonSwerveEnclosure("enc NE", driveMotorFrontright, steerMotorFrontright, GEAR_RATIO);
		swerveEnclosureBackright = new CanTalonSwerveEnclosure("enc SE", driveMotorBackright, steerMotorBackright, GEAR_RATIO);
		swerveEnclosureBackleft = new CanTalonSwerveEnclosure("enc SW", driveMotorBackleft, steerMotorBackleft, GEAR_RATIO);
		swerveEnclosureBack = new CanTalonSwerveEnclosure("enc S", driveMotorBack, steerMotorBack, GEAR_RATIO);

		swerveEnclosureFrontleft.setReverseSteerMotor(true);
		swerveEnclosureFrontright.setReverseSteerMotor(true);
		swerveEnclosureBackright.setReverseSteerMotor(true);
		swerveEnclosureBackleft.setReverseSteerMotor(true);
		swerveEnclosureBack.setReverseSteerMotor(true);

		swerveEnclosureFrontleft.setReverseEncoder(true);
		swerveEnclosureFrontright.setReverseEncoder(true);
		swerveEnclosureBackright.setReverseEncoder(true);
		swerveEnclosureBackleft.setReverseEncoder(true);
		swerveEnclosureBack.setReverseEncoder(true);


		swerveDrive = new SwerveDrive(swerveEnclosureFrontleft, swerveEnclosureFrontright, swerveEnclosureBackright, swerveEnclosureBackleft, swerveEnclosureBack, W_Default, L_Default, W_Plus, L_Plus);
		swerveDrive.setCentricMode(centricMode);

		resetEncoders();
		calibrateGyro();
	}
	
	public void drive(double fwd, double strafe, double rotateCW) {
		if (LimitSpeed()) {
			fwd *= 0.25;
			strafe *= 0.25;
			rotateCW *= 0.25;	
		}	
		if ((centricMode != CentricMode.ROBOT) || (!isBackForward())) {
			fwd = -fwd;
			strafe = -strafe;			
		}
		swerveDrive.move(fwd, strafe, rotateCW, getHeading());
	}

	public double[] getWheelAngles() {
		wheelAngles[0] = steerMotorFrontleft.getSelectedSensorPosition();
		wheelAngles[1] = steerMotorFrontright.getSelectedSensorPosition();
		wheelAngles[2] = steerMotorBackright.getSelectedSensorPosition();
		wheelAngles[3] = steerMotorBackleft.getSelectedSensorPosition();
		wheelAngles[4] = steerMotorBack.getSelectedSensorPosition();
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
		swerveEnclosureFrontleft.setEncPosition(0);
		swerveEnclosureFrontright.setEncPosition(0);
		swerveEnclosureBackright.setEncPosition(0);
		swerveEnclosureBackleft.setEncPosition(0);
		swerveEnclosureBack.setEncPosition(0);
		System.out.println("Drivetrain encoders have been reset.");
	}

	public void setCentricMode(CentricMode mode) {
		swerveDrive.setCentricMode(mode);
		centricMode = mode;
	}

	public void setBackAsForward() {
		backIsForward = true;
	}

	public void setFrontAsForward() {
		backIsForward = false;
	}

	public boolean isBackForward() {
		return backIsForward;
	}

	public CentricMode getCentricMode() {
		return centricMode;
	}
	
	public boolean LimitSpeed() {
		return limitSpeed;
	}

	public void setLimitSpeed(){
		limitSpeed = true;
	}
	public void setFullSpeed(){
		limitSpeed = false;
	}


}