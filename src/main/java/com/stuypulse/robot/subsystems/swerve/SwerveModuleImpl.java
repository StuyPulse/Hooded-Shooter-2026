package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Gains.Swerve.Drive;
import com.stuypulse.robot.constants.Gains.Swerve.Turn;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleImpl extends SwerveModule {

    private final Rotation2d angleOffset;

    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMax pivotMotor;
    private final CANcoder pivotEncoder;

    private final PIDController driveControllerPID;
    private final SimpleMotorFeedforward driveControllerFF;

    private final PIDController pivotController;

    public SwerveModuleImpl(String name, Translation2d location, Rotation2d angleOffset, int driveMotorID, int pivotMotorID, int pivotEncoderID, boolean driveInverted) {
        super(name, location);

        this.angleOffset = angleOffset;

        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
        pivotMotor.configure(Motors.Swerve.Turn.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotEncoder = new CANcoder(pivotEncoderID, Settings.Swerve.DRIVE_CANBUS);

        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.configure(Motors.Swerve.Drive.motorConfig.inverted(driveInverted), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveEncoder = driveMotor.getEncoder();

        driveControllerPID  = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        driveControllerFF = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

        pivotController = new PIDController(Turn.kP, Turn.kI, Turn.kD);
        pivotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getVelocity(); // RPM
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(angleOffset);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();

        double driveVoltage = 
            driveControllerPID.calculate(getTargetState().speedMetersPerSecond, getVelocity()) +
            driveControllerFF.calculate(getTargetState().speedMetersPerSecond);
        
        double pivotVoltage = pivotController.calculate(getTargetState().angle.getRadians(), getAngle().getRadians());
        
        if (Math.abs(getTargetState().speedMetersPerSecond) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
            driveMotor.setVoltage(0);
            pivotMotor.setVoltage(0);
        } else {
            driveMotor.setVoltage(driveVoltage);
            pivotMotor.setVoltage(pivotVoltage);
        }

        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Turn Voltage", pivotVoltage);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Voltage", driveVoltage);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Angle Error (deg)", pivotController.getError() * 180.0 / Math.PI);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Angle (deg)", pivotEncoder.getPosition().getValueAsDouble() * 360.0);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Raw Encoder Angle (deg)", Units.rotationsToDegrees(pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360.0));
    }
}