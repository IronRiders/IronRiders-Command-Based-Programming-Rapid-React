package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax bottomMotor, topMotor;
    private SparkMaxPIDController topPID, bottomPID;
    
    public ShooterSubsystem() {
        bottomMotor = new CANSparkMax(Constants.SHOOTER_PORT_BOTTOM, MotorType.kBrushless);
        topMotor = new CANSparkMax(Constants.SHOOTER_PORT_TOP, MotorType.kBrushless);
        bottomMotor.setIdleMode(IdleMode.kCoast);
        topMotor.setIdleMode(IdleMode.kCoast);
        topPID = topMotor.getPIDController();
        bottomPID = bottomMotor.getPIDController();
        topMotor.setInverted(true);
        bottomMotor.setInverted(true);
        bottomMotor.enableVoltageCompensation(12);
        topMotor.enableVoltageCompensation(12);
        topPID.setOutputRange(-1, 1);
        bottomPID.setOutputRange(-1, 1);
        bottomPID.setP(Constants.SHOOTER_P);
        bottomPID.setI(Constants.SHOOTER_I);
        bottomPID.setD(Constants.SHOOTER_D);
        topPID.setP(Constants.SHOOTER_P);
        topPID.setI(Constants.SHOOTER_I);
        topPID.setD(Constants.SHOOTER_D);
    }

    public void shoot(double rpm) {
        double minimum = Constants.SHOOTER_MINIMUM_SPEED;
        double maximum = Constants.SHOOTER_MAXIMUM_SPEED;
        rpm = Math.min(Math.max(rpm, minimum), maximum);
        topPID.setReference(rpm * Constants.SHOOTER_TOP_MOTOR_CHANGE, ControlType.kVelocity);
        bottomPID.setReference(rpm, ControlType.kVelocity);
    }
    
    public void stop() {
        topPID.setReference(0, ControlType.kVelocity);
        bottomPID.setReference(0, ControlType.kVelocity);
    }

    public static double distanceToRPM(double distance) {
       return 1921.5 + (-3.0514 * distance) + (0.0262 * distance * distance);
    }
}
