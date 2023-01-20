package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants; 

public class DriveSubsystem extends SubsystemBase {
    private boolean inverted;
    private CANSparkMax[] motors;

    private final DifferentialDriveKinematics kinematics;

    public  DriveSubsystem() {
        this.motors = new CANSparkMax[4];
        this.motors[0] = new CANSparkMax(Constants.WHEEL_PORT_FRONT_LEFT, MotorType.kBrushless);
        this.motors[1] = new CANSparkMax(Constants.WHEEL_PORT_FRONT_RIGHT, MotorType.kBrushless);
        this.motors[2] = new CANSparkMax(Constants.WHEEL_PORT_REAR_LEFT, MotorType.kBrushless);
        this.motors[3] = new CANSparkMax(Constants.WHEEL_PORT_REAR_RIGHT, MotorType.kBrushless);

        this.motors[0].setInverted(true);
        this.motors[1].setInverted(false);
        this.motors[2].setInverted(true);
        this.motors[3].setInverted(false);
        inverted = false;

        motors[0].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        motors[1].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        motors[2].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        motors[3].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);

        motors[0].setIdleMode(IdleMode.kBrake);
        motors[1].setIdleMode(IdleMode.kBrake);
        motors[2].setIdleMode(IdleMode.kBrake);
        motors[3].setIdleMode(IdleMode.kBrake);
        
        // meter per second
        kinematics = new DifferentialDriveKinematics(0.5715);
    }

    public void invertDrive() {
        inverted = !inverted;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        super.periodic();

        SmartDashboard.putBoolean("isInverted", inverted);
    }

    public void updateSpeed(double strafe, double drive, double turn, boolean useInverted) {
        double xSpeed = drive * Constants.MOVEMENT_SPEED;
        if (useInverted && inverted) {
            xSpeed = -xSpeed;
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, turn * -Constants.TURN_SPEED);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds); 

        double leftVelocity = wheelSpeeds.leftMetersPerSecond;

        double rightVelocity = wheelSpeeds.rightMetersPerSecond;

        MotorControllerGroup leftMotor = new MotorControllerGroup(this.motors[0], this.motors[2]);

        MotorControllerGroup rightMotor = new MotorControllerGroup(this.motors[1], this.motors[3]);

       leftMotor.set(leftVelocity);
       rightMotor.set(rightVelocity);
    }

    public void stop() {
        updateSpeed(0, 0, 0, false);
    }
}
