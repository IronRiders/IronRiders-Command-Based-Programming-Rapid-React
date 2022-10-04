package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

public class IndexerSubsystem extends SubsystemBase {
    private Servo servo;

    public IndexerSubsystem() {
        servo = new Servo(Constants.INDEXER_SERVO_PORT);
    }

    public void extend() {
        servo.setAngle(Constants.INDEXER_EXTENSION);
    }

    public void retract() {
        servo.setAngle(0);
    }
}
