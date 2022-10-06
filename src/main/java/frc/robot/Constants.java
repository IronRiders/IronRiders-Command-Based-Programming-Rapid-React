package frc.robot;

public final class Constants {
    // Drivetrain
    public static final double DRIVE_SPEED_MULT = 1.0; // should be between 0 and 1
    public static final double DRIVE_SPEED_AUTO = 0.1 / DRIVE_SPEED_MULT;
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final double MOVEMENT_SPEED = 1; // meters per second
    public static final double TURN_SPEED = 1; // radians per second

    // Indexer
    public static final double INDEXER_EXTENSION = 40; // Degrees

    // Intake
    public static final double INTAKE_SPEED = 0.8;
    public static final double DEPLOY_ENCODER_COUNT = 4096;
    public static final double DEPLOY_SPEED = 0.1;
    public static final int DEPLOY_CURRENT_PEAK_LIMIT = 0;
    public static final int DEPLOY_CURRENT_PEAK_TIME = 10;
    public static final int DEPLOY_CURRENT_CONT_LIMIT = 1;
    public static final double DEPLOY_FORWARD_LIMIT = DEPLOY_ENCODER_COUNT / 4 * 3;
    public static final double DEPLOY_REVERSE_LIMIT = DEPLOY_ENCODER_COUNT / 4;

    // Shooter
    public static final double SHOOTER_P = 0.00017;
    public static final double SHOOTER_I = 0.0000007;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_TOP_MOTOR_CHANGE = 1.0;
    public static final double SHOOTER_MAXIMUM_SPEED = 3000;
    public static final double SHOOTER_MINIMUM_SPEED = 500;

    // Automatic distancing
    public static final double KP_DIST = 0.225;
    public static final double DIST_MAX_SPEED = 0.2;
    public static final double OK_DISTANCE = 3;
    public static final double TARGET_DIST = 200;

    // Automatic turning
    public static final double TURN_MIN_ANGLE = 0.3;
    public static final double TURN_MAX_SPEED = 0.18;
    public static final double TURN_P = 0.025;
    public static final double TURN_TOLERANCE = 0.01;

    // Ports
    public static final int WHEEL_PORT_FRONT_LEFT = 1;
    public static final int WHEEL_PORT_REAR_LEFT = 3;
    public static final int WHEEL_PORT_FRONT_RIGHT = 2;
    public static final int WHEEL_PORT_REAR_RIGHT = 4;
    public static final int SHOOTER_PORT_BOTTOM = 5;
    public static final int SHOOTER_PORT_TOP = 6;
    public static final int INTAKE_BELT_PORT = 7;
    public static final int INTAKE_DEPLOYMENT_PORT = 8;
    public static final int CAMERA_USB_PORT = 0;
    public static final int INDEXER_SERVO_PORT = 0;
    public static final int CLIMBER_PORT_1 = 9;
    public static final int CLIMBER_PORT_2 = 10;

    // Vision
    public static final double VISION_CAMERA_ANGLE = 20;
    public static final double VISION_DELTA_HEIGHT = 104 - 20; // target - camera

    // Joystick
    public static final double DEADBAND = 0.07;
    public static final double EXPONENT = 0.0; // between 0 and 1

    // Climber
    public static final double CLIMBER_POWER = 1;
}
