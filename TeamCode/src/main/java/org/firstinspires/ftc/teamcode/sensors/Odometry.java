package org.firstinspires.ftc.teamcode.sensors;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;

import java.util.function.DoubleSupplier;

public class Odometry extends com.arcrobotics.ftclib.kinematics.Odometry {
    protected double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    protected Rotation2d previousAngle;
    protected double centerWheelOffset;
    protected Telemetry telemetry;

    protected Motor.Encoder left, right, center;
    // Double representations
    protected double dx, dy, dw;

    private Odometry(Pose2d initialPose, double trackWidth, double centerWheelOffset) {
        super(initialPose, trackWidth);
        previousAngle = initialPose.getRotation();
        this.centerWheelOffset = centerWheelOffset;
    }

    private Odometry(double trackWidth, double centerWheelOffset) {
        this(new Pose2d(), trackWidth, centerWheelOffset);
    }

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry) {
        this(OdometryConfig.PARALLEL_DISTANCE, OdometryConfig.PERPENDICULAR_DISTANCE);

        this.left = new MotorEx(hardwareMap, HardwareNames.LEFT_ODOMETRY_NAME).encoder
                .setDistancePerPulse(-OdometryConfig.IN_PER_TICK);
        this.right = new MotorEx(hardwareMap, HardwareNames.RIGHT_ODOMETRY_NAME).encoder
                .setDistancePerPulse(OdometryConfig.IN_PER_TICK);
        this.center = new MotorEx(hardwareMap, HardwareNames.CENTER_ODOMETRY_NAME).encoder
                .setDistancePerPulse(OdometryConfig.IN_PER_TICK);

        this.telemetry = telemetry;
        this.left.reset();
        this.right.reset();
        this.center.reset();
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        update(this.left.getDistance(), this.right.getDistance(), this.center.getDistance());
    }

    @Override
    public void updatePose(Pose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = this.left.getDistance();
        prevRightEncoder = this.right.getDistance();
        prevHorizontalEncoder = this.center.getDistance();
    }

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        // w, or omega, is a rotational speed
        Rotation2d w = new Rotation2d((deltaLeftEncoder - deltaRightEncoder) / trackWidth);

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;


        dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        dy = deltaHorizontalEncoder - (centerWheelOffset * w.getRadians());
        dw = w.getRadians();

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = w.plus(previousAngle);

        robotPose = new Pose2d(newPose.getTranslation(), w);
    }

    public void addData() {
        telemetry.addData("Odometry Left Velocity", left.getCorrectedVelocity());
        telemetry.addData("Odometry Right Velocity", right.getCorrectedVelocity());
        telemetry.addData("Odometry Center Velocity", center.getCorrectedVelocity());
        telemetry.addData("Odometry X Velocity", dx);
        telemetry.addData("Odometry Y Velocity", dy);
        telemetry.addData("Odometry Heading Velocity", dw);
        telemetry.addData("Odometry Left Encoder Value", left.getPosition());
        telemetry.addData("Odometry Right Encoder Value", right.getPosition());
        telemetry.addData("Odometry Center Encoder Value", center.getPosition());
        telemetry.addData("Odometry Left Encoder Distance", left.getDistance());
        telemetry.addData("Odometry Right Encoder Distance", right.getDistance());
        telemetry.addData("Odometry Center Encoder Distance", center.getDistance());
        telemetry.addData("Odometry X Position", getX());
        telemetry.addData("Odometry Y Position", getY());
        telemetry.addData("Odometry Heading", Math.toDegrees(getHeading()));
    }

    public double getX() {
        return this.robotPose.getX();
    }

    public double getY() {
        return this.robotPose.getY();
    }

    public double getHeading() {
        return this.robotPose.getHeading();
    }
}
