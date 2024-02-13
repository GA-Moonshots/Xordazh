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

    // the suppliers
    protected DoubleSupplier m_left, m_right, m_horizontal;
    protected Motor.Encoder left, right, center;

    protected double dx, dy, w;

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
        this.m_horizontal = center::getDistance;
        this.m_right = right::getDistance;
        this.m_left = left::getDistance;

    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        update(m_left.getAsDouble(), m_right.getAsDouble(), m_horizontal.getAsDouble());
    }

    @Override
    public void updatePose(Pose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = m_left.getAsDouble();
        prevRightEncoder = m_right.getAsDouble();
        prevHorizontalEncoder = m_horizontal.getAsDouble();
    }

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaLeftEncoder - deltaRightEncoder) / trackWidth
                )
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (centerWheelOffset * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }

    public void addData() {
        telemetry.addData("Odometry Left Velocity", left.getCorrectedVelocity());
        telemetry.addData("Odometry Right Velocity", right.getCorrectedVelocity());
        telemetry.addData("Odometry Center Velocity", center.getCorrectedVelocity());
        telemetry.addData("Odometry X Velocity", dx);
        telemetry.addData("Odometry Y Velocity", dy);
        telemetry.addData("Odometry Heading Velocity", w);
        telemetry.addData("Odometry Left Encoder Value", left.getPosition());
        telemetry.addData("Odometry Right Encoder Value", right.getPosition());
        telemetry.addData("Odometry Center Encoder Value", center.getPosition());
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
