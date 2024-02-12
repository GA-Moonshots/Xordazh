package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Xordazh;
import org.firstinspires.ftc.teamcode.config.DriveConfig;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;

import java.util.Optional;

public class MecanumDrive extends SubsystemBase {
    private boolean isFieldCentric;
    private double fieldCentricTarget;

    private final MotorEx leftFront;
    private final MotorEx leftBack;
    private final MotorEx rightBack;
    private final MotorEx rightFront;

    private final Motor.Encoder leftEncoder;
    private final Motor.Encoder rightEncoder;
    private final Motor.Encoder centerEncoder;

    private final HolonomicOdometry odometry;

    private final Telemetry telemetry;

    public MecanumDrive(Xordazh robot, Pose2d startingPosition) {
        HardwareMap hardwareMap = robot.opMode.hardwareMap;

        this.leftFront = new MotorEx(hardwareMap, HardwareNames.LEFT_FRONT_NAME);
        this.leftBack = new MotorEx(hardwareMap, HardwareNames.LEFT_BACK_NAME);
        this.rightBack = new MotorEx(hardwareMap, HardwareNames.RIGHT_BACK_NAME);
        this.rightFront = new MotorEx(hardwareMap, HardwareNames.RIGHT_FRONT_NAME);

        this.leftFront.setRunMode(Motor.RunMode.RawPower);
        this.leftBack.setRunMode(Motor.RunMode.RawPower);
        this.rightBack.setRunMode(Motor.RunMode.RawPower);
        this.rightFront.setRunMode(Motor.RunMode.RawPower);


        this.leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.fieldCentricTarget = startingPosition.getHeading();

        this.leftEncoder = this.leftFront.encoder
                .setDistancePerPulse(OdometryConfig.IN_PER_TICK);
        this.rightEncoder = this.rightFront.encoder
                .setDistancePerPulse(OdometryConfig.MM_PER_TICK);
        this.centerEncoder = this.leftBack.encoder
                .setDistancePerPulse(OdometryConfig.MM_PER_TICK);

        this.odometry = new HolonomicOdometry(
                this.leftEncoder::getDistance,
                this.rightEncoder::getDistance,
                this.centerEncoder::getDistance,
                OdometryConfig.PARALLEL_DISTANCE_MM, OdometryConfig.PERPENDICULAR_DISTANCE_MM
        );

        this.odometry.updatePose(startingPosition);
        this.telemetry = robot.opMode.telemetry;
    }

    public void drive(double forward, double strafe, double turn) {
        this.odometry.updatePose();

        if(this.isFieldCentric) {
            double heading = this.odometry.getPose().getHeading();
            double temp = forward;

            forward = temp * Math.cos(heading) - strafe * Math.sin(heading);
            strafe = temp * Math.sin(heading) + strafe * Math.cos(heading);

            this.telemetry.addData("Drive Mode", "Field Centric");
        } else {
            this.telemetry.addData("Drive Mode", "Robot Centric");
        }

        // There's not much else we need to do, Mecanum drives are pretty easy.
        double lf = - forward + strafe - turn;
        double lb = - forward - strafe - turn;
        double rb =   forward - strafe - turn;
        double rf =   forward + strafe - turn;

        // This ensures that no motor power gets clipped
        // By multiplying the ratio by MOTOR_MAX_SPEED / powCap,
        // no power value exceeds MOTOR_MAX_SPEED, and it remains proportional to the other values.
        double powCap = Math.max(
                Math.max(
                        Math.max(
                                lf, lb
                        ),
                        Math.max(
                                rb, rf
                        )
                ),
                1.0
        );
        drive(
                lf * (DriveConfig.MOTOR_MAX_SPEED / powCap),
                lb * (DriveConfig.MOTOR_MAX_SPEED / powCap),
                rb * (DriveConfig.MOTOR_MAX_SPEED / powCap),
                rf * (DriveConfig.MOTOR_MAX_SPEED / powCap)
        );

        telemetry.addData("Odometry Encoders", "(%s, %s, %s)",
                this.leftEncoder.getDistance(),
                this.rightEncoder.getDistance(),
                this.centerEncoder.getDistance()
        );
        telemetry.addData("Odometry", "(%.2f, %.2f, %.2f)",
                this.odometry.getPose().getX(),
                this.odometry.getPose().getY(),
                this.odometry.getPose().getHeading()
        );
    }

    public void drive(double lf, double lb, double rb, double rf) {
        // Even though we are ensuring that no value is above MOTOR_MAX_SPEED, we should still clip
        // the values here.
        this.leftFront.set(Range.clip(lf, -DriveConfig.MOTOR_MAX_SPEED, DriveConfig.MOTOR_MAX_SPEED));
        this.leftBack.set(Range.clip(lb, -DriveConfig.MOTOR_MAX_SPEED, DriveConfig.MOTOR_MAX_SPEED));
        this.rightBack.set(Range.clip(rb, -DriveConfig.MOTOR_MAX_SPEED, DriveConfig.MOTOR_MAX_SPEED));
        this.rightFront.set(Range.clip(rf, -DriveConfig.MOTOR_MAX_SPEED, DriveConfig.MOTOR_MAX_SPEED));
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void resetFieldCentricTarget() {
        this.fieldCentricTarget = this.odometry.getPose().getHeading();
    }

    public void stop() {
        drive(0, 0, 0, 0);
    }
}
