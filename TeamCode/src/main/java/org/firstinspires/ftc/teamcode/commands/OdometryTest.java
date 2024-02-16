package org.firstinspires.ftc.teamcode.commands;

import android.os.Environment;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Xordazh;
import org.firstinspires.ftc.teamcode.config.InputConfig;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class OdometryTest extends CommandBase {
    private final MecanumDrive drive;
    private final Xordazh robot;
    private final RevIMU imu;
    private double time = 0;
    private ElapsedTime rt = new ElapsedTime();
    private double lastImuTheta, lastUpdateTime = 0;

    public OdometryTest(Xordazh robot) {
        this.drive = robot.getDrive();
        this.robot = robot;
        this.imu = new RevIMU(robot.opMode.hardwareMap);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Runs before anything else (besides constructor ofc) from this command is run.
        this.imu.init();
        this.rt.reset();

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(
                Environment.getExternalStorageDirectory().getPath()+"/FIRST/forward.csv"
        )) ) {
            writer.write("Elv,Erv,Ecv,dx,dy,El,Er,Ec,Eld,Erd,Ecd,x,y,theta,omega,imuTheta,imuOmega,t,dt,theta/imuTheta\n");
            //writer.write("x,y,theta,omega,imuTheta,imuOmega,t,dt,theta/imuTheta\n");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void execute() {
        drive.drive(0, 0, 0.5 * Math.cos(rt.seconds() / 2.5));
        addData();
    }

    private void addData() {
        robot.opMode.telemetry.addData("Position", drive.odometry.left.getPosition());
        robot.opMode.telemetry.addData("Distatnce", drive.odometry.left.getDistance());
        robot.opMode.telemetry.addData("Ratio",
                drive.odometry.left.getPosition() != 0d ?
                drive.odometry.left.getDistance() / drive.odometry.left.getPosition() :
                0d
        );

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(
                Environment.getExternalStorageDirectory().getPath()+"/FIRST/forward.csv", true
        ))) {
            double t = rt.seconds();
            double h = drive.odometry.getHeading();
            double ih = Math.toRadians(imu.getHeading());
            double s = ih != 0d ? h / ih : h;
            String d = String.format(
                    "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
                    drive.odometry.left.getCorrectedVelocity(),drive.odometry.right.getCorrectedVelocity(),drive.odometry.center.getCorrectedVelocity(),
                    drive.odometry.dx,drive.odometry.dy,
                    drive.odometry.left.getPosition(),drive.odometry.right.getPosition(),drive.odometry.center.getPosition(),
                    drive.odometry.left.getDistance(),drive.odometry.right.getDistance(),drive.odometry.center.getDistance(),
                    drive.odometry.getX(),drive.odometry.getY(),
                    h,drive.odometry.w,
                    ih,ih - lastImuTheta,
                    t,t-lastUpdateTime,s
            );
            //robot.opMode.telemetry.addData("Data", d);
            writer.write(d);
            lastUpdateTime = t;
            lastImuTheta = h;
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    //@Override
    public void execute_forward() {
        if(rt.seconds() <= 1){
            drive.drive(-0.3,0.0,0.0);
        }
        else{
            drive.stop();
            return;
        }
        addData();
    }

    /**
     * This command never ends
     * @return false
     */
    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        try {
            writer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
