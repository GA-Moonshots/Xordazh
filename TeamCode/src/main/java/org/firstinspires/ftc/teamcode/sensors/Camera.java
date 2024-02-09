package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Camera {
    private final AprilTagProcessor aprilTag;
    private DashboardStreamProcessor dashboardStream;
    private final VisionPortal visionPortal;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        dashboardStream = new DashboardStreamProcessor();


        WebcamName name = hardwareMap.get(WebcamName.class, HardwareNames.WEBCAM_NAME);

        visionPortal = new VisionPortal.Builder()
                .setCamera(name)
                .addProcessor(aprilTag)
                .addProcessor(dashboardStream)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(25);
        }
        return aprilTag.getDetections();
    }

    public String getStatus(){
        return visionPortal.getCameraState().toString();
    }
}
