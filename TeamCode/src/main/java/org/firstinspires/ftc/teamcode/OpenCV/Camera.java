package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.Pipelines.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {

    private OpenCvCamera camera;
    private String cameraConfigurationName = "CleverCamera";
    private OpenCvCameraRotation cameraOrientation = OpenCvCameraRotation.UPRIGHT;

    private int HEIGHT = 240, WIDTH = 320;
    private boolean isOpened = false;

    private LinearOpMode opMode;
    private Telemetry dashboardTelemetry;

    public Camera(LinearOpMode opMode, Telemetry dashboardTelemetry) {
        this.dashboardTelemetry = dashboardTelemetry;
        this.opMode = opMode;

        initCamera();
    }

    private void initCamera(){
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, cameraConfigurationName), cameraMonitorViewId);
    }

    public void setPipeline(Enums.Pipelines desiredPipeline) {
        if (isOpened) {
            camera.closeCameraDeviceAsync(() -> {});
            isOpened = false;
        }

        switch (desiredPipeline) {
            case DETECTING_PIXELS_ON_BACKDROP: { camera.setPipeline(new PixelDetectionPipeline()); }
            default: {}
        }

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WIDTH, HEIGHT, cameraOrientation);
                FtcDashboard.getInstance().startCameraStream(camera, 0);

                isOpened = true;
            }
            @Override
            public void onError(int errorCode) {
                dashboardTelemetry.addLine("ERROR: couldn't open camera");
                dashboardTelemetry.update();
            }
        });
    }

    public boolean isOpened() { return isOpened; }

    public void setCameraParameters(int width, int height) {
        WIDTH = width;
        HEIGHT = height;
    }

    public void setCameraParameters(OpenCvCameraRotation cameraOrientation) { this.cameraOrientation = cameraOrientation; }

    public void setCameraParameters(int width, int height, OpenCvCameraRotation cameraOrientation) {
        setCameraParameters(width, height);
        setCameraParameters(cameraOrientation);
    }

    protected String getCameraName() { return cameraConfigurationName; }

    public int getWidth() { return WIDTH; }

    public int getHeight() { return HEIGHT; }

    public OpenCvCameraRotation getOrientation() { return cameraOrientation; }

}
