//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Constants;
//
//public class VisionSubsystem extends SubsystemBase {
//
//    private final Limelight3A limelight;
//    private boolean hasTarget = false;
//    private double calculatedSliderExtension = 0;
//    private double calculatedClawRotation = 0;
//    private double targetHorizontalAngle = 0;
//    private double targetVerticalAngle = 0;
//
//
//    public VisionSubsystem(HardwareMap hardwareMap) {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.start();
//    }
//
//    @Override
//    public void periodic() {
//
//        double[] pythonInputs = new double[8];
//        pythonInputs[1] = Constants.Vision.LIMELIGHT_HORIZONTAL_OFFSET_CM;
//        pythonInputs[2] = Constants.Vision.LIMELIGHT_FOV_DEG;
//
//        limelight.updatePythonInputs(pythonInputs);
//        LLResult result = limelight.getLatestResult();
//
//        this.hasTarget = false;
//
//        if (result != null && result.getTa() > 0.001) {
//            double[] pythonResults = result.getPythonOutput();
//            if (pythonResults != null && pythonResults.length > 3 && pythonResults[0] == 1.0) {
//                this.hasTarget = true;
//                this.targetHorizontalAngle = result.getTx();
//                this.targetVerticalAngle = result.getTy();
//
//                // 1. Calculate Real-World Distance
//                double angleToTargetRad = Math.toRadians(Constants.Vision.LIMELIGHT_PITCH_ANGLE_DEG + result.getTy());
//                double distanceToTargetCm = 0;
//                if (angleToTargetRad < 0) { // Angle must be negative (pointing down)
//                    distanceToTargetCm = Constants.Vision.LIMELIGHT_HEIGHT_CM / Math.tan(-angleToTargetRad);
//                }
//
//                // 2. Map Distance to Slider Extension
//                double targetExtensionCm = (1.0 * (distanceToTargetCm - 13.0));
//                if (targetExtensionCm < 0) { targetExtensionCm = 0; }
//
//                // 3. Save the final calculated values
//                this.calculatedSliderExtension = Math.min(targetExtensionCm, Constants.Vision.MAX_AUTO_INTAKE_EXTENSION_CM);
//
//                // We'll calculate the rotation as well, for future use
//                double visionAngle = pythonResults[3];
//                // Note: We can't do the "shortest path" logic here because we don't know the claw's current angle.
//                // The command will handle that. For now, we just provide the primary calculated angle.
//
//                if (visionAngle >= 0 && visionAngle <= 30 || visionAngle >= 150) {
//                    // If the sample is nearly vertical (0-30 or 150-180 degrees)
//                    this.calculatedClawRotation = 96.0; // Use the 'completely vertical' servo position
//                } else if (visionAngle > 30 && visionAngle <= 70) {
//                    // If the sample is at the specific diagonal 'sample angle' (100-135)
//                    this.calculatedClawRotation = 252.0; // Use the 'sample angle' servo position
//                } else if(visionAngle > 70 && visionAngle <= 110) {
//                    // For all other angles (mostly horizontal)
//                    this.calculatedClawRotation = 200.0; // Use the 'completely horizontal' servo position
//                } else { //100-150
//                    this.calculatedClawRotation = 148.5;
//                }
//            }
//        }
//    }
//
//    // --- PUBLIC GETTERS ---
//
//    public boolean hasTarget() {
//        return hasTarget;
//    }
//
//    public double getTargetSliderExtension() {
//        return calculatedSliderExtension;
//    }
//
//    public double getTargetClawRotation() {
//        return calculatedClawRotation;
//    }
//
//    public double getTargetHorizontalAngle() {
//        return targetHorizontalAngle;
//    }
//
//    public double getTargetVerticalAngle() {
//        return targetVerticalAngle;
//    }
//
//    public void setPipeline(int pipelineIndex) {
//        limelight.pipelineSwitch(pipelineIndex);
//    }
//}