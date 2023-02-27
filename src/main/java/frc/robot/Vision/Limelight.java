package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.LimelightHelpers.LimelightResults;

public class Limelight extends SubsystemBase {
    private final NetworkTable table;

    private final NetworkTableEntry tv;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry ts;
    private final NetworkTableEntry tl;

    private final NetworkTableEntry tCornX;
    private final NetworkTableEntry tCornY;

    private final NetworkTableEntry ledMode;
    private final NetworkTableEntry camMode;
    private final NetworkTableEntry pipeline;
    private final NetworkTableEntry stream;
    private final NetworkTableEntry snapshot;

    private final NetworkTableEntry tid;
    private final NetworkTableEntry botpose;

    private LimelightResults llresults = LimelightHelpers.getLatestResults("");
    public Limelight() {
        this(NetworkTableInstance.getDefault().getTable("limelight"));
    }

    public Limelight(String name) {
        this(NetworkTableInstance.getDefault().getTable("limelight-" + name));
    }

    public Limelight(NetworkTable table) {
        this.table = table;

        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tl = table.getEntry("tl");

        tCornX = table.getEntry("tcornx");
        tCornY = table.getEntry("tcorny");

        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        pipeline = table.getEntry("pipeline");
        stream = table.getEntry("stream");
        snapshot = table.getEntry("snapshot");

        tid = table.getEntry("tid");
        botpose = table.getEntry("botpose");

        ShuffleboardTab tab = Shuffleboard.getTab("April Tags");
        tab.getLayout("Bot X", BuiltInLayouts.kList)
                 .withSize(1, 1)
                 .withPosition(0, 0)
                 .addNumber("Current X", () -> Math.round(getRobotPose().getX() * 100d)/100d);
        tab.getLayout("Bot Y", BuiltInLayouts.kList)
                 .withSize(1, 1)
                 .withPosition(1, 0)
                 .addNumber("Current Y", () -> Math.round(getRobotPose().getY() * 100d)/100d);
        tab.getLayout("Bot Rot", BuiltInLayouts.kList)
                 .withSize(1, 1)
                 .withPosition(2, 0)
                 .addNumber("Current Rot", () -> Math.round(getRobotPose().getRotation().getDegrees() * 100d)/100d);
        
        
            
    }

    public boolean hasTarget()
    {
        return tv.getDouble(0) == 1;
    }

    public double getHorizontalOffset()
    {
        return tx.getDouble(0);
    }

    public double getVerticalOffset()
    {
        return ty.getDouble(0);
    }

    public double getTargetSkew() {
        return ts.getDouble(0);
    }

    public double getTargetArea()
    {
        return ta.getDouble(0);
    }

    public double getPipelineLatency() {
        return tl.getDouble(0);
    }

    public double[][] getCorners() {
        double[] x = tCornX.getDoubleArray(new double[]{0, 0});
        double[] y = tCornY.getDoubleArray(new double[]{0, 0});
        double[][] corners = new double[x.length][2];
        for (int i = 0; i < x.length; i++) {
            corners[i][0] = x[i];
            corners[i][1] = y[i];
        }
        return corners;
    }

    public void setCamMode(CamMode mode) {
        switch (mode) {
            case VISION:
                camMode.setNumber(0);
                break;
            case DRIVER:
                camMode.setNumber(1);
        }
    }

    public void setLedMode(LedMode mode) {
        switch (mode) {
            case DEFAULT:
                ledMode.setNumber(0);
                break;
            case OFF:
                ledMode.setNumber(1);
                break;
            case BLINK:
                ledMode.setNumber(2);
                break;
            case ON:
                ledMode.setNumber(3);
                break;
        }
    }

    public void setSnapshotsEnabled(boolean enabled) {
        if (enabled) {
            snapshot.setNumber(1);
        } else {
            snapshot.setNumber(0);
        }
    }

    public void setPipeline(int pipeline) {
        this.pipeline.setNumber(pipeline);
    }

    public void setStreamMode(StreamMode mode) {
        switch (mode) {
            case STANDARD:
                stream.setNumber(0);
                break;
            case PIP_MAIN:
                stream.setNumber(1);
                break;
            case PIP_SECONDARY:
                stream.setNumber(2);
                break;
        }
    }

    public long getTagID() {
        return tid.getInteger(-1);
    }

    public Pose2d getRobotPose() {
        /*
        double[] arr = botpose.getDoubleArray(new double[]{Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN});
        if (getTagID() != -1) return new Pose2d(arr[0],arr[1],Rotation2d.fromDegrees(arr[5])); 
        else return new Pose2d();     */
        return llresults.targetingResults.getBotPose2d();
    }

    public NetworkTable getTable() {
        return table;
    }

    public enum CamMode {
        VISION,
        DRIVER
    }

    public enum LedMode {
        DEFAULT,
        ON,
        OFF,
        BLINK
    }

    public enum StreamMode {
        STANDARD,
        PIP_MAIN,
        PIP_SECONDARY
    }

    @Override
    public void periodic() {
        System.out.println(getPipelineLatency());
    }
}