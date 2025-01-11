package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Limelight Object
 */
public class Limelight extends SubsystemBase {

    private final NetworkTable m_networkTable;
    private final NetworkTableEntry m_tx;
    private final NetworkTableEntry m_ty;
    private final NetworkTableEntry m_ta;
    private final NetworkTableEntry m_tv;
    private final NetworkTableEntry m_ts;

    private double m_cached_tx;
    private double m_cached_ty;
    private double m_cached_ta;
    private double m_cached_tv;
    private double m_cached_ts;

    private double[] xAverageArray = { 0, 0, 0, 0, 0 };
    private double[] yAverageArray = { 0, 0, 0, 0, 0 };

    private double xAverageOffset = 0;
    private double yAverageOffset = 0;
    private String limelightName;

    /**
     * Initializes all network table entries, and creates object Turns on vision and
     * LEDs by default
     */
    public Limelight() {
        m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");

        setVision(true);
        setLED(false);

        m_tx = m_networkTable.getEntry("tx");
        m_ty = m_networkTable.getEntry("ty");
        m_ta = m_networkTable.getEntry("ta");
        m_tv = m_networkTable.getEntry("tv");
        m_ts = m_networkTable.getEntry("ts");

        setPipeline(0);
    }

    /**
     * Returns whether or not the limelight has any valid targets
     *
     * @return true if the limelight detects a valid target
     */
    public boolean getHasTarget() {
        return (m_cached_tv == 1);
    }

    /**
     * Returns the coordinates of the centre of the targeting box
     *
     * @return An array containing the x offset and y offset of the target {xOffset,
     *         yOffset}
     */
    public double[] getOffset() {
        return new double[] { m_cached_tx, m_cached_ty };
    }

    /**
     * Returns the x-offset from the target
     *
     * @return A double containing the x-offset
     */
    public double getOffsetX() {
        return m_cached_tx;
    }

    /**
     * Returns the y-offset from the target
     *
     * @return A double containing the y-offset
     */
    public double getOffsetY() {
        return m_cached_ty;
    }

    /**
     * Returns the area of the target as a percentage of the whole image (0% to
     * 100%)
     *
     * @return the area of the target
     */
    public double getTargetArea() {
        return m_cached_ta;
    }

    /**
     * Returns the skew of the targeting box from -90 to 0 degrees
     *
     * @return the skew of the targeting box
     */
    public double getTargetSkew() {
        return m_cached_ts;
    }

    /**
     * Returns true if the LED is on and false if it isn't
     *
     * @return true if the LED is on
     */
    public boolean getLED() {
        return (!(m_networkTable.getEntry("ledMode").getDouble(0.0) == 1.0));
    }

    /**
     * Turns the leds on the limelight on or off
     *
     * @param on Should the LEDs on the limelight be on
     */
    public void setLED(boolean on) {
        if (on) {
            m_networkTable.getEntry("ledMode").setNumber(3);
        } else {
            m_networkTable.getEntry("ledMode").setNumber(1);
        }

    }

    public void blink() {
        m_networkTable.getEntry("ledMode").setNumber(2);
    }

    /**
     * Sets the leds on the limelight to the pipeline default
     */
    public void setLEDDefault() {
        m_networkTable.getEntry("ledMode").setNumber(0);
    }

    /**
     * Enables or disables vision processcing on the limelight
     *
     * @param on Should vision processing be on or off
     */
    public void setVision(boolean on) {
        if (on) {
            m_networkTable.getEntry("camMode").setNumber(0);
        } else {
            m_networkTable.getEntry("camMode").setNumber(1);
        }

    }

    /**
     * Sets the limelight to the desired vision processcing pipeline
     *
     * @param pipeline the pipeline to set the limelight to from 0 to 9
     * @throws IndexOutOfBoundsException if the pipeine number is not between 0 and
     *                                   9
     */
    public void setPipeline(int pipeline) {

        if (pipeline > 9 || pipeline < 0) {
            throw new IndexOutOfBoundsException("Pipeline Index Is Out Of Bounds");
        }
        m_networkTable.getEntry("pipeline").setNumber(pipeline);
    }

    @Override
    public void periodic() {

        m_cached_tx = m_tx.getDouble(0);
        m_cached_ty = m_ty.getDouble(0);
        m_cached_ta = m_ta.getDouble(0);
        m_cached_tv = m_tv.getDouble(0);
        m_cached_ts = m_ts.getDouble(0);

        // SmartDashboard.putNumberArray("vision/targetPosition",
        // findTurretRelativePosition());
        // SmartDashboard.putNumber("vision/targetDistance", findDistance());
        Logger.recordOutput("Vision/Limelight/Has Targets", getHasTarget());
        Logger.recordOutput("Vision/Limelight/x Offset", getOffsetX());

    }
}