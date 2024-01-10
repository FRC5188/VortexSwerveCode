package frc.robot.VortexSwerveCode.lib.math;

public class Conversions {
    private static int vortexResolution = 7168;

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double vortexToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * vortexResolution));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Vortex Position Counts
     */
    public static double degreesToVortex(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * vortexResolution));
    }

    /**
     * @param velocityCounts Vortex Velocity Counts
     * @param gearRatio Gear Ratio between Vortex and Mechanism (set to 1 for Vortex RPM)
     * @return RPM of Mechanism
     */
    public static double vortexToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / vortexResolution);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Vortex and Mechanism (set to 1 for Vortex RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToVortex(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (vortexResolution / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Vortex Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Vortex and Mechanism (set to 1 for Vortex MPS)
     * @return Vortex Velocity Counts
     */
    public static double vortexToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = vortexToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Vortex and Mechanism (set to 1 for Vortex MPS)
     * @return Vortex Velocity Counts
     */
    public static double MPSToVortex(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToVortex(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Vortex Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Vortex and Wheel
     * @return Meters
     */
    public static double vortexToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * vortexResolution));
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Vortex and Wheel
     * @return Vortex Position Counts
     */
    public static double MetersToVortex(double meters, double circumference, double gearRatio){
        return meters / (circumference / (gearRatio * vortexResolution));
    }
}