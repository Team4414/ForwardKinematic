/**
 * Forward Kinematics Class.
 *
 * <P>First Test of Forward Kinematics Model for a Differential Drive Robot.</P>
 *
 * @author Avidh Bavkar (FRC Team 7404 : HighTide) [avidhbavkar@gmail.com]
 * @version 1.0
 * @since 1.0
 */
public class ForwardKinematics {

    /**
     * Radius of the drive wheels
     */
    private final double kRadius;

    /**
     * Distance between the centers of both wheels
     */
    private final double kWheelBase;

    /**
     * The amount, in seconds, between calls of the "update" method
     */
    private static final double K_TIMESTEP = 0.05;

    /**
     * Robot X Position
     */
    private double mXpos;

    /**
     * Robot Y Position
     */
    private double mYpos;

    /**
     * Robot Heading
     */
    private double mHeading;

    /**
     * Constructor.
     *
     * @param wheelRadius Radius of the drive wheels.
     * @param wheelBaseLength Distance between the centers of both wheels.
     */
    public ForwardKinematics(double wheelRadius, double wheelBaseLength){
        kRadius = wheelRadius;
        kWheelBase = wheelBaseLength;

        mXpos = 0;
        mYpos = 0;
        mHeading = 0;
    }

    /**
     * Constructor.
     *
     * @param wheelRadius Radius of the drive wheels.
     * @param wheelBaseLength Distance between the centers of both wheels.
     * @param xPos Initial X Position of the Robot.
     * @param yPos Initial Y Position of the Robot.
     * @param heading Initial Heading of the Robot.
     */
    public ForwardKinematics(double wheelRadius, double wheelBaseLength, double xPos, double yPos, double heading){
        this(wheelRadius, wheelBaseLength);

        mXpos = xPos;
        mYpos = yPos;
        mHeading = heading;
    }

    /**
     * Update Method.
     *
     * <P>Integrates coordinates and heading of the robot. Expected to be called once per specified TIMESTEP</P>
     */
    public void update(){
        mXpos += K_TIMESTEP * getDeltaX();
        mYpos += K_TIMESTEP * getDeltaY();
        mHeading += K_TIMESTEP * getDeltaHeading();
    }

    /**
     * Get X Position Method.
     * @return The current X position of the robot.
     */
    public double getXPosition(){ return mXpos; }

    /**
     * Get Y Position Method.
     * @return The current Y position of the robot.
     */
    public double getYPosition(){ return mYpos; }

    /**
     * Get Heading Method.
     * @return The current heading of the robot.
     */
    @Deprecated
    public double getHeading(){ return mHeading; }


    /**
     * Get Delta X Method.
     *
     * Calculates the change in the X Position of the Robot with respect to wheel velocities and heading.
     *
     * @return The change in X Position.
     */
    private double getDeltaX(){

        //
        // xDot = cos(omega * dt)(x - ICCx) - sin(omega * dt)(y - ICCy) + ICCx
        //

        double[] icc = calculateICC();

        return (  Math.cos(calculateOmega() * K_TIMESTEP) * (mXpos - icc[0])
                - Math.sin(calculateOmega() * K_TIMESTEP) * (mYpos - icc[1])
                + icc[0] );
    }

    /**
     * Get Delta Y Method.
     *
     * Calculates the change in the Y Position of the Robot with respect to wheel velocities and heading.
     *
     * @return The change in Y Position.
     */
    private double getDeltaY(){

        //
        // yDot = sin(omega * dt)(x - ICCx) + cos(omega * dt)(y - ICCy) + ICCy
        //

        double[] icc = calculateICC();

        return (  Math.sin(calculateOmega() * K_TIMESTEP) * (mXpos - icc[0])
                - Math.cos(calculateOmega() * K_TIMESTEP) * (mYpos - icc[1])
                + icc[1] );
    }

    /**
     * Calculate ICC Method.
     *
     * Calculates the coordinates of the Instantaneous Center of Curvature
     *
     * @return Index 0 is the X coordinate of the ICC; Index 1 is the Y coordinate of the ICC
     */
    private double[] calculateICC() {

        //
        // ICC = [x - R sin(theta), y + R cos(theta)]
        //

        return new double[]{
                mXpos - calculateR() * Math.sin(mHeading),
                mYpos + calculateR() * Math.cos(mHeading)
        };
    }

    /**
     * Calculate R Method.
     *
     * Calculates the radius from the wheelBase center to the Instantaneous Center of Curvature
     *
     * @return The radius of the turning arc.
     */
    private double calculateR(){

        //     l (Vl + Vr)
        // R = ------------
        //      2(Vr - Vl)

        return ((kWheelBase * (getLeftWheelVelocity() + getRightWheelVelocity())) /
                2 * (getRightWheelVelocity() - getLeftWheelVelocity()));

    }

    /**
     * Calculate Omega Method.
     *
     * Calculates the average velocity travelled by both sides. Necessary for other calculations
     *
     * @return The average velocity travelled by both sides of the drivetrain.
     */
    private double calculateOmega(){

        //         Vr - Vl
        // omega = -------
        //            l

        return ((getRightWheelVelocity() - getLeftWheelVelocity()) / kWheelBase);
    }

    /**
     * Get Delta Heading Method.
     *
     * Calculates the change in the Heading of the Robot with respect to wheel velocities.
     *
     * @return The change in heading.
     */
    @Deprecated
    private double getDeltaHeading(){
        return (kRadius/kWheelBase) * (getRightWheelVelocity() - getLeftWheelVelocity());
    }

    /**
     * Get Left Wheel Velocity Method.
     *
     * @return The current measured velocity of the left wheel.
     */
    private static double getLeftWheelVelocity(){
        //TODO: Overwrite this declaration
       return Main.getNextVelocities()[0];
    }

    /**
     * Get Left Wheel Velocity Method.
     *
     * @return The current measured velocity of the right wheel.
     */
    private static double getRightWheelVelocity(){
        //TODO: Overwrite this declaration
        return Main.getNextVelocities()[1];
    }
}
