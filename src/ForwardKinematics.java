public class ForwardKinematics {

    private final double kRadius;
    private final double kWheelBase;

    private static final double K_TIMESTEP = 0.05;

    private double mXpos;
    private double mYpos;
    private double mHeading;

    public ForwardKinematics(double wheelRadius, double wheelBaseLength){
        kRadius = wheelRadius;
        kWheelBase = wheelBaseLength;

        mXpos = 0;
        mYpos = 0;
        mHeading = 0;
    }

    public ForwardKinematics(double wheelRadius, double wheelBaseLength, double xPos, double yPos, double heading){
        this(wheelRadius, wheelBaseLength);

        mXpos = xPos;
        mYpos = yPos;
        mHeading = heading;
    }

    public void update(){
        mXpos += K_TIMESTEP * getDeltaX();
        mYpos += K_TIMESTEP * getDeltaY();
        mHeading += K_TIMESTEP * getDeltaHeading();
    }

    public double getXPosition(){ return mXpos; }

    public double getYPosition(){ return mYpos; }

    public double getHeading(){ return mHeading; }

    private double getDeltaX(){
        return (kRadius/2) * (getLeftWheelVelocity() + getRightWheelVelocity()) * Math.cos(mHeading);
    }

    private double getDeltaY(){
        return (kRadius/2) * (getLeftWheelVelocity() + getRightWheelVelocity()) * Math.sin(mHeading);
    }

    private double getDeltaHeading(){
        return (kRadius/kWheelBase) * (getRightWheelVelocity() - getLeftWheelVelocity());
    }

    private static double getLeftWheelVelocity(){
       return Main.getNextVelocities()[0];
    }

    private static double getRightWheelVelocity(){
        return Main.getNextVelocities()[1];
    }
}
