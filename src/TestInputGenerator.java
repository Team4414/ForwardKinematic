import java.io.File;
import java.util.ArrayList;

public class TestInputGenerator {
    double lVel;
    double rVel;

    private ArrayList<double[]> vels;
    private double[][] arrayVels;
    int timeCount;

    public static final double TIMESTEP = 0.05;

    private static TestInputGenerator instance;
    public static TestInputGenerator getInstance(){
        if (instance == null)
            instance = new TestInputGenerator();
        return instance;
    }

    private TestInputGenerator(){
        lVel = 0;
        rVel = 0;
        timeCount = 0;
        vels = new ArrayList<>();

        changeVel(0,0,0);
        changeVel(1, 2, 0.5);
        changeVel(9,10,1);
        changeVel(30, 2, 1.5);
        changeVel(25,8, 2.5);
        changeVel(8,10,4);
        changeVel(0,0,5);

        arrayVels = new double[vels.size()][vels.get(0).length];
        int xCnt= 0;
        for(double[] vals: vels){
            int yCnt = 0;
            for (double val : vals){
                arrayVels[xCnt][yCnt] = val;
                yCnt++;
            }
            xCnt++;
        }

        try {
            CSVHelper.writeCSV(arrayVels, new File("csv/testVelocityValues.csv"));
        } catch (Exception e){
            System.out.println("CSV didn't write properly");
        }

    }

    public double[][] getRawContent(){
        return arrayVels;
    }

    private void changeVel(double lVel, double rVel, double timeInSeconds){
        while(timeCount * TIMESTEP < timeInSeconds){
            timeCount ++;
            vels.add(new double[]{this.lVel, this.rVel});
        }
        this.lVel = lVel;
        this.rVel = rVel;
    }
}


