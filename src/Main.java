import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class Main {

    private static int csvIterator = 0;

    private static int contentLength = TestInputGenerator.getInstance().getRawContent().length;

    public static void main(String[] args){
        TestInputGenerator.getInstance();
        System.out.println("Done!");

        ArrayList<double[]> kineOutput = new ArrayList<>();

        ForwardKinematics kinematics = new ForwardKinematics(0.33, 3);

//        System.out.println(TestInputGenerator.getInstance().getRawContent().length);
//        System.out.println(csvIterator);
        while(csvIterator < contentLength){
            csvIterator++;
            System.out.println(csvIterator + "\t" + contentLength);
            kinematics.update();

            kineOutput.add(new double[]{kinematics.getXPosition(), kinematics.getYPosition(), kinematics.getHeading()});
        }

        double[][] kineOutputAsArray = new double[kineOutput.size()][kineOutput.get(0).length];

        for(int i = 0; i < kineOutput.size(); i++){
            kineOutputAsArray[i] = kineOutput.get(i);
        }

        try {
            CSVHelper.writeCSV(kineOutputAsArray, new File("csv/output.csv"));
        } catch (IOException e){
            System.out.println("!!! Main Could Not Write CSV File !!!");
        }
    }

    public static double[] getNextVelocities(){
        try {
            return CSVHelper.parseCSV(new File("csv/testVelocityValues.csv"))[csvIterator - 1];
        }catch (IOException e){
            System.out.println("!!! Main was Unable to Parse CSV !!!");
            return new double[]{0,0};
        }
    }
}
