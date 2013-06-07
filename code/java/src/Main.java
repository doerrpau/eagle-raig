import java.io.*;
import java.util.*;
import java.lang.*;

public class Main
{

    public static void main(String args[]) throws IOException
    {
        RAIGDriver2 driver = new RAIGDriver2();
        while (true) {
            while (!driver.lsm_data.isEmpty()) {
                long time = driver.lsm_data.getFirst().timestamp;
                for (int i = 0; i < driver.lsm_data.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample lsm_samp = driver.lsm_data.getFirst().samples.get(i);
                    System.out.println("LSM" + lsm_samp.id + ": " + 
                            lsm_samp.rateX + " " +
                            lsm_samp.rateY + " " +
                            lsm_samp.rateZ + " " +
                            time
                            );
                }
                driver.lsm_data.clear();
            }
            while (!driver.mpu_data.isEmpty()) {
                long time = driver.mpu_data.getFirst().timestamp;
                for (int i = 0; i < driver.mpu_data.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample mpu_samp = driver.mpu_data.getFirst().samples.get(i);
                    System.out.println("MPU" + mpu_samp.id + ": " + 
                            mpu_samp.rateX + " " +
                            mpu_samp.rateY + " " +
                            mpu_samp.rateZ + " " +
                            time
                            );
                }
                driver.mpu_data.clear();
            }
            try {
                Thread.sleep(100);
            } 
            catch (Exception e) { return; }
        }
    }
}
