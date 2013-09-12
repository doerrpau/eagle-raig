import java.io.*;
import java.util.*;
import java.lang.*;

public class RawPrint
{

    public static void main(String args[]) throws IOException
    {
        RAIGDriver driver = RAIGDriver.getSingleton();
        while (true) {
            if (!driver.lsm_data.isEmpty()) {
                long time = driver.lsm_data.getFirst().timestamp;
                for (int i = 0; i < driver.lsm_data.getFirst().samples.size(); i++) {
                    RAIGDriver.IMUSample lsm_samp = driver.lsm_data.getFirst().samples.get(i);
                    System.out.println("LSM" + lsm_samp.id + ": " + 
                            lsm_samp.rate[0] + " " +
                            lsm_samp.rate[1] + " " +
                            lsm_samp.rate[2] + " " +
                            time
                            );
                }
                driver.lsm_data.clear();
            }
            if (!driver.mpu_data.isEmpty()) {
                long time = driver.mpu_data.getFirst().timestamp;
                for (int i = 0; i < driver.mpu_data.getFirst().samples.size(); i++) {
                    RAIGDriver.IMUSample mpu_samp = driver.mpu_data.getFirst().samples.get(i);
                    System.out.println("MPU" + mpu_samp.id + ": " + 
                            mpu_samp.rate[0] + " " +
                            mpu_samp.rate[1] + " " +
                            mpu_samp.rate[2] + " " +
                            time
                            );
                }
                driver.mpu_data.clear();
            }
            try {
                Thread.sleep(1000);
            } 
            catch (Exception e) { return; }
        }
    }
}
