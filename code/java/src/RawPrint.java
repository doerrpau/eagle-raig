import java.io.*;
import java.util.*;
import java.lang.*;

public class RawPrint
{

    public static void main(String args[]) throws IOException
    {
        RAIGDriver driver = RAIGDriver.getSingleton();
        System.out.println("MPU 0 Z Axis,MPU 1 Z Axis,MPU 2 Z Axis,MPU 3 Z Axis");
        while (true) {
            if (!driver.lsm_data.isEmpty()) {
            /*    long time = driver.lsm_data.getFirst().timestamp;
                for (int i = 0; i < driver.lsm_data.getFirst().samples.size(); i++) {
                    RAIGDriver.IMUSample lsm_samp = driver.lsm_data.getFirst().samples.get(i);
                    System.out.println("LSM" + lsm_samp.id + ": " + 
                            lsm_samp.rate[0] + " " +
                            lsm_samp.rate[1] + " " +
                            lsm_samp.rate[2] + " " +
                            time
                            );
                }*/
                driver.lsm_data.clear();
            }
            if (!driver.mpu_data.isEmpty()) {
                long time = driver.mpu_data.getFirst().timestamp;
                for (int i = 0; i < driver.mpu_data.getFirst().samples.size(); i++) {
                    RAIGDriver.IMUSample mpu_samp = driver.mpu_data.getFirst().samples.get(i);
                    System.out.print(mpu_samp.rate[2]);
                    if (i+1 < driver.mpu_data.getFirst().samples.size()) {
                        System.out.print(",");
                    } else {
                        System.out.print("," + mpu_samp.temp + "\n");
                    }
                }
                driver.mpu_data.clear();
            }
            try {
                Thread.sleep(200);
            } 
            catch (Exception e) { return; }
        }
    }
}
