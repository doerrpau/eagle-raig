import java.io.*;
import java.util.*;
import java.lang.*;

public class Main
{

    public static void main(String args[]) throws IOException
    {
        RAIGDriver driver = new RAIGDriver();
        while (true) {
            System.out.println(driver.lsm_data.size());
        }
    }
}
