package com.team5472.robot.pathfinder.from_c;

import java.io.FileOutputStream;
import java.io.IOException;

public class IO {

    public static byte[] intToBytes(int n){
        byte[] toReturn = new byte[4];
        toReturn[0] = (byte)((n >> 24) & 0xFF);
        toReturn[1] = (byte)((n >> 16) & 0xFF);
        toReturn[2] = (byte)((n >> 8) & 0xFF);
        toReturn[3] = (byte)(n & 0xFF);
        return toReturn;
    }

    public static int bytesToInt(byte[] bytes){
        return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
    }

    public static byte[] longToBytes(long n){
        byte[] toReturn = new byte[8];
        toReturn[0] = (byte)((n >> 56) & 0xFF);
        toReturn[1] = (byte)((n >> 48) & 0xFF);
        toReturn[2] = (byte)((n >> 40) & 0xFF);
        toReturn[3] = (byte)((n >> 32) & 0xFF);
        toReturn[4] = (byte)((n >> 24) & 0xFF);
        toReturn[5] = (byte)((n >> 16) & 0xFF);
        toReturn[6] = (byte)((n >> 8) & 0xFF);
        toReturn[7] = (byte)(n & 0xFF);
        return toReturn;
    }

    public static byte[] doubleToBytes(double d){
        return longToBytes(Double.doubleToLongBits(d));
    }


    public void Serialize(FileOutputStream fos, Segment[] trajectory){
        try {
            fos.write(intToBytes(trajectory.length));

        } catch (IOException e){
            e.printStackTrace();
            System.out.println("Failed to write trajectory to file.");
        }

    }

}
