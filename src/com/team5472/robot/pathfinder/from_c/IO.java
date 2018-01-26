package com.team5472.robot.pathfinder.from_c;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Path;

public class IO {

    private static byte[] intToBytes(int n){
        byte[] toReturn = new byte[4];
        toReturn[0] = (byte)((n >> 24) & 0xFF);
        toReturn[1] = (byte)((n >> 16) & 0xFF);
        toReturn[2] = (byte)((n >> 8) & 0xFF);
        toReturn[3] = (byte)(n & 0xFF);
        return toReturn;
    }

    private static int bytesToInt(byte[] bytes){
        return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
    }

    private static byte[] longToBytes(long n){
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

    private static long bytesToLong(byte[] bytes){
        return ((long)bytes[0] << 56) |
                ((long)bytes[0] << 48) |
                ((long)bytes[0] << 40) |
                ((long)bytes[0] << 32) |
                ((long)bytes[0] << 24) |
                ((long)bytes[0] << 16) |
                ((long)bytes[0] << 8) |
                ((long)bytes[0]);
    }

    private static byte[] doubleToBytes(double d){
        return longToBytes(Double.doubleToLongBits(d));
    }

    private static double bytesToDouble(byte[] bytes){
        return Double.longBitsToDouble(bytesToLong(bytes));
    }


    public static void serialize(Path filePath, Segment[] trajectory){
        try {
            File file = filePath.toFile();
            if(file.exists()) file.delete();
            file.createNewFile();
            FileOutputStream fos = new FileOutputStream(file);
            fos.write(intToBytes(trajectory.length));

            for(Segment s : trajectory) {
                fos.write(doubleToBytes(s.dt));
                fos.write(doubleToBytes(s.x));
                fos.write(doubleToBytes(s.y));
                fos.write(doubleToBytes(s.position));
                fos.write(doubleToBytes(s.velocity));
                fos.write(doubleToBytes(s.acceleration));
                fos.write(doubleToBytes(s.jerk));
                fos.write(doubleToBytes(s.heading));
            }

            fos.close();
        } catch (IOException e){
            e.printStackTrace();
            System.out.println("Failed to write trajectory to file.");
        }

    }

    public static Segment[] deserialize(Path filePath){

        File toFile = filePath.toFile();
        if(!toFile.exists()) return new Segment[]{};

        try{

            FileInputStream fis = new FileInputStream(toFile);
            byte[] buffer = new byte[4];
            byte[] dwordBuffer = new byte[8];

            fis.read(buffer);
            int trajectoryLength = bytesToInt(buffer);
            Segment[] traj = new Segment[trajectoryLength];

            for(int i = 0; i < trajectoryLength; i++){

                Segment seg = new Segment();

                fis.read(dwordBuffer);
                seg.dt = bytesToDouble(dwordBuffer);

                fis.read(dwordBuffer);
                seg.x = bytesToDouble(dwordBuffer);

                fis.read(dwordBuffer);
                seg.y = bytesToDouble(dwordBuffer);

                fis.read(dwordBuffer);
                seg.position = bytesToDouble(dwordBuffer);

                fis.read(dwordBuffer);
                seg.velocity = bytesToDouble(dwordBuffer);

                fis.read(dwordBuffer);
                seg.acceleration = bytesToDouble(dwordBuffer);

                fis.read(dwordBuffer);
                seg.jerk = bytesToDouble(dwordBuffer);

                fis.read(dwordBuffer);
                seg.heading = bytesToDouble(dwordBuffer);

                traj[i] = seg;
            }

            fis.close();
            return traj;
        } catch(IOException e) {
            e.printStackTrace();
        }

        return new Segment[]{};
    }

}
