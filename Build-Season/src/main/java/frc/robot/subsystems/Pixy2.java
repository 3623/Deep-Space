/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.I2C;

/**
 * From http://www.cmucam.org/boards/9/topics/3120
 */

public class Pixy2 {
    private static final int FRAME_SIZE = 14;
    private static final int MAX_FRAMES = 4;
    private GhettoI2C i2cDevice;
    List<PixyPacket> frames;

    public Pixy2(){
        i2cDevice = new GhettoI2C(GhettoI2C.Port.kMXP, 0x54);
        List<PixyPacket> frames = new ArrayList<PixyPacket>();
    }

    public PixyPacket getFrame(byte[] bytes) {
        PixyPacket PixyPacket = new PixyPacket();
        PixyPacket.sync = convertBytesToInt(bytes[1], bytes[0]);
        // System.out.println("\nsync: "+Integer.toHexString(PixyPacket.sync));
        PixyPacket.checksum = convertBytesToInt(bytes[3], bytes[2]);

        // if the checksum is 0 or the checksum is a sync byte, then there
        // are no more frames.
        if (PixyPacket.checksum == 0 || PixyPacket.checksum == 0xaa55) {
            return null;
        }
        PixyPacket.signature = convertBytesToInt(bytes[5], bytes[4]);
        PixyPacket.xCenter = convertBytesToInt(bytes[7], bytes[6]);
        PixyPacket.yCenter = convertBytesToInt(bytes[9], bytes[8]);
        PixyPacket.width = convertBytesToInt(bytes[11], bytes[10]);
        PixyPacket.height = convertBytesToInt(bytes[13], bytes[12]);

        return PixyPacket;
    }

    public List<PixyPacket> readBlocks() throws IOException {

        byte[] bytes = new byte[FRAME_SIZE * MAX_FRAMES];
        if (frames == null){
            frames = new ArrayList<PixyPacket>();
            System.out.println("LIST NOT INITIALIZED COCKSUCKER");
        }
        frames.clear();
        // read data from pixy

        Boolean bytesReadAborted = true;

        try {
            bytesReadAborted = i2cDevice.readOnly(bytes, bytes.length);
        } catch (Exception e) {
            System.out.println("==================" + e);
        }
        if(bytesReadAborted){
            System.out.println("==================" + "i2c read aborted");
            i2cDevice.restart();
        } else{
            System.out.println("New Frame Received");
        }

        // search for sync
        Boolean foundSync = false;
        for (int byteOffset = 0; byteOffset < bytes.length - (FRAME_SIZE - 1);) {

            int b1 = bytes[byteOffset];
            if (b1 < 0) b1+=256;
        
            int b2 = bytes[byteOffset+1];
            if (b2 < 0) b2+=256;

            if (b1 == 0x55 && b2 == 0xaa) {
                // found sync
                System.out.println("FOUND SYNC AT: " + byteOffset);
                foundSync = true;

                byte[] tempBytes = new byte[FRAME_SIZE]; //Creates a temp array for reading block
                for (int tempByteOffset = 0; tempByteOffset < FRAME_SIZE; tempByteOffset++) {
                    tempBytes[tempByteOffset] = bytes[byteOffset
                            + tempByteOffset];
                }

                PixyPacket PixyPacket = getFrame(tempBytes);
                if (PixyPacket != null) {
                    // it was a valid PixyPacket!
                    frames.add(PixyPacket);
                    System.out.println("Valid Packet Found: " + PixyPacket.toString());
                    // skip to next PixyPacket -1 as byteOffset will be incremented at the end of the loop block
                    byteOffset += FRAME_SIZE - 1;
                } else {
                    // it wasn't a valid PixyPacket, we can skip 2 bytes
                    System.out.println("Invalid Packet Found");
                    byteOffset++;
                }
            }
            byteOffset++;
        } 

        System.out.println("Targets Found: " + frames.size());

        return frames;
    }

    public int convertBytesToInt(int msb, int lsb){
        // System.out.println(Integer.toHexString(msb)+" "+Integer.toHexString(lsb));
        if (msb < 0) msb += 256;
        int value = msb * 256;

        if (lsb < 0) {
            // lsb should be unsigned
            value += 256;
        }
        value += lsb;
        return value;
    }

    public static void main ( String[] args ) {
        List<PixyPacket> frames = new ArrayList<PixyPacket>();
        System.out.println(frames.size());
    }

}