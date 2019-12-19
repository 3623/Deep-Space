package frc.robot.subsystems;

public class PixyPacket {
	int sync = 0;
	int checksum = 0;
	public int signature;
	public int xCenter;
	public int yCenter;
	public int width;
	public int height;
	
	//public int checksumError;
	
	public String toString() {
		return "" +
	" S:" + signature +
	" X:" + xCenter + 
	" Y:" + yCenter +
	" W:" + width + 
	" H:" + height;
	}
}