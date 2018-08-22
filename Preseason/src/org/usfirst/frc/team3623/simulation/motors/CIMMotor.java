package org.usfirst.frc.team3623.simulation.motors;

/**
 * CIM Motor model, implements Motor abstract class 
 * 
 * @author eric
 *
 */
public class CIMMotor extends Motor{
	
	public CIMMotor () {
		STALL_TORQUE = 2.41; // N. m
		FREE_SPEED = 5330.0; // rpm
		calculateSlopes();
	}
	
	// Testing calculations
	public static void main ( String[] args )
	  {
	      CIMMotor cim = new CIMMotor();
	      System.out.println(cim.outputTorque(12, 5330));
	      System.out.println(cim.outputTorque(12, 0));
	      System.out.println(cim.outputTorque(12, 2000));
	      System.out.println(cim.outputTorque(12, -5330));
	  }
}
