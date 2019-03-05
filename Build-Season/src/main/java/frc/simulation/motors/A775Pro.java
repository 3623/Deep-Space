package frc.simulation.motors;

/**
 * CIM Motor model, implements Motor abstract class 
 * 
 * @author eric
 *
 */
public class A775Pro extends Motor{
	
	public A775Pro () {
		STALL_TORQUE = 0.7; // N. m
		FREE_SPEED = 21020.0; // rpm
		STALL_CURRENT = 130.1;
		FREE_CURRENT = 3.8;
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
