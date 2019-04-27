package frc.util;

public class Tuple {
	public double left;
	public double right;
	
	public Tuple(double left, double right) {
		this.left = left;
		this.right = right;
	}

	public Tuple scale (double factor){
		return new Tuple(this.left*factor, this.right*factor);
	}
}
