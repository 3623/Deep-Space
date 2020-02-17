package frc.util;

public class Tuple {
	public double left;
	public double right;

	public Tuple(double left, double right) {
		this.left = left;
		this.right = right;
	}

	public Tuple scale(double factor) {
		this.left *= factor;
		this.right *= factor;
		return this;
	}

	public Tuple difference(Tuple subtrahend) {
		double a = this.left - subtrahend.left;
		double b = this.right - subtrahend.right;
		return new Tuple(a, b);
	}

	public Tuple difference(double left, double right) {
		return difference(new Tuple(left, right));
	}

	public String toString() {
		return this.left + ", " + this.right;
	}
}
