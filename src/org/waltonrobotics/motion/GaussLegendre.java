package org.waltonrobotics.motion;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.exp;

import java.util.Arrays;
import java.util.function.Function;

public class GaussLegendre {

	private double[] weights;
	private double[] nodes;
	private double lowerBound;
	private double upperBound;

	/**
	 * Constructs nodes and weights for an n-dimensional Gauss Legendre quadrature between on the interval [lowerBound,
	 * upperBound]. The constructor creates nodes and weights and then initializes all fields of the instance.
	 *
	 * @param numberOfPoints, lowerBound, upperBound
	 * @author Alessandro Gnoatto based on Matlab code by Greg von Winckel.
	 */
	public GaussLegendre(int numberOfPoints, double lowerBound, double upperBound) {

		numberOfPoints = numberOfPoints - 1;
		int n1 = numberOfPoints + 1;
		int n2 = numberOfPoints + 2;

		double h = 2.0 / (n1 - 1);

		double[] xu = new double[n1];
		for (int i = 0; i < n1; i++) {
			xu[i] = -1 + h * i;
		}

		//Initial guess
		double[] y = new double[numberOfPoints + 1];
		double[] y0 = new double[numberOfPoints + 1];
		for (int i = 0; i <= numberOfPoints; i++) {
			y[i] = Math.cos((2.0 * i + 1) * Math.PI / (2 * numberOfPoints + 2)) + 0.27 / n1 * Math
				.sin(Math.PI * xu[i] * numberOfPoints / n2);
			y0[i] = 2.0;

		}

		/*
		 * Compute the zeros of the N+1 Legendre Polynomial
		 * using the recursion relation and the Newton-Raphson method
		 */

		double eps = 2.22E-16;
		//Legendre-Gauss Vandermonde Matrix
		double[][] L = new double[n1][n2];
		double[] Lp = new double[n1];

		//Iterate until new points are uniformly within epsilon of old points
		while (maxDistance(y, y0) > eps) {

			for (int j = 0; j < n1; j++) {
				L[j][0] = 1.0;
				L[j][1] = y[j];
				y0[j] = y[j];
			}

			for (int k = 1; k < n1; k++) {
				for (int j = 0; j < n1; j++) {
					L[j][k + 1] = ((2.0 * (k + 1) - 1) * y[j] * L[j][k] - (k) * L[j][k - 1]) / (k + 1);
				}
			}

			for (int j = 0; j < n1; j++) {
				//Lp(j)=(N2)*( L(j,N1)-y(j).*L(j,N2) )./(1-y(j).^2);
				Lp[j] = (n2) * (L[j][n1 - 1] - y[j] * L[j][n2 - 1]) / (1 - y[j] * y[j]);
				//y(j)=y(j)-L(j,N2)./Lp(j);

				y[j] = y[j] - L[j][n2 - 1] / Lp[j];
			}

		}

		double[] x = new double[n1];
		double[] w = new double[n1];
		double[] xInverted = new double[n1];

		for (int j = 0; j < n1; j++) {
			//Linear map from[-1,1] to [a,b]
			//x=(a*(1-y)+b*(1+y))/2;
			x[j] = (lowerBound * (1 - y[j]) + upperBound * (1 + y[j])) / 2.0;
			xInverted[n1 - 1 - j] = x[j]; //TODO Change this to make the nodes reversed
			//corresponding weights
			//w=(b-a)./((1-y.^2).*Lp.^2)*(N2/N1)^2;
			w[j] = (upperBound - lowerBound) / ((1 - y[j] * y[j]) * Lp[j] * Lp[j]) * Math.pow(((double) n2) / n1, 2);
		}

		this.lowerBound = lowerBound;
		this.upperBound = upperBound;
		this.weights = w;
		this.nodes = xInverted;
	}

	public static void main(String[] args) {

		int n = 5;
		GaussLegendre gl = new GaussLegendre(n, -1, 1);

		System.out.println("Gauss Legendre nodes for n =" + n + " between 0 and 1");
		for (int j = 0; j < n; j++) {
			System.out.println(gl.getNodes()[j]);
		}
		System.out.println("Gauss Legendre weights for n =" + n + " between 1 and 1");
		for (int j = 0; j < n; j++) {
			System.out.println(gl.getWeights()[j]);
		}

	}

	public double[] getWeights() {
		return this.weights;
	}

	public double[] getNodes() {
		return this.nodes;
	}

	public double getLowerBound() {
		return this.lowerBound;
	}

	public double getUpperBound() {
		return this.upperBound;
	}

	/**
	 * @return max(abs ( a[] - b[]))
	 */
	private double maxDistance(double[] a, double[] b) {

		int n = a.length;
		double[] distance = new double[n];

		for (int i = 0; i < n; i++) {
			distance[i] = Math.abs(a[i] - b[i]);
		}
		//sort element of the array
		Arrays.sort(distance);
		//return the largest element which is stored at the end
		return distance[distance.length - 1];
	}


}

class Test {

	final static int N = 5;

	static double[] lroots = new double[N];
	static double[] weight = new double[N];
	static double[][] lcoef = new double[N + 1][N + 1];

	static void legeCoef() {
		lcoef[0][0] = lcoef[1][1] = 1;

		for (int n = 2; n <= N; n++) {

			lcoef[n][0] = -(n - 1) * lcoef[n - 2][0] / n;

			for (int i = 1; i <= n; i++) {
				lcoef[n][i] = ((2 * n - 1) * lcoef[n - 1][i - 1]
					- (n - 1) * lcoef[n - 2][i]) / n;
			}
		}
	}

	static double legeEval(int n, double x) {
		double s = lcoef[n][n];
		for (int i = n; i > 0; i--) {
			s = s * x + lcoef[n][i - 1];
		}
		return s;
	}

	static double legeDiff(int n, double x) {
		return n * (x * legeEval(n, x) - legeEval(n - 1, x)) / (x * x - 1);
	}

	static void legeRoots() {
		double x, x1;
		for (int i = 1; i <= N; i++) {
			x = cos(PI * (i - 0.25) / (N + 0.5));
			do {
				x1 = x;
				x -= legeEval(N, x) / legeDiff(N, x);
			} while (x != x1);

			lroots[i - 1] = x;

			x1 = legeDiff(N, x);
			weight[i - 1] = 2 / ((1 - x * x) * x1 * x1);
		}
	}

	static double legeInte(Function<Double, Double> f, double a, double b) {
		double c1 = (b - a) / 2, c2 = (b + a) / 2, sum = 0;
		for (int i = 0; i < N; i++) {
			sum += weight[i] * f.apply(c1 * lroots[i] + c2);
		}
		return c1 * sum;
	}

	public static void main(String[] args) {
		legeCoef();
		legeRoots();

		System.out.print("Roots: ");
		for (int i = 0; i < N; i++) {
			System.out.printf(" %f", lroots[i]);
		}

		System.out.print("\nWeight:");
		for (int i = 0; i < N; i++) {
			System.out.printf(" %f", weight[i]);
		}

		System.out.printf("%nintegrating Exp(x) over [-3, 3]:%n\t%10.8f,%n"
				+ "compared to actual%n\t%10.8f%n",
			legeInte(x -> exp(x), -3, 3), exp(3) - exp(-3));
	}
}
