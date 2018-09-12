package org.waltonrobotics.util;

import java.util.HashMap;

public class Helper {

  public final static HashMap<Integer, int[]> coefficents = new HashMap<>();

  static {
    coefficents.put(0, new int[]{1});
    coefficents.put(1, new int[]{1, 1});
    coefficents.put(2, new int[]{1, 2, 1});
    coefficents.put(3, new int[]{1, 3, 3, 1});
    coefficents.put(4, new int[]{1, 4, 6, 4, 1});
    coefficents.put(5, new int[]{1, 5, 10, 10, 5, 1});
    coefficents.put(6, new int[]{1, 6, 15, 20, 15, 6, 1});
    coefficents.put(7, new int[]{1, 7, 21, 35, 35, 21, 7, 1});
    coefficents.put(8, new int[]{1, 8, 28, 56, 70, 56, 28, 8, 1});
    coefficents.put(9, new int[]{1, 9, 36, 84, 126, 126, 84, 36, 9, 1});
  }

  /**
   * Finds the factorial of any integer or double, d
   *
   * @return the factorial of d
   */

  /**
   * Calculates the binomial coefficients for the demanded path degree
   */
  public static int[] calculateCoefficients(int degree) {
    if (coefficents.containsKey(degree)) {
      return coefficents.get(degree);
    }

    int[] coefficients = new int[degree + 1];
    for (int i = 0; i < coefficients.length; i++) {
      coefficients[i] = Math.toIntExact(findNumberOfCombination(degree, i));
    }

    coefficents.put(degree, coefficients);

    return coefficients;
  }

  /**
   * Uses the formula to find the value of nCr
   *
   * @return nCr
   */
  private static long findNumberOfCombination(int n, int r) {
    int nFactorial = factorial(n);
    int rFactorial = factorial(r);
    int nMinusRFactorial = factorial(n - r);

    return nFactorial / (rFactorial * nMinusRFactorial);
  }

  /**
   * Finds the factorial of any integer d
   *
   * @return the factorial of d
   */
  private static int factorial(int d) {
    if (d >= 13) {
      throw new ArithmeticException(String
          .format(
              "The number %d is too big of a number to factorize as it will cause integer overflow the maximum is 12",
              d));
    }

    int result = 1;
    for (int i = 1; i <= d; i++) {
      result = result * i;
    }

    return result;
  }

}
