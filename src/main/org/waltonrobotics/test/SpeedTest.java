package org.waltonrobotics.test;

public class SpeedTest {

  public static final int ITERATIONS = Integer.MAX_VALUE/50;

  public static void main(String[] args) {
    String[] numbers = new String[ITERATIONS];
    for (int i = 0 ; i < ITERATIONS; i++)
    {
      numbers[i] = String.valueOf(i);
    }

    System.out.println("Finished Generating Numbers");
    long startTime = System.currentTimeMillis();
    for (int i = 0; i < ITERATIONS; i++) {
      Double.parseDouble(numbers[i]);
    }
    System.out.println("Double parse: " + (System.currentTimeMillis() - startTime));

    startTime = System.currentTimeMillis();
    for (int i = 0; i < ITERATIONS; i++) {
      Integer.parseInt(numbers[i]);
    }
    System.out.println("Integer parse: " + (System.currentTimeMillis() - startTime));

    startTime = System.currentTimeMillis();
    for (int i = 0; i < ITERATIONS; i++) {
      Integer.parseUnsignedInt(numbers[i]);
    }
    System.out.println("Unsigned Integer parse: " + (System.currentTimeMillis() - startTime));
  }

}
