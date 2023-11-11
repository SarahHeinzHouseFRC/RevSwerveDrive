package frc.utils;

public class AutoUtils {

    public static int autoStep = 0;

    public static boolean aproxEqual(double valueA, double valueB) {
        return aproxEqual(valueA, valueB, 0.1);
    }

    public static boolean aproxEqual(double valueA, double valueB, double limit) {
        return (valueA < valueB + limit) && (valueA > valueB - limit);
    }

}