package frc.robot;

/**
 *
 */
class Sensitivity {
    double threshold;
    double linCoef;
    double cuspX;
    double limit;
    double a, b, c;

    public Sensitivity(double th, double cx, double lc, double li) {
        set(th, cx, lc, li);
    }

    public void set(double th, double cx, double lc, double li) {
        threshold = th;
        cuspX = cx;
        linCoef = lc;
        limit = li;

        double x0 = cuspX;
        double g = (2.0 - x0) * x0;
        double denom = 1.0 - g;
        a = (1.0 - linCoef) / denom;
        c = a * x0 * x0;
        b = (linCoef + (linCoef * x0 - 2.0) * x0) / denom;
    }

    double transfer(double x) {
        double xabs = Math.abs(x);
        if (threshold < 0) {
            xabs -= threshold;
            if (xabs <= 0)
              return 0;
        }
        else if (xabs < threshold)
          return 0;

        if (xabs <= cuspX) {
            //System.out.println("Lin " + m_linCoef + " x: " + x);
            xabs *= linCoef;
        }
        else {
            //if (xabs <= 1.0) {
            //System.out.println("Quad " + m_threshold + " x: " + x);
            xabs = a * xabs * xabs + b * xabs + c;
        }
        if (xabs > limit)
            xabs = limit;
        // if (xabs < minLimit)
        //     xabs = limit;
// /        System.out.println("SENS in: " + x + " out: " + xabs);
        return x >= 0 ? xabs : -xabs;
    }

    public static double minLimit(double value, double minValue) {
      double abs = Math.abs(value);
      double sign = value > 0 ? 1 : -1;
      return abs > 0 && abs < minValue ? sign * minValue : value;
    }
}

/*
double sensitivity(double x) {
    //m_speedLimit1 = SmartDashboard.getNumber("Drive speed limit1", 0.1);

    double xabs = Math.abs(x);
    if (xabs <= m_threshold) {
      return m_linCoef * x;
    }
    else {//if (xabs <= 1.0) {
      // double x2 = (xabs - m_speedStep1);
      // x2 *= x2;

      double c = m_linCoef * m_cuspX;
      double coef = (1 - c) / (1 - m_threshold);
      coef *= m_speedLimitX;
      xabs = coef * (xabs - m_threshold) + c;
      x = x >= 0 ? xabs : -xabs;
    }
    // else if (xabs <= 0.6) {
    //   // 0.32 = 0.4 * k
    //   //x = 0.4 * (x - 0.2) + 0.04;
    // }
    // else {
    //   // 0.2
    //   //x = m_speedLimit1 * (x - 0.6) + 0.2;
    // }
    return x;
  }

  static double sensitivity3(double x) {
    double xabs = Math.abs(x);
    if (xabs < 0.1)
      return 0;
    else if (xabs < 0.6)
      return 0.4 * Math.signum(x) * (xabs);
    else return 0;
    //return 0.5 * x * x * x;
  }
}
*/