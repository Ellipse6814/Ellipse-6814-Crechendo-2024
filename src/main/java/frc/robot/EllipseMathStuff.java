//math stuff that i dont wanna write in each file seperately
package frc.robot;

public final class EllipseMathStuff {
  public static double negative180_180_to_0_360(double angle)
  {
    if(angle < 0)
    {
      return angle + 360;
    }
    else
    {
      return angle;
    }
  }

  public static double rad_negative180_180_to_0_360(double rad_angle)
  {
    if(rad_angle < 0)
    {
      return rad_angle + Math.toRadians(360);
    }
    else
    {
      return rad_angle;
    }
  }

  public static double angle_to_negative90_90(double deg_angle)
  {
    double correctedAngle = negative180_180_to_0_360(deg_angle);

    if(correctedAngle > 90)
    {
      correctedAngle -= 270.0;
    } 
    else
    {
      correctedAngle -= 90.0;
    }

    return correctedAngle;
  }

  public static double rad_angle_to_negative90_90(double rad_angle)
  {
    return Math.toRadians(angle_to_negative90_90(Math.toDegrees(rad_angle)));
  }
}
