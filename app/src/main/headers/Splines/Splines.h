#ifndef SPLINES_H
#define SPLINES_H

#include <vector>
#include <math.h>

#ifdef __linux__ 
  #include <bits/stdc++.h>
#elif _WIN32
  #include <iomanip>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

namespace Splines {
  struct Waypoint {
    double x,y;
    double segLength = 0;
  };

  struct Spline {
    std::vector<Waypoint> points;
    double totalLength;
  };

  namespace Util {
    static double r2d(double rad) {
      return (rad * 180 / M_PI);
    }

    static double d2r(double deg) {
      return (deg * M_PI / 180);
    }
  };

  /**
   * Virtual Spline Base with common functions for splines
   */
  class SplineBase {
   public:
    virtual Waypoint getSplinePoint(float t, Spline spline) {};
    virtual Waypoint getSplineGradientPoint(float t, Spline spline) {};

    virtual double getSplineAngleRad(double t, Spline spline) {
      Waypoint gradient = getSplineGradientPoint(t, spline);
      return atan2(gradient.y, gradient.x);
    }

    virtual double getSplineAngleDeg(double t, Spline spline) {
      return Util::r2d(getSplineAngleRad(t, spline));
    }

    virtual double calculateSegLength(int node, Spline spline) {
      Waypoint oldPoint, newPoint;
      oldPoint = getSplinePoint((double)node, spline);

      std::vector<float> lengthBuffer;
      for (double t = 0.0; t < 1.0; t += stepSize) {
        newPoint = getSplinePoint((float)node + t, spline);
        float xrt = (newPoint.x - oldPoint.x)*(newPoint.x - oldPoint.x);
        float yrt = (newPoint.y - oldPoint.y)*(newPoint.y - oldPoint.y);
        float xyrt = (xrt+yrt);
        
        float bufferValue = 0;

        if (xyrt > 0) {
          bufferValue = sqrt(xyrt);
          if (isinf(bufferValue)) {
            bufferValue = 0;
            std::cout << "It broke" << std::endl;
          }
        } else {
          bufferValue = 0;
        }

        lengthBuffer.push_back(bufferValue);
        oldPoint = newPoint;
      }

      // Add the values up
      double totalLength = 0;
      for (size_t i = 0; i < lengthBuffer.size(); i++) {
        totalLength += lengthBuffer[i];
      }

      std::cout << "Seg Length: " << totalLength << std::endl;
      return totalLength;
    }

    /**
     * Calculate length of spline,
     * use remove nodes to increase the nodes to remove, (0 being none, 1 being 1 at the start and 1 at the end)
     * Used mostly for control points at the start and end of spline, (CatmullRom)
     */
    virtual void calculateSpline(Spline &spline, int removeNodes = 0) {
      int nodeNum = spline.points.size();
      std::cout << "-- Calculating Length of spline --" << std::endl;

      std::vector<double> bufferLength;
      for (size_t node = removeNodes; node < nodeNum - removeNodes; node++) {
        bufferLength.push_back(calculateSegLength(node, spline));
      }

      for (size_t i = 0; i < bufferLength.size(); i++) {
        spline.points[i].segLength = bufferLength[i];
        spline.totalLength += bufferLength[i];
        std::cout << "Node " << i << "-" << i+1 << ": Segment Length: " << spline.points[i].segLength << std::endl; 
      }
    }

    static void setStepSize(double step) {
      stepSize = step;
    }

   private:
    inline static double stepSize = 0.005;
  };
}

#endif