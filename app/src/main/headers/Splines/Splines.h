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

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

static void printProgress(double percentage) {
  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);
}

namespace Splines {
  struct Waypoint {
    long double x = 0, y = 0; // you'd be surprised how much this breaks if it's not long double...
    double segLength = 0; // length of segment
    double totalLength = 0; // length of everything up to and including this segment
  };

  struct Spline {
    std::vector<Waypoint> points;
    double totalLength = 0;
    int segmentNum = 0;
  };

  namespace Util {
    /**
     * Convert Radians to Degrees
     */
    static double r2d(double rad) {
      return (rad * 180 / M_PI);
    }

    /**
     * Convert degrees to radians
     */
    static double d2r(double deg) {
      return (deg * M_PI / 180);
    }
  };

  /**
   * Virtual Spline Base with common functions for splines
   */
  class SplineBase {
   public:
    /**
     * Get the spline point from t value and spline
     */
    virtual Waypoint getSplinePoint(float t, Spline spline) {};

    /**
     * Get gradient point from t value and spline
     */
    virtual Waypoint getSplineGradientPoint(float t, Spline spline) {};

    /**
     * Get angle in radians based from t value and spline
     */
    virtual double getSplineAngleRad(float t, Spline spline) {
      Waypoint gradient = getSplineGradientPoint(t, spline);
      return atan2(gradient.y, gradient.x);
    }

    /**
     * Get angle in degrees based from t value and spline
     */
    virtual double getSplineAngleDeg(double t, Spline spline) {
      return Util::r2d(getSplineAngleRad(t, spline));
    }

    /**
     * Calculate the length of a segment, using the node (int) and spline
     */
    virtual double calculateSegLength(int node, Spline spline) {
      Waypoint oldPoint, newPoint;
      oldPoint = getSplinePoint((float)node, spline);

      std::cout << "[Node " << node << "-" << node+1 << "]" << std::endl;
      std::vector<double> lengthBuffer;
      for (double t = 0.0; t < 1.0; t += stepSize) {
        newPoint = getSplinePoint((float)node + t, spline);
        double xrt = (newPoint.x - oldPoint.x)*(newPoint.x - oldPoint.x);
        double yrt = (newPoint.y - oldPoint.y)*(newPoint.y - oldPoint.y);
        double xyrt = (xrt+yrt);
        
        double bufferValue = 0;

        if (xyrt > 0) {
          bufferValue = sqrt(xyrt);
          if (isinf(bufferValue) || isnan(bufferValue)) {
            bufferValue = 0;
            std::cout << " -- Overflow detected, Debug Below -- " << std::endl;
            std::cout << "| New points x,y: (" << newPoint.x << "," << newPoint.y << ")" << std::endl;
            std::cout << "| Old points x,y: (" << oldPoint.x << "," << oldPoint.y << ")" << std::endl;
            std::cout << "| t value: " << t << std::endl;
            std::cout << "| XY rt was xrt: (" << xrt << ") & yrt: (" << yrt << ")" << std::endl;
            return -1;
          }
        } else {
          bufferValue = 0;
        }

        lengthBuffer.push_back(bufferValue);
        oldPoint = newPoint;
        printProgress(t);
      }

      printProgress(1);
      std::cout << " Complete" << std::endl;

      // Add the values up
      double totalLength = 0;
      for (size_t i = 0; i < lengthBuffer.size(); i++) {
        totalLength += lengthBuffer[i];
      }

      return totalLength;
    }

    /**
     * Calculate length of spline,
     * use remove nodes to increase the nodes to remove, (0 being none, 1 being 1 at the start and 1 at the end)
     * Used mostly for control points at the start and end of spline, (CatmullRom)
     * 
     * returns -1 if error
     */
    virtual int calculateSpline(Spline &spline, int removeNodes = 0) {
      int nodeNum = spline.points.size();
      std::cout << "-- Calculating Length of spline --" << std::endl;
      std::cout << "-- Total Nodes: " << nodeNum << std::endl;
      std::vector<double> bufferLength;
      for (size_t node = removeNodes; node < nodeNum - removeNodes; node++) {
        double segLength = calculateSegLength(node, spline);
        if (segLength == -1) {
          std::cout << "Segment Length Error" << std::endl;
          return -1;
        } else {
          bufferLength.push_back(segLength);
        }
      }

      for (size_t i = 0; i < bufferLength.size(); i++) {
        spline.points[i].segLength = bufferLength[i];
        spline.totalLength += bufferLength[i];
        spline.points[i].totalLength = spline.totalLength;
        if (spline.points[i].segLength > 0) {
          spline.segmentNum++;
          std::cout << "Segment " << i << "-" << i+1 << ", Length: " << spline.points[i].segLength << ", Length up to and including: " << spline.points[i].totalLength << std::endl; 
        }
      }

      std::cout << "Number of segments: " << spline.segmentNum << std::endl;

      return 0;
    }

    static double getTFromDistance(double distanceMeters, Splines::Spline spline) {
      // int segments = spline.segmentNum;
      int segmentNum = 0;

      // Determin segment number
      while (distanceMeters > spline.points[segmentNum].totalLength) {
        segmentNum++;
      }

      double distAlongSegment = 0;
      if (segmentNum != 0) {
        distAlongSegment = distanceMeters - spline.points[segmentNum-1].totalLength;
      } else {
        distAlongSegment = distanceMeters;
      }

      // scaledNum = ((x-a)*(d-c)/(b-a))+c
      // Redundant because it's range 0-segLength to 0-1 but it makes sense drawn out.
      double oldRange = (spline.points[segmentNum].segLength - 0);
      double newRange = (1 - 0);
      double t = (((distAlongSegment - 0) * newRange) / oldRange) + 0;

      return t+segmentNum;
    }

    static void setStepSize(double step) {
      stepSize = step;
    }

   private:
    inline static double stepSize = 0.0001;
  };
}

#endif