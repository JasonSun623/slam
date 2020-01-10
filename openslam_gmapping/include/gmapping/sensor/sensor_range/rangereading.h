#ifndef RANGEREADING_H
#define RANGEREADING_H

#include <vector>
#include <gmapping/sensor/sensor_base/sensorreading.h>
#include "gmapping/sensor/sensor_range/rangesensor.h"

namespace GMapping{

class RangeReading: public SensorReading, public std::vector<double>{
	public:
		RangeReading(const RangeSensor* rs, double time=0);
		RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time=0);
		virtual ~RangeReading();
		inline const OrientedPoint& getPose() const {return m_pose;}
		inline void setPose(const OrientedPoint& pose) {m_pose=pose;}

		/**
		 * @brief  Copy the scanned data into an array
		 * 
		 * @param v 
		 * @param density Filtering parameter. 
		 * If the distance between the points detected by several adjacent scan data is less than this parameter, a large value is assigned.
		 * @return unsigned int 
		 */
		unsigned int rawView(double* v, double density=0.) const;

		std::vector<Point> cartesianForm(double maxRange=1e6) const;
		unsigned int activeBeams(double density=0.) const;
	protected:
		OrientedPoint m_pose;
};

};

#endif
