/*
 * realtimeStatistics.h
 *
 *  Created on: 01/ott/2013
 *      Author: Murtas Matteo
 */

#ifndef REALTIMESTATISTICS_H_
#define REALTIMESTATISTICS_H_

class realtimeStatistics {
public:
	realtimeStatistics();
	virtual ~realtimeStatistics();

	void clear();
	void append(float v);
	float mean();
	float sqdev();
	float stddev();
	float vmin();
	float vmax();
protected:
	unsigned int m_count;
	float m_mean;
	float m_sqdev;
	float m_min;
	float m_max;
};

#endif /* REALTIMESTATISTICS_H_ */
