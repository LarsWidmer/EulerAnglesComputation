#ifndef STREAMTRANSFORMER_H
#define STREAMTRANSFORMER_H

#include "Defines.h"
#include <math.h>
#include <iostream>
using namespace std;

/**
 * Transforms NodeData to three euler angles,
 * used to represent 360 rotation around each euler axis.
 * DirectX uses a left handed coordinate system!
 *
 * Currently unused class?
 * The idea was to encapsulate the transformation of the raw sensor data
 * to euler angles in this class.
 * Currently all this stuff is directly done in SerialData.
 * SerialData directly accesses StreamReader.
 */
class StreamTransformer
{
        ///////////////////////////////////////////////////////////
        //Transforms NodeData into three euler angles.           //
        //Pass the calibration data as second argument.          //
        ///////////////////////////////////////////////////////////
		public: EulerRotation transform(NodeData newData, NodeData calibrationData);

        ///////////////////////////////////////////////////////////
        //Transforms NodeData into three euler angles.           //
        ///////////////////////////////////////////////////////////
		public: EulerRotation transform(NodeData newData);

		///////////////////////////////////////////////////////////
        //class constructor                                      //
        ///////////////////////////////////////////////////////////
		public: StreamTransformer();


		///////////////////////////////////////////////////////////
        //class destructor                                       //
        ///////////////////////////////////////////////////////////
		public: ~StreamTransformer();


		///////////////////////////////////////////////////////////
        //A function used to get the polar angle between         //
        //(1,0) and (x,y).                                       //
        ///////////////////////////////////////////////////////////
		public: static float getPolarAngle(float x, float y);
};

#endif // STREAMTRANSFORMER_H
