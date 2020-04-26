#include "StreamTransformer.h" // class's header file

/**
 * Does the necessary vector algebra. 
 * Computes the euler angles out of the sensor data.
 * At first the calibration gets applied.
 */
EulerRotation StreamTransformer::transform(NodeData newData, NodeData calibrationData) {
	newData.accX -= calibrationData.accX;
	newData.accY -= calibrationData.accY;
	newData.accZ -= calibrationData.accZ;
	newData.magX -= calibrationData.magX;
	newData.magY -= calibrationData.magY;
	newData.magZ -= calibrationData.magZ;
	return transform(newData);
}

/**
 * Does the necessary vector algebra.
 * Computes the euler angles out of the sensor data.
 * The given NodeData is assumed to already have been calibrated.
 */
EulerRotation StreamTransformer::transform(NodeData newData) {
	EulerRotation result;
	result.xRotation = result.yRotation = result.zRotation = 0;
    /******************************/
	// transform algebra, author: Lars Widmer
	float sok = false;
	if(newData.valid) {
		sok = true;
		newData.valid=false;
		// cout << "calib in transform: " << calibrationData.accX << "; " << calibrationData.accY << "; " << calibrationData.accZ << endl;
		/*
		float compaccx	= currentNodeData.accX;
		float compaccy	= currentNodeData.accY;
		float compaccz	= currentNodeData.accZ;
		float voltage=currentNodeData.volts;
		*/
		/* // TODO: Why do we discard the comp-values?
		float uncompaccx	= currentNodeData.accX;
		float compaccx 		= uncompaccx + uncompaccx * (THREEVOLTS - voltage)/THREEVOLTS;
		float uncompaccy	= currentNodeData.accY;
		float compaccy		= uncompaccy + uncompaccy * (THREEVOLTS - voltage)/THREEVOLTS;
		float uncompaccz	= currentNodeData.accZ;
		float compaccz		= uncompaccz + uncompaccz * (THREEVOLTS - voltage)/THREEVOLTS;
		compaccx = uncompaccx;
		compaccy = uncompaccy;
		compaccz = uncompaccz;
		*/
		// float averagemagvectorsize=1.0f;
		// calculate correct measurements using calibration data
		// float accX = ( currentNodeData.accX - accXoffset[sensor]) * accXscale[sensor];
		// float accY = ( currentNodeData.accY - accYoffset[sensor]) * accYscale[sensor];
		// float accZ = ( currentNodeData.accZ - accZoffset[sensor]) * accZscale[sensor];
		/*
		float accX = (newData.accX - calibrationData.accX);
		float accY = (newData.accY - calibrationData.accY);
		float accZ = (newData.accZ - calibrationData.accZ);
		*/
		float accX = newData.accX;
		float accY = newData.accY;
		float accZ = newData.accZ;

		// float magX = ( currentNodeData.magX - magXoffset[sensor]) / averagemagvectorsize;
		// float magY = (-currentNodeData.magY - magYoffset[sensor]) / averagemagvectorsize;
		// float magY = ( currentNodeData.magY - magYoffset[sensor]) / averagemagvectorsize;
		// float magZ = ( currentNodeData.magZ - magZoffset[sensor]) / averagemagvectorsize;
		/*
		float magX = (newData.magX - calibrationData.magX);
		float magY = (newData.magY - calibrationData.magY);
		float magZ = (newData.magZ - calibrationData.magZ);
		*/
		float magX = newData.magX;
		float magY = newData.magY;
		float magZ = newData.magZ;

		// calculate absolute values
		float absaccZ;
		float absaccY;
		float absaccX;
		if(accZ < 0.0f)
			absaccZ = -accZ;
		else
			absaccZ =  accZ;
		if(accY < 0.0f)
			absaccY = -accY;
		else
			absaccY =  accY;
		if(accX < 0.0f)
			absaccX = -accX;
		else
			absaccX =  accX;

		// calculate the signs
		int signX = (int)(accX/absaccX);
		int signY = (int)(accY/absaccY);
		int signZ = (int)(accZ/absaccZ);

		// correct the orientation of the magnetometer data
		// Afterwards the coordinate system for the magn. and the acc. data is the same
		magX *= -1.0;
		magY *= -1.0;

		// calculate raw vector sizes
		// On a limited system (embedded device) this values can be replaced by constants
		// Nevertheless, performance is a piece of cake compared to semantic correctness
		float accVecSize = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));
		float magVecSize = sqrt(pow(magX,2)+pow(magY,2)+pow(magZ,2));

		// normalize vectors
		accX /= accVecSize;
		accY /= accVecSize;
		accZ /= accVecSize;
		magX /= magVecSize;
		magY /= magVecSize;
		magZ /= magVecSize;

		// compute helping values
		float sqrg = pow(accX,2)+pow(accY,2)+pow(accZ,2); // square of gravitation
		float grav = sqrt(sqrg); // gravitation
		float gravYZ = sqrt(pow(accY,2)+pow(accZ,2));

		// compute pitch and roll
		float pitch = asin(accX/grav);
		float roll = signY * acos(accZ/gravYZ);

		// For the computation of azimuth we introduce a plane G through (0,0,0) rectangular to the gravitation
		// This is exactly the place where we want to measure azimuth!
		// compute the height of the mag-vector above G
		float heightA = - (accX*magX + accY*magY + accZ*magZ) / sqrg;
		// compute the orthogonal projection vector x of the mag-vector in G
		float x1 = magX + accX*heightA;
		float x2 = magY + accY*heightA;
		float x3 = magZ + accZ*heightA;
		// length of x
		float lenX = sqrt(pow(x1,2)+pow(x2,2)+pow(x3,2));

		// reference vector
		float r1 = 1.0;
		float r2 = 0.0;
		float r3 = 0.0;
		float lenR = sqrt(pow(r1,2)+pow(r2,2)+pow(r3,2)); // 1

		// compute the height of the reference vector above G
		float heightB = - (accX*r1 + accY*r2 + accZ*r3) / sqrg;
		// compute the orthogonal projection vector y of the reference vector in G
		float y1 = r1 + accX*heightB;
		float y2 = r2 + accY*heightB;
		float y3 = r3 + accZ*heightB;
		// length of y
		float lenY = sqrt(pow(y1,2)+pow(y2,2)+pow(y3,2));

		// we now have x and y in the same plane
		// whereas the angle between them is our beloved azimuth
		float xy = x1*y1 + x2*y2 + x3*y3; // scalar product
		float azimuth = acos(xy/(lenX*lenY)); // azimuth angle

		// find vector product of acc & mag ==> vector t, orthogonal to acc & mag
		float t1 = accY*magZ - accZ*magY;
		float t2 = accZ*magX - accX*magZ;
		float t3 = accX*magY - accY*magX;

		// length of the projection of the reference vector on t
		float b = - (t1*r1+t2*r2+t3*r3) / (pow(t1,2)+pow(t2,2)+pow(t3,2));

		// reintroduce the sign of the azimuth
		if (b < 0) {
			azimuth *= -1.0;
		}

		// testing output
		// Defines::getInstance()->echo((int)(magX*100),(int)(magY*100),(int)(magZ*100));
		// Defines::getInstance()->echo((int)(x1*100),(int)(x2*100),(int)(x3*100));
		// cout << "; ";
		// Defines::getInstance()->echo((int)(y1*100),(int)(y2*100),(int)(y3*100));
		// Defines::getInstance()->echo((int)(xy*100),(int)(lenX*100),(int)(lenY*100));
		// cout << "; ";
		// Defines::getInstance()->echoBr((int)(gravYZ*100),(int)(azimuth*180.0/M_PI));
		// Defines::getInstance()->echo((int)(azimuth*180.0/M_PI),(int)(pitch*180.0/M_PI),(int)(roll*180.0/M_PI));
		// cout << "; " << endl;
		/*
		roll = 0.0;
		pitch = 0.0;
		*/

		// cout << "pitch: " << pitch << "; roll: " << roll << "; azimuth: " << azimuth << endl;

		result.xRotation = roll;
		result.yRotation = pitch;
		result.zRotation = azimuth;

	}
	/******************************/
	return result;
}

/**
 * The two given floats depict a rectangle.
 * This method returns the angle between x and the diagonal of the rectangle.
 */
float StreamTransformer::getPolarAngle(float x, float y)
{
      if( fabs(x) < 0.0000000001 && fabs(y) < 0.0000000001)
      {
          return 0.0f;
      }

      //normalize the vector
      float length;
      length = sqrt(x*x + y*y);
      x /= length;
      y /= length;

      //calculate the angle
      float angle;
      //since length (=>hypotenouse) = 1:
      angle = asin(y);

      //switch the qudrants
      if(x >= 0)
      {
         if(y >= 0)
         {
            //Quadrant I
            return angle;
         }
         //Quadrant IV
         return (6.283185307 - fabs(angle));
      }
      if(y >= 0)
      {
          //Quadrant II
          return (3.141592654 - angle);
      }
      //Quadrant III
      return (3.141592654 + fabs(angle));
}

/**
 * Empty class constructor.
 */
StreamTransformer::StreamTransformer()
{
	//Nothing to do here
}

/**
 * Empty class destructor.
 */
StreamTransformer::~StreamTransformer()
{
	//Nothing to do here
}
