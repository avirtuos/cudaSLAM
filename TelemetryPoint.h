#ifndef TELEMETRY_POINT_H
#define TELEMETRY_POINT_H

class TelemetryPoint
{
	public:
		TelemetryPoint();
		TelemetryPoint(int x_arg, int y_arg, int quality_arg, float distance_arg, float angle_arg);
		int x;
		int y;
		int quality;
		float distance;
		float angle;
		static void freeTelemetry(TelemetryPoint *point);
};

TelemetryPoint::TelemetryPoint(){
	x = 0;
	y = 0;
	distance = 0;
	angle = 0;
	quality = 0;
}

TelemetryPoint::TelemetryPoint(int x_arg, int y_arg, int quality_arg, float distance_arg, float angle_arg){
	x = x_arg;
	y = y_arg;
	quality = quality_arg;
	distance = distance_arg;
	angle = angle_arg;
}

#endif