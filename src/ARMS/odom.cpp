#include "ARMS/lib.h"
#include "api.h"

using namespace pros;

namespace arms::odom {

bool debug;
double left_right_distance;
double middle_distance;
double left_right_tpi;
double middle_tpi;

// odom tracking values
double global_x;
double global_y;
double heading;
double heading_degrees;
double prev_heading = 0;
double prev_left_pos = 0;
double prev_right_pos = 0;
double prev_middle_pos = 0;

int odomTask() {

	global_x = 0;
	global_y = 0;
	heading = 0;

	while (true) {
		double left_pos;
		double right_pos;
		double middle_pos;

		// get positions of each encoder
		left_pos = chassis::leftEncoder->get();
		right_pos = chassis::rightEncoder->get();
		if (chassis::middleEncoder)
			middle_pos = chassis::middleEncoder->get();

		// calculate change in each encoder
		double delta_left = (left_pos - prev_left_pos) / left_right_tpi;
		double delta_right = (right_pos - prev_right_pos) / left_right_tpi;
		double delta_middle = chassis::middleEncoder
		                          ? (middle_pos - prev_middle_pos) / middle_tpi
		                          : 0;

		// calculate new heading
		double delta_angle;
		if (chassis::imu) {
			heading_degrees = chassis::angle();
			heading = heading_degrees * M_PI / 180.0;
			delta_angle = heading - prev_heading;
		} else {
			delta_angle = (delta_right - delta_left) / (left_right_distance * 2);

			heading += delta_angle;
			heading_degrees = heading * 180.0 / M_PI;
		}

		// store previous positions
		prev_left_pos = left_pos;
		prev_right_pos = right_pos;
		prev_middle_pos = middle_pos;
		prev_heading = heading;

		// calculate local displacement
		double local_x;
		double local_y;

		if (delta_angle) {
			double i = sin(delta_angle / 2.0) * 2.0;
			local_x = (delta_right / delta_angle + left_right_distance) * i;
			local_y = (delta_middle / delta_angle + middle_distance) * i;
		} else {
			local_x = delta_right;
			local_y = delta_middle;
		}

		double p = heading - delta_angle / 2.0; // global angle

		// convert to absolute displacement
		global_x += cos(p) * local_x + sin(p) * local_y;
		global_y += cos(p) * local_y + sin(p) * local_x;

		if (debug)
			printf("%.2f, %.2f, %.2f \n", global_x, global_y, heading_degrees);

		delay(10);
	}
}

void reset(Point point) {
	global_x = point.x;
	global_y = point.y;
}

void reset(Point point, double angle) {
	reset(point);
	heading = angle * M_PI / 180.0;
	prev_heading = heading;
	chassis::resetAngle(angle);
}

double getAngleError(Point point) {
	double x = point.x;
	double y = point.y;

	x -= global_x;
	y -= global_y;

	double delta_theta = atan2(y, x) - heading;

	while (fabs(delta_theta) > M_PI) {
		delta_theta -= 2 * M_PI * delta_theta / fabs(delta_theta);
	}

	return delta_theta;
}

double getDistanceError(Point point) {
	double x = point.x;
	double y = point.y;

	y -= global_y;
	x -= global_x;
	return sqrt(x * x + y * y);
}

void init(bool debug, double left_right_distance, double middle_distance,
          double left_right_tpi, double middle_tpi) {
	odom::debug = debug;
	odom::left_right_distance = left_right_distance;
	odom::middle_distance = middle_distance;
	odom::left_right_tpi = left_right_tpi;
	odom::middle_tpi = middle_tpi;
	Task odom_task(odomTask);
}

} // namespace arms::odom
