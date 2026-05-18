#ifndef  FILTER_EXPANSION
#define FILTER_EXPANSION
#include <ros/ros.h>
#include <Eigen/Eigen>
double det(std::vector<std::vector<double>> mat, int excludeRow, int excludeCol);
std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, std::vector<std::vector<double>> B);
std::vector<double> operator * (std::vector<double> a, std::vector<std::vector<double>> B);
std::vector<std::vector<double>> operator * (double a, std::vector<std::vector<double>> B);
std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, double b);
std::vector<double> operator * (std::vector<std::vector<double>> A, std::vector<double> b);
std::vector<std::vector<double>> transpose(std::vector<std::vector<double>> mat);
std::vector<std::vector<double>> inv(std::vector<std::vector<double>> mat);


double det(std::vector<std::vector<double>> mat, int excludeRow, int excludeCol)
{
	std::vector<std::vector<double>> excludedMat;
	int exMatDim = 0;
	for (int i = 0; i < mat.size(); i++) {
		if (i == excludeRow) continue;
		excludedMat.push_back({});
		for (int j = 0; j < mat.size(); j++) {
			if (j == excludeCol) continue;
			excludedMat[exMatDim].push_back(mat[i][j]);
		}
		exMatDim++;
	}
	if (exMatDim == 1) return excludedMat[0][0];
	double result = 0;
	for (int j = 0; j < exMatDim; j++) {
		if (j % 2 == 0)
			result += excludedMat[0][j] * det(excludedMat, 0, j);
		else
			result -= excludedMat[0][j] * det(excludedMat, 0, j);
	}
	return result;
}

std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, std::vector<std::vector<double>> B)
{
	if (A[0].size() != B.size()) throw - 1;
	vector<vector<double>> C;
	C.resize(A.size());
	for (int i = 0; i < C.size(); i++) {
		C[i].resize(B[0].size());
	}
	for (int i = 0; i < A.size(); i++) {
		for (int j = 0; j < B[0].size(); j++) {
			for (int k = 0; k < A[0].size(); k++) {
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}
	return C;
}

std::vector<std::vector<double>> operator * (double a, std::vector<std::vector<double>> B)
{
	for (int i = 0; i < B.size(); i++)
		for (int j = 0; j < B[0].size(); j++)
			B[i][j] *= a;
	return B;
}

std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, double b)
{
	return b * A;
}

std::vector<double> operator * (std::vector<std::vector<double>> A, std::vector<double> b)
{
	if (A[0].size() != b.size()) throw - 1;
	//vector<vector<double>> B = {};
	//for (int i = 0; i < b.size(); i++) {
	//	B.push_back({ b[i] });
	//}
	vector<vector<double>> B = transpose({ b });
	vector<vector<double>> C = A * B;
	vector<double> c = {};
	for (int i = 0; i < C.size(); i++) {
		c.push_back(C[i][0]);
	}
	return c;
}

std::vector<double> operator * (std::vector<double> a, std::vector<std::vector<double>> B)
{
	if (a.size() != B.size()) throw - 1;
	vector<vector<double>> A = {};
	A.push_back(a);
	vector<vector<double>> C = A * B;
	vector<double> c = C[0];
	return c;
}

std::vector<std::vector<double>> transpose(std::vector<std::vector<double>> mat)
{
	vector<vector<double>> matT;
	matT.resize(mat[0].size());
	for (int i = 0; i < matT.size(); i++) {
		matT[i].resize(mat.size());
	}
	for (int i = 0; i < mat.size(); i++) {
		for (int j = 0; j < mat[0].size(); j++) {
			matT[j][i] = mat[i][j];
		}
	}
	return matT;
}

std::vector<std::vector<double>> inv(std::vector<std::vector<double>> mat)
{
	if (mat.size() != mat[0].size()) throw -1;
	double matDet = det(mat,-1,-1);
	if (matDet == 0) throw -1;
	vector<vector<double>> invMat(mat);
	for (int i = 0; i < mat.size(); i++) {
		for (int j = 0; j < mat.size(); j++) {
			if ((i + j) % 2 == 0)
				invMat[j][i] = (det(mat, i, j) / matDet);
			else
				invMat[j][i] = (-det(mat, i, j) / matDet);
		}
	}
	return invMat;
}



double getTimeSec(){
    ros::Time t1 = ros::Time::now();
    double t_cur = t1.toSec();//获取的是自1970年一月一日到现在时刻的秒数
    return t_cur;
}

namespace filter_expansion::planner
{
	class exponential_mean_filter
	{
	private:
		double ultimate_output = -1.0;
		double last_output = -1.0;
		double last_output_time = -1.0;
		bool has_init = false;
		double nominal_delay_sec;
	public:
		exponential_mean_filter(double nominal_delay_sec = 10.0) :nominal_delay_sec(nominal_delay_sec) {};
		void update(double new_input)
		{
			ultimate_output = new_input;
			if (!has_init) {
				last_output = new_input;
				has_init = true;
			}
		}
		double output()
		{
			double now_time = getTimeSec();
			if (has_init) {
				double coef = (now_time - last_output_time) / nominal_delay_sec;
				last_output = (1.0 - coef) * last_output + coef * ultimate_output;
			}
			last_output_time = now_time;
			return last_output;
		}
	};
	typedef exponential_mean_filter order_1_filter;

/////
	class polynom_interpolator
	{
	private:
		int polynom_order;
		std::deque<double> control_points = {};
		std::deque<double> sample_times = {};
		double ultimate_output = -1.0;
		bool has_init = false;
		double sample_sec;
		std::vector<std::vector<double>> coef_mat = {};
		//Eigen::MatrixXd coef_mat={};
	public:
		const double nominal_delay_sec;
		polynom_interpolator(double sample_sec = 0.04, int polynom_order = 3) :polynom_order(polynom_order), sample_sec(sample_sec), nominal_delay_sec((polynom_order + 1)* sample_sec)
		{
			for (int row = 0; row < polynom_order + 1; row++) {
				coef_mat.push_back({});
				for (int col = 0; col < polynom_order + 1; col++) {
					coef_mat[row].push_back(pow((row * sample_sec), col));
				}
			}
			coef_mat = inv(coef_mat);
		};
		void update(double new_input)
		{
			double now_time = getTimeSec();
			if (!has_init) {
				if (control_points.empty()) {
					for (int isample = 0; isample < polynom_order + 1; isample++) {
						control_points.push_front(new_input);
						sample_times.push_front(now_time - isample * sample_sec);
					}
				}
				has_init = true;
			}
			ultimate_output = new_input;
		}
		double output()
		{
			double now_time = getTimeSec();
			//���¿��Ƶ�
			while (sample_times.back() + sample_sec < now_time) {
				control_points.push_back(ultimate_output);
				sample_times.push_back(sample_times.back() + sample_sec);
			}
			while (control_points.size() > polynom_order + 1) {
				control_points.pop_front();
				sample_times.pop_front();
			}
			//�������
			double output;
			if (!has_init)
				output = -1.0;
			else {
				double dt = now_time - sample_times.back();
				std::vector<double> control_points_vec(control_points.begin(), control_points.end());
				std::vector<double> polynom_coefs = coef_mat * control_points_vec;
				output = 0.0;
				for (int iorder = 0; iorder < polynom_order + 1; iorder++) {
					output += polynom_coefs[iorder] * pow(dt, iorder);
				}
			}
			return output;
		}
	};
////


	struct SplineControlPoint {
		double x;   // ���Ƶ�����
		double w;   // ���Ƶ�Ȩ��
	};

	class Bspline_interpolator
	{
	private:
		int spline_order;
		std::deque<SplineControlPoint> control_points = {};
		std::deque<double> sample_times = {};
		double ultimate_output = -1.0;
		bool has_init = false;
		double sample_sec;
		std::vector<std::vector<double>> coef_mat = {};
	public:
		
		// �������ĵݹ鶨��
		double N(int i, int k, double t, const std::vector<double>& knots) {
			if (k == 1) {
				if (knots[i] <= t && t < knots[i + 1]) {
					return 1.0;
				}
				else {
					return 0.0;
				}
			}

			double denom1 = knots[i + k - 1] - knots[i];
			double denom2 = knots[i + k] - knots[i + 1];
			double term1 = 0.0;
			double term2 = 0.0;

			if (denom1 != 0.0) {
				term1 = (t - knots[i]) / denom1 * N(i, k - 1, t, knots);
			}
			if (denom2 != 0.0) {
				term2 = (knots[i + k] - t) / denom2 * N(i + 1, k - 1, t, knots);
			}

			return term1 + term2;
		}

		// ���ɽڵ�����
		std::vector<double> generateKnotVector(int numControlPoints, int splineOrder) {
			int n = numControlPoints + splineOrder - 1;
			std::vector<double> knots(n + 1);
			for (int i = 0; i <= n; ++i) {
				knots[i] = static_cast<double>(i);
			}
			return knots;
		}

		// ��������B����
		std::vector<double> rationalBSpline(int numControlPoints, int splineOrder, const std::deque<SplineControlPoint>& controlPoints, const std::vector<double>& knots, const std::vector<double>& params) {
			std::vector<double> result(params.size());

			for (size_t j = 0; j < params.size(); ++j) {
				double t = params[j];
				double numerator = 0.0;
				double denominator = 0.0;

				for (int i = 0; i < numControlPoints; ++i) {
					double B = N(i, splineOrder, t, knots);
					numerator += B * controlPoints[i].x * controlPoints[i].w;
					denominator += B * controlPoints[i].w;
				}

				if (denominator != 0.0) {
					result[j] = numerator / denominator;
				}
				else {
					result[j] = 0.0;  // ���������
				}
			}

			return result;
		}
		
		const double nominal_delay_sec;
		Bspline_interpolator(double sample_sec = 0.04, int spline_order = 3) :spline_order(spline_order), sample_sec(sample_sec), nominal_delay_sec((spline_order + 1)* sample_sec)
		{};
		void update(double new_input)
		{
			double now_time = getTimeSec();
			if (!has_init) {
				if (control_points.empty()) {
					for (int isample = 0; isample < spline_order + 1; isample++) {
						control_points.push_front({ new_input, 1.0 });
						sample_times.push_front(now_time - isample * sample_sec);
					}
					has_init = true;
				}
			}
			ultimate_output = new_input;
		}
		double output()
		{
			double now_time = getTimeSec();
			//���¿��Ƶ�
			while (sample_times.back() + sample_sec < now_time) {
				control_points.push_back({ ultimate_output, 1.0 });
				sample_times.push_back(sample_times.back() + sample_sec);
			}
			while (control_points.size() > spline_order + 1) {
				control_points.pop_front();
				sample_times.pop_front();
			}
			//�������
			double output;
			if (!has_init)
				output = -1.0;
			else {
				double dt = now_time - sample_times.back();
				double u = 3 + dt / sample_sec;

				std::vector<double> knots = generateKnotVector(control_points.size(), spline_order);

				double result = rationalBSpline(control_points.size(), spline_order, control_points, knots, {u})[0];

				output = result;
			}
			return output;
		}

	};

/*
	class plannertest1 : public filter
	{
	private:
		double last_update_time;
	public:
		virtual void init() final
		{
		}
		virtual void main_loop() final
		{
			double t = getTimeSec();
			t = ceil(t/4 / 0.08) * 0.08;
			double raw_data = sin(t) / ((cos(t * 3.1416) * cos(t * 3.1416)) + 1);//ģ��һ��0.03sΪ���ڵ�ԭʼ�ź�
			//double raw_data = t;
			static planner::order_1_filter exp_planner(0.30);//������ʱ
			static planner::polynom_interpolator poly_planner(0.10, 3);//�������ڡ�����
			static planner::Bspline_interpolator Bspline_planner(0.10, 3);//�������ڡ�����
			exp_planner.update(raw_data);
			poly_planner.update(raw_data);
			Bspline_planner.update(raw_data);
			double res_data1 = exp_planner.output();
			double res_data2 = poly_planner.output();
			double res_data3 = Bspline_planner.output();
			write("Before Plan", raw_data);
			write("After Exp Plan", res_data1);
			write("After Poly Plan", res_data2);
			write("After Spline Plan", res_data3);
			Sleep(1);
		}
	};
    */
}


#endif