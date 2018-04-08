#include <vector>

using namespace std;

namespace math{

class WeightedAverageFilter{
public:
	WeightedAverageFilter():a1_(0.4), a2_(0.2), a3_(0.2), a4_(0.1), a5_(0.1){
		window_.clear();
		for(int i=0; i < 4; i++) window_.push_back(0);
	}
	WeightedAverageFilter(double a1, double a2, double a3, double a4, double a5):
	a1_(a1), a2_(a2), a3_(a3), a4_(a4), a5_(a5){
		window_.clear();
		for(int i=0; i < 4; i++) window_.push_back(0);
	}

	double apply(double sample){
		double result = a1_ * sample + a2_ * window_[3] + a3_ * window_[2] + a4_ * window_[1] + a5_ * window_[0];
		window_.erase(window_.begin());
		window_.push_back(result);
		return result;
	}

private:
	double a1_;
	double a2_;
	double a3_;
	double a4_;
	double a5_;
	vector<double> window_;
};
}