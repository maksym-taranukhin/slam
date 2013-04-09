#include <coreslam.h>
#include <opencv2/highgui/highgui.hpp>

#include <Logger.hpp>
#include <Config.hpp>
#include <AlgoInterface.hpp>

using namespace LuxSlam;

class SlamInterface : public IAbstractAlgorithm
{
    int frames_counter;
    int frames_step;
public:
	SlamInterface (Config::ptr);
	bool create ();
	bool run (cv::Mat &, cv::Mat &);

	CoreSlam * slam;
};
