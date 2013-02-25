#include <coreslam.h>
#include <opencv2/highgui/highgui.hpp>

#include <logger.hpp>
#include <config.hpp>
#include <algo_interface.hpp>

using namespace LuxSlam;

class SlamInterface : public IAbstractAlgorithm
{
public:
	SlamInterface (Config::ptr);
	bool create ();
	bool run (cv::Mat &, cv::Mat &);

	CoreSlam * slam;
};
