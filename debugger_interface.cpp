#include <debugger_interface.hpp>

SlamInterface::SlamInterface (Config::ptr c) : IAbstractAlgorithm (c)
{
}

bool SlamInterface::create()
{
	slam = new CoreSlam ();
	return true;
}

bool SlamInterface::run(cv::Mat &image, cv::Mat &dmap)
{
	LuxFrame * f = new LuxFrame;
	f->image = image;
	f->depth_map = dmap;

	slam->run(f);

	return true;
}
