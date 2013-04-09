#include <debugger_interface.hpp>

SlamInterface::SlamInterface (Config::ptr c) : IAbstractAlgorithm (c)
{
    frames_counter = 0;
    frames_step = atoi(config->get<std::string> ("coreslam.frames_step").c_str());
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

    if (frames_counter%frames_step == 0)
        slam->run(f);

    frames_counter++;

	return true;
}
