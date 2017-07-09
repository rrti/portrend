#ifndef PORTREND_CONSTS_HDR
#define PORTREND_CONSTS_HDR

#include <cmath>

namespace consts {
	static constexpr int64_t WIN_SIZE_X = 640 + 320;
	static constexpr int64_t WIN_SIZE_Y = 480;

	static constexpr int64_t WIN_HSIZE_X = WIN_SIZE_X >> 1;
	static constexpr int64_t WIN_HSIZE_Y = WIN_SIZE_Y >> 1;


	static constexpr uint32_t SIM_STEP_RATE    = 60; // Hz
	static constexpr    float SIM_STEP_SIZE    = 1.0f / SIM_STEP_RATE; // dt (ms)
	static constexpr    float SIM_STEP_SIZE_SQ = SIM_STEP_SIZE * SIM_STEP_SIZE;
	static constexpr uint64_t SIM_STEP_TIME_NS = (1000.0f / SIM_STEP_RATE) * 1000 * 1000;
	static constexpr uint64_t WALL_SEC_TIME_NS = 1000 * 1000 * 1000;


	static constexpr float VIEW_HEIGHT = 6.00f;   // height above sector-floor when standing
	static constexpr float DUCK_HEIGHT = 1.50f;   // height above sector-floor when crouching
	static constexpr float HEAD_MARGIN = 1.00f;   // distance above camera before head hits sector-ceiling
	static constexpr float KNEE_HEIGHT = 2.00f;   // maximum non-jumping obstacle height
	static constexpr float GRAVITY_ACC = 0.02f;

	static constexpr float MAX_PITCH_ANGLE = M_PI * 0.4995f * 2.0f;
	static constexpr float MAX_ADJUST_DIST = 5.0f;


	static constexpr const float INF_DIST = 1e+3f;
	static constexpr const float EPS_DIST = 1e-3f;


	static constexpr float ZN_DIST =    0.00125f;
	static constexpr float ZF_DIST = 1000.00000f;

	static constexpr float HOR_ASP = WIN_SIZE_X / WIN_SIZE_Y;
	static constexpr float HOR_FOV = 45.0f * (M_PI / 180.0f); // 0.7854[rad]

	// tan(hfov) = xdim / zf -> xdim = zf * tan(hfov) -> xdim = zf (if hfov=45)
	// tan(vfov) = ydim / zf -> ydim = zf * tan(vfov) -> vfov = atan(ydim / zf)
	static constexpr float ZN_XDIM = ZN_DIST * std::tan(HOR_FOV);
	static constexpr float ZF_XDIM = ZF_DIST * std::tan(HOR_FOV);

	static constexpr float VER_FOV = std::atan((ZF_XDIM / HOR_ASP) / ZF_DIST);

	static constexpr float FOV_WIN_SIZE_X = WIN_SIZE_X * HOR_FOV;
	static constexpr float FOV_WIN_SIZE_Y = WIN_SIZE_Y * VER_FOV;
};

#endif

