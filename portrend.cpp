// bisqwit's homebrew software-rendering 3D engine (supporting
// non-Euclidean geometry editable at run-time like Duke Nukem
// 3D, but no sloped surfaces), rewritten in modern C++11
//
// does not perform texture-mapping, and also requires that
// each sector is a convex polygon for inside-test purposes


#include <SDL/SDL.h>

#include "base.hpp"
#include "consts.hpp"
#include "input.hpp"
#include "util.hpp"

static SDL_Surface* init_sdl() {
	SDL_Surface* s = SDL_SetVideoMode(consts::WIN_SIZE_X, consts::WIN_SIZE_Y, 32, 0);

	SDL_EnableKeyRepeat(150, 30);
	SDL_ShowCursor(SDL_DISABLE);
	// SDL_WM_GrabInput(SDL_GRAB_ON);
	SDL_WarpMouse(consts::WIN_SIZE_X >> 1, consts::WIN_SIZE_Y >> 1);
	return s;
}

static void kill_sdl() {
	SDL_WM_GrabInput(SDL_GRAB_OFF);
	SDL_Quit();
}


int main(int argc, char** argv) {
	SDL_Surface* draw_surface = init_sdl();

	if (draw_surface == nullptr)
		return EXIT_FAILURE;

	base::t_engine engine;
	util::t_system_timer timer;
	util::t_wall_clock clock;
	util::t_input_handler input_handler;

	engine.load((argc > 1)? argv[1]: "maps/main.txt");

	while (input_handler.poll_state()) {
		timer.tick_time();
		engine.queue_player_inputs(input_handler.get_state());

		while (clock.get_render_time_ns() >= consts::SIM_STEP_TIME_NS) {
			engine.update_player(consts::SIM_STEP_TIME_NS * 0.001f * 0.001f * 0.001f);
			clock.add_render_time_ns(-consts::SIM_STEP_TIME_NS);
		}

		engine.draw(draw_surface);
		clock.add_render_time_ns(timer.tock_time());
	}

	kill_sdl();
	return EXIT_SUCCESS;
}

