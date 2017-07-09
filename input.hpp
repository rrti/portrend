#ifndef PORTREND_INPUT_HDR
#define PORTREND_INPUT_HDR

#include <vector>
#include <SDL/SDL.h>

namespace util {
	enum {
		CTRL_MOVE_FRWD_KEY = 0,
		CTRL_MOVE_BACK_KEY = 1,
		CTRL_MOVE_LEFT_KEY = 2,
		CTRL_MOVE_RGHT_KEY = 3,

		CTRL_TURN_P_UP_KEY = 4,
		CTRL_TURN_P_DN_KEY = 5,
		CTRL_TURN_Y_LT_KEY = 6,
		CTRL_TURN_Y_RT_KEY = 7,

		CTRL_STATE_JUMP_KEY =  8,
		CTRL_STATE_DUCK_KEY =  9,
		CTRL_STATE_CLIP_KEY = 10,

		MISC_SET_PAUSED_KEY = 0,
		MISC_SET_REPLAY_KEY = 1,
	};

	struct t_input_state {
		uint8_t ctrl_keys[11]; // wsad, udlr, j,d,c
		uint8_t misc_keys[ 2];
		int32_t mouse_vec[ 2]; // dx,dy
	};


	struct t_input_buffer {
	public:
		t_input_buffer() { clear(true); }

		void push(const t_input_state& s) { m_buffer.emplace_back(m_frame, s); }
		void tick() { m_frame += 1; }

		void clear(bool reset) {
			m_frame = 0;
			m_index = 0;

			if (!reset)
				return;

			// erase all previous inputs
			m_buffer.clear();
			m_buffer.reserve(1000);
		}

		bool peek() const { return (m_index < m_buffer.size() && m_frame == m_buffer[m_index].first); }
		bool empty() const { return (m_index >= m_buffer.size()); }

		const t_input_state& get() { return (m_buffer[m_index  ].second); }
		const t_input_state& pop() { return (m_buffer[m_index++].second); }

	private:
		std::vector< std::pair<size_t, t_input_state> > m_buffer;

		size_t m_frame;
		size_t m_index;
	};


	struct t_input_handler {
	public:
		bool poll_state() {
			SDL_Event ev;

			// note:
			//   SDL's polling-rate is smaller than our rendering frame-rate
			//   this means its inputs will be reused across multiple frames
			while (SDL_PollEvent(&ev)) {
				switch (ev.type) {
					case SDL_KEYDOWN:
					case SDL_KEYUP: {
						switch (ev.key.keysym.sym) {
							case 'w': { m_state.ctrl_keys[CTRL_MOVE_FRWD_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case 's': { m_state.ctrl_keys[CTRL_MOVE_BACK_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case 'a': { m_state.ctrl_keys[CTRL_MOVE_LEFT_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case 'd': { m_state.ctrl_keys[CTRL_MOVE_RGHT_KEY] = (ev.type == SDL_KEYDOWN); } break;

							#if 0
							case SDLK_UP   : { m_state.ctrl_keys[CTRL_TURN_P_UP_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case SDLK_DOWN : { m_state.ctrl_keys[CTRL_TURN_P_DN_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case SDLK_LEFT : { m_state.ctrl_keys[CTRL_TURN_Y_LT_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case SDLK_RIGHT: { m_state.ctrl_keys[CTRL_TURN_Y_RT_KEY] = (ev.type == SDL_KEYDOWN); } break;
							#else
							case SDLK_KP8  : { m_state.ctrl_keys[CTRL_TURN_P_UP_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case SDLK_KP5  : { m_state.ctrl_keys[CTRL_TURN_P_DN_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case SDLK_KP4  : { m_state.ctrl_keys[CTRL_TURN_Y_LT_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case SDLK_KP6  : { m_state.ctrl_keys[CTRL_TURN_Y_RT_KEY] = (ev.type == SDL_KEYDOWN); } break;
							#endif

							case SDLK_LSHIFT: { m_state.ctrl_keys[CTRL_STATE_DUCK_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case SDLK_LALT  : { m_state.ctrl_keys[CTRL_STATE_JUMP_KEY] = (ev.type == SDL_KEYDOWN); } break;
							case SDLK_SPACE : { m_state.ctrl_keys[CTRL_STATE_CLIP_KEY] = (ev.type == SDL_KEYDOWN); } break;

							case 'p': { m_state.misc_keys[MISC_SET_PAUSED_KEY] |= (ev.type == SDL_KEYDOWN); } break;
							case 'r': { m_state.misc_keys[MISC_SET_REPLAY_KEY] |= (ev.type == SDL_KEYDOWN); } break;
							case 'q': { return false; } break;
							default: {} break;
						}
					} break;

					case SDL_QUIT: {
						return false;
					} break;
				}
			}

			SDL_GetRelativeMouseState(&m_state.mouse_vec[0], &m_state.mouse_vec[1]);
			return true;
		}

		const t_input_state& get_state() const { return m_state; }
		      t_input_state& get_state()       { return m_state; }

	private:
		t_input_state m_state = {{0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, {0, 0}, {0, 0}};
	};
};

#endif

