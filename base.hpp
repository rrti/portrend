#ifndef PORTRENDS_BASE_HDR
#define PORTRENDS_BASE_HDR

#include <cassert>
#include <cstdio>
#include <cstring>

#include <algorithm>
#include <vector>

#include <SDL/SDL.h>

#include "consts.hpp"
#include "draw.hpp"
#include "input.hpp"
#include "math.hpp"
#include "util.hpp"

namespace base {
	enum {
		PLAYER_STATE_DUCK = 0,
		PLAYER_STATE_JUMP = 1,
		PLAYER_STATE_CLIP = 2,
		PLAYER_STATE_LAST = 3,
	};
	enum {
		CURR_PLAYER_IDX = 0,
		PREV_PLAYER_IDX = 1,
	};

	struct t_wsector {
	public:
		void load(const std::vector<math::t_vec2f>& vertices, const std::vector<int32_t>& indices) {
			m_neighbors.resize(m_num_edges = (indices.size() >> 1));
			m_vertices.resize(m_num_edges + 1);

			for (uint32_t k = 0; k < m_num_edges; k++)
				m_neighbors[k] = indices[m_num_edges + k];

			for (uint32_t k = 0; k < m_num_edges; k++) {
				assert(size_t(indices[k]) < vertices.size());
				m_vertices[k + 1] = vertices[ indices[k] ];
			}

			// ensure the vertices form a loop
			m_vertices[0] = m_vertices[m_num_edges];
		}

		bool contains_point(const math::t_vec3f& p) const { return (contains_point(p, get_z_bounds())); }
		bool contains_point(const math::t_vec3f& p, const math::t_vec2f& zb) const {
			uint32_t outside_count = 0;

			// check the z-component against {base,ceil}height
			if (p.z() < zb.x()) return false;
			if (p.z() > zb.y()) return false;

			for (uint32_t s = 0; s < m_num_edges; ++s) {
				const math::t_vec2f& v = m_vertices[s + 0];
				const math::t_vec2f& w = m_vertices[s + 1];

				outside_count += (math::point_line_side_sign({p.x(), p.y()}, {v, w}) < 0);
			}

			return (outside_count == 0);
		}

		void calc_bounds(const math::t_vec2f& hgts) {
			const math::t_vec2f* verts = get_raw_verts();

			m_mins.x() =  9e9f;
			m_mins.y() =  9e9f;
			m_mins.z() = hgts.x();
			m_maxs.x() = -9e9f;
			m_maxs.y() = -9e9f;
			m_maxs.z() = hgts.y();

			for (uint32_t k = 0; k < m_num_edges; k++) {
				m_mins.x() = std::min(m_mins.x(), verts[k].x());
				m_mins.y() = std::min(m_mins.y(), verts[k].y());
				m_maxs.x() = std::max(m_maxs.x(), verts[k].x());
				m_maxs.y() = std::max(m_maxs.y(), verts[k].y());
			}
		}

		void inc_tmp_stamp() { m_tmp_stamp++; }

		const math::t_vec2f* get_raw_verts() const { return (m_vertices.data()); }
		const math::t_vec2f  get_rel_heights(float eyez) const { return {get_base_height() - eyez, get_ceil_height() - eyez}; }
		const math::t_vec2f  get_bb_mins() const { return {m_mins.x(), m_mins.y()}; }
		const math::t_vec2f  get_bb_maxs() const { return {m_maxs.x(), m_maxs.y()}; }
		const math::t_vec2f  get_z_bounds() const { return {m_mins.z(), m_maxs.z()}; }

		uint8_t get_ngb_idx(size_t i) const { assert(i < m_neighbors.size()); return m_neighbors[i]; }
		uint32_t get_num_edges() const { return m_num_edges; }
		uint32_t get_tmp_stamp() const { return m_tmp_stamp; }

		float get_base_height() const { return (m_mins.z()); }
		float get_ceil_height() const { return (m_maxs.z()); }

	private:
		math::t_vec3f m_mins;                    // bounding-rectangle mins; .z is floor-height
		math::t_vec3f m_maxs;                    // bounding-rectangle maxs; .z is ceiling-height

		std::vector<math::t_vec2f> m_vertices;   // {x,y}-coordinates per edge vertex
		std::vector<uint8_t> m_neighbors;        // index of neighboring sector per edge; 0xFF for walls

		uint32_t m_num_edges = 0;                // how many edges this sector contains
		uint32_t m_tmp_stamp = 0;                // already-processed-this-{frame,call} marker
	};


	typedef std::vector<t_wsector>& t_wsector_vector;
	typedef util::t_vector_queue<const base::t_wsector*> t_wsector_queue;



	struct t_player;
	struct t_world {
	public:
		t_player load(const char* map_name);
		void draw(const t_player& player, uint32_t* pixels);

		math::t_ray_hit trace_ray(const base::t_wsector& ray_sect, const math::t_ray_def& inf_ray) const;

		const std::vector<base::t_wsector>& get_sectors() const { return m_wsectors; }
		      std::vector<base::t_wsector>& get_sectors()       { return m_wsectors; }

		const base::t_wsector& get_sector(uint32_t i) const { return m_wsectors[i]; }
		      base::t_wsector& get_sector(uint32_t i)       { return m_wsectors[i]; }

		const util::t_vector_queue<const base::t_wsector*>& get_sector_queue() const { return m_wsector_queue; }
		      util::t_vector_queue<const base::t_wsector*>& get_sector_queue()       { return m_wsector_queue; }

	private:
		void load_vertices(char* ptr, int32_t* ncs, std::vector<math::t_vec2f>& vertices);
		void load_sector(char* ptr, char* buf, int32_t* ncs, std::vector<math::t_vec2f>& vertices, std::vector<int32_t>& indices);
		void load_player(char* ptr, int32_t* ncs, t_player& player);

	private:
		std::vector<base::t_wsector> m_wsectors;
		std::vector<uint32_t> m_rsectors;

		util::t_vector_queue<const base::t_wsector*> m_wsector_queue;
		util::t_vector_queue<      draw::t_rsector > m_rsector_queue;

		draw::t_rstate m_rstate;
	};



	struct t_player {
	public:
		void set_ws_mat(const math::t_mat33f& mat) { m_ws_mat = mat; }

		void set_lin_pos(const math::t_vec3f& pos) { m_lin_pos = pos; }
		void set_lin_vel(const math::t_vec3f& vel) { m_lin_vel = vel; }
		void set_ang_pos(const math::t_vec2f& pos) { m_ang_pos = {math::clamp(pos.x(), -consts::MAX_PITCH_ANGLE, consts::MAX_PITCH_ANGLE), pos.y()}; }
		void set_yaw_sca(const math::t_vec2f& ysc) { m_yaw_sca = ysc; }

		void set_lin_acc(const math::t_vec2f& acc) { m_lin_acc = acc; }
		void set_ang_vel(const math::t_vec2f& vel) { m_ang_vel = vel; }

		// NOTE:
		//   when yaw=0, sin(yaw) and cos(yaw) are 0 and 1 respectively
		//   i.e. compose_rot_xy(get_yaw_sin(), get_yaw_cos()) produces
		//   the identity rotation matrix in which the y-axis is offset
		//   by 90 degrees from the actual forward-vector (1, 0); could
		//   either call compose_rot_xy(yaw - M_PI * 0.5f) or construct
		//   the matrix directly from vectors
		//   world and player handedness mismatch, flip the right-vector
		math::t_mat33f calc_ws_mat() const {
			const math::t_vec3f& pos = m_lin_pos;
			const math::t_vec2f& fwd = calc_fwd_vec();
			const math::t_vec2f& rgt = calc_rgt_vec();
			#if 0
			return (math::t_mat33f{{rgt.x(), rgt.y(), 0.0f}, {fwd.x(), fwd.y(), 0.0f}, {pos.x(), pos.y(), 1.0f}});
			#else
			return (math::t_mat33f{{-rgt.x(), -rgt.y(), 0.0f}, {fwd.x(), fwd.y(), 0.0f}, {pos.x(), pos.y(), 1.0f}});
			#endif
		}

		math::t_vec2f calc_fwd_vec(const math::t_vec2f& mul = {1.0f, 1.0f}) const { return { get_yaw_cos() * mul.x(),  get_yaw_sin() * mul.y()}; } // <x,y>=<1,0>
		#if 0
		math::t_vec2f calc_rgt_vec(const math::t_vec2f& mul = {1.0f, 1.0f}) const { return { get_yaw_sin() * mul.y(), -get_yaw_cos() * mul.x()}; } // <y,-x>=<0,-1>
		#else
		math::t_vec2f calc_rgt_vec(const math::t_vec2f& mul = {1.0f, 1.0f}) const { return {-get_yaw_sin() * mul.y(),  get_yaw_cos() * mul.x()}; } // <-y,x>=<0,1>
		#endif

		math::t_vec2f calc_lin_acc(const uint8_t* ctrl_keys) const;
		math::t_vec3f calc_lin_vel() const;
		math::t_vec2f calc_ang_vel(const uint8_t* ctrl_keys, const int32_t* mouse_vec) const;


		math::t_vec3f calc_lin_pos() const { return (m_lin_pos + m_lin_vel); }
		math::t_vec2f calc_ang_pos() const { return (m_ang_pos + m_ang_vel); }


		void set_sector_idx(uint32_t s) { m_sector_idx = s; }
		void set_state_val(uint32_t s, uint32_t v) { m_state_val[s] = v; }

		void load(const math::t_vec3f& lin_pos, const math::t_vec2f& ang_pos);

		void apply_input(const util::t_input_state& state);
		void update(const t_wsector_vector& sectors, t_wsector_queue& queue);

		void jump(const t_wsector& sector);
		void fall(const t_wsector& sector);

		bool adjust_vel(const t_wsector_vector& sectors, t_wsector_queue& queue);
		bool adjust_pos(const t_wsector_vector& sectors, t_wsector_queue& queue);

		uint32_t get_sector_idx() const { return m_sector_idx; }
		uint32_t get_state_val(uint32_t s) const { return m_state_val[s]; }

		uint32_t calc_sector_idx(const t_wsector_vector& sectors, t_wsector_queue& queue) const;

		math::t_mat33f get_ws_mat() const { return m_ws_mat; }
		math::t_vec2f get_ang_pos() const { return m_ang_pos; }

		float get_eye_zhgt() const { return (math::lerp(consts::VIEW_HEIGHT, consts::DUCK_HEIGHT, get_state_val(PLAYER_STATE_DUCK) != 0)); }
		float get_max_zpos() const { return ((get_eye_zpos()                 ) + consts::HEAD_MARGIN); } // base+eye+head
		float get_min_zpos() const { return ((get_eye_zpos() - get_eye_zhgt()) + consts::KNEE_HEIGHT); } // base    +knee
		float get_eye_zpos() const { return (m_lin_pos.z()); } // base+eye

		float get_p_angle() const { return (m_ang_pos.x()); }
		float get_y_angle() const { return (m_ang_pos.y()); }

		float get_yaw_sin() const { return m_yaw_sca.x(); }
		float get_yaw_cos() const { return m_yaw_sca.y(); }

	private:
		math::t_mat33f m_ws_mat;

		math::t_vec3f m_lin_pos;   // current position
		math::t_vec3f m_lin_vel;   // current velocity (motion vector)
		math::t_vec2f m_lin_acc;
		math::t_vec2f m_ang_pos;   // view-direction (.x=pitch, .y=yaw) angles
		math::t_vec2f m_ang_vel;   // radians per frame
		math::t_vec2f m_yaw_sca;   // sin(yaw) and cos(yaw)

		uint32_t m_sector_idx = 0; // index of current sector we are in
		uint32_t m_temp_stamp = 1;

		uint32_t m_state_val[PLAYER_STATE_LAST];
	};


	struct t_engine {
	public:
		void load(const char* map_name);
		void draw(SDL_Surface* s);

		void queue_player_inputs(util::t_input_state& state);
		bool apply_player_inputs();

		void update_player(float);

	public:
		base::t_world m_world;
		base::t_player m_players[2];
		util::t_input_buffer m_inputs;

		bool m_in_replay = false;
		bool m_is_paused = false;
	};
};

#endif

