#include "base.hpp"

void base::t_world::load_vertices(char* ptr, int32_t* ncs, std::vector<math::t_vec2f>& vertices) {
	math::t_vec2f vert;

	// vertex; scan y-coordinate (shared by all vertices on this line)
	std::sscanf(ptr, "%f%n", &vert[1], ncs);

	// scan x-coordinates
	for (ptr += (*ncs); std::sscanf(ptr, "%f%n", &vert[0], ncs) == 1; ptr += (*ncs)) {
		vertices.push_back(vert);
	}
}

void base::t_world::load_sector(char* ptr, char* buf, int32_t* ncs, std::vector<math::t_vec2f>& vertices, std::vector<int32_t>& indices) {
	m_wsectors.emplace_back();

	base::t_wsector& sect = m_wsectors.back();
	math::t_vec2f hgts = {0.0f, 0.0f};

	// floor and ceiling heights
	std::sscanf(ptr, "%f%f%n", &hgts[0], &hgts[1], ncs);

	// sector; scan vertex-indices (N+1) and neighbor edge-indices (N)
	for (ptr += (*ncs); std::sscanf(ptr, "%32s%n", buf, ncs) == 1 && buf[0] != '#'; ptr += (*ncs)) {
		indices.push_back(std::atoi(buf));
	}

	sect.load(vertices, indices);
	sect.calc_bounds(hgts);

	// reset for next sector (if any)
	indices.clear();
}

void base::t_world::load_player(char* ptr, int32_t* ncs, t_player& player) {
	math::t_vec3f pos;

	// player; scan position and yaw-angle
	std::sscanf(ptr, "%f %f %f %d", &pos[0], &pos[1], &pos[2], ncs);

	player.set_sector_idx(*ncs);
	player.load({pos.x(), pos.y(), m_wsectors[*ncs].get_base_height() + consts::VIEW_HEIGHT}, {0.0f, pos.z()});
	player.update(m_wsectors, m_wsector_queue);
}

base::t_player base::t_world::load(const char* map_name) {
	FILE* fp = std::fopen(map_name, "rt");

	if (fp == nullptr) {
		std::printf("[world::%s] could not open file \"%s\"\n", __func__, map_name);
		std::exit(1);
	}

	char line_buf[256] = {0};
	char word_buf[256] = {0};

	int32_t num_chars = 0;
	int32_t num_words = 0;

	t_player player;

	std::vector<math::t_vec2f> vertices;
	std::vector<int32_t> indices;

	vertices.reserve(400);
	indices.reserve(200);

	m_wsectors.clear();
	m_wsectors.reserve(100);

	while (fgets(line_buf, sizeof(line_buf), fp) != nullptr) {
		if (line_buf[0] == '\0' || line_buf[0] == '#')
			continue;

		// parse line for "vertex"/"sector"/"player" keyword
		if ((num_words = std::sscanf(line_buf, "%32s%n", word_buf, &num_chars)) != 1)
			continue;

		switch (word_buf[0]) {
			case 'v': { load_vertices(line_buf + num_chars, &num_chars, vertices); } break;
			case 's': { load_sector(line_buf + num_chars, word_buf, &num_chars, vertices, indices); } break;
			case 'p': { load_player(line_buf + num_chars, &num_chars, player); } break;
			default: {} break;
		}
	}

	std::fclose(fp);
	std::printf("[world::%s] #sectors=%lu\n", __func__, m_wsectors.size());

	m_rsectors.clear();
	m_rsectors.resize(m_wsectors.size(), 0);

	return player;
}



void base::t_world::draw(const t_player& player, uint32_t* pixels) {
	// clear color-buffer
	std::memset(&pixels[0], 0, consts::WIN_SIZE_X * consts::WIN_SIZE_Y * 4);
	std::fill(m_rsectors.begin(), m_rsectors.end(), 0);

	// start rendering from player's sector; queue
	// sorts sectors in (roughly) near to far order
	m_rsector_queue.clear();
	m_rsector_queue.reserve(32);
	m_rsector_queue.push_back(draw::t_rsector{player.get_sector_idx(), 0, consts::WIN_SIZE_X - 1});
	m_rstate.reset();

	const math::t_mat33f& ws_mat = player.get_ws_mat();
	const math::t_mat33f  es_mat = ws_mat.invert();
	const math::t_vec2f   ws_pos = {ws_mat.m[6], ws_mat.m[7]};
	const math::t_vec2f    r_pov = {player.get_eye_zpos(), player.get_p_angle()};

	// note: this ray is always emitted horizontally (i.e. in the xy-plane)
	const math::t_ray_def pov_ray = {{ws_pos.x(), ws_pos.y(), r_pov.x()}, ws_mat.get_y()};
	const math::t_ray_hit ray_hit = trace_ray(get_sector(player.get_sector_idx()), pov_ray);


	// eye-space frustum lines; left is right since matrix inverts handedness
	// the frustum has to start at the player's position rather than at z-near
	// or clipping can fail, e.g. if player is exactly on top of a portal-edge
	constexpr math::t_vec2f2 es_fline_l = {{-consts::ZN_XDIM * 0.0f, consts::ZN_DIST * 0.0f}, {-consts::ZF_XDIM, consts::ZF_DIST}};
	constexpr math::t_vec2f2 es_fline_r = {{ consts::ZN_XDIM * 0.0f, consts::ZN_DIST * 0.0f}, { consts::ZF_XDIM, consts::ZF_DIST}};

	#if 0
	const auto point_in_frustum = [&](const math::t_vec3f p_es) {
		const float a = math::point_line_side({p_es.x(), p_es.y()}, es_fline_l);
		const float b = math::point_line_side({p_es.x(), p_es.y()}, es_fline_r);
		return ((-a) >= 0.0f && b >= 0.0f && p_es.y() >= consts::ZN_DIST && p_es.y() <= consts::ZF_DIST);
	};
	#endif
	const auto sector_in_frustum = [&](const base::t_wsector* s) {
		const math::t_vec3f es_mins = es_mat * s->get_bb_mins();
		const math::t_vec3f es_maxs = es_mat * s->get_bb_maxs();

		int8_t fl_sum = 0;
		int8_t fr_sum = 0;

		// left is right, invert signs
		fl_sum -= math::point_line_side_sign({es_mins.x(), es_mins.y()}, es_fline_l);
		fl_sum -= math::point_line_side_sign({es_maxs.x(), es_mins.y()}, es_fline_l);
		fl_sum -= math::point_line_side_sign({es_maxs.x(), es_maxs.y()}, es_fline_l);
		fl_sum -= math::point_line_side_sign({es_mins.x(), es_maxs.y()}, es_fline_l);

		fr_sum += math::point_line_side_sign({es_mins.x(), es_mins.y()}, es_fline_r);
		fr_sum += math::point_line_side_sign({es_maxs.x(), es_mins.y()}, es_fline_r);
		fr_sum += math::point_line_side_sign({es_maxs.x(), es_maxs.y()}, es_fline_r);
		fr_sum += math::point_line_side_sign({es_mins.x(), es_maxs.y()}, es_fline_r);

		// false iff all corners are behind *one* frustum-edge
		return (fl_sum != -4 && fr_sum != -4);
	};


	while (!m_rsector_queue.empty()) {
		// pick a sector from the queue to draw; must be a
		// copy since the queue might resize itself on any
		// push_back
		const draw::t_rsector rsector = m_rsector_queue.pop_front();

		// prevent infinite loops from cyclic portal-chains
		// (e.g. if A links to B and B links back to A, the
		// traversal might never terminate)
		assert(rsector.sector_idx < m_rsectors.size());
		assert(rsector.sector_idx < m_wsectors.size());

		if ((m_rsectors[rsector.sector_idx] += 1) >= m_wsectors.size())
			continue;

		const base::t_wsector* const sect = &m_wsectors[rsector.sector_idx];
		const math::t_vec2f* const verts = sect->get_raw_verts();

		// draw floor and ceiling relative to player's view-height
		const math::t_vec2f cur_sec_hgts = sect->get_rel_heights(r_pov.x());

		// test sector bounding-box against frustum edges; glitchy
		if (false && !sector_in_frustum(sect))
			continue;

		// render walls of this sector that are facing towards the player
		// this uses each portal-wall as though it were a viewing frustum
		// (i.e. clips neighboring sectors against the portal boundaries)
		for (uint32_t ei = 0, nsi = 0xFF; ei < sect->get_num_edges(); ei++) {
			const math::t_vec2f2 ws_e = {verts[ei + 0], verts[ei + 1]};
			const math::t_vec2f ws_o = {-(ws_e.maxs.y() - ws_e.mins.y()), (ws_e.maxs.x() - ws_e.mins.x())}; // ortho=<-y,x>

			const float ws_e_dist_raw = math::point_line_side(ws_pos, ws_e);
			const float ws_e_dist_abs = std::fabs(ws_e_dist_raw);

			// inverse-transform world-space points into eye-space
			//
			// prior to transforming, nudge both vertices away along the wall's
			// normal-vector if player is right up against it; otherwise es_v.x
			// and es_w.x can both become 0 (when player's viewing-direction is
			// parallel to wall) and the edge-span will have zero width
			const math::t_vec3f es_v = es_mat * (ws_e.mins - ws_o * consts::EPS_DIST * (ws_e_dist_abs < 0.0001f));
			const math::t_vec3f es_w = es_mat * (ws_e.maxs - ws_o * consts::EPS_DIST * (ws_e_dist_abs < 0.0001f));

			// eye-space edge from v to w
			math::t_vec2f2 es_edge = {{es_v.x(), es_v.y()}, {es_w.x(), es_w.y()}};

			// skip if wall is entirely behind the player
			if (es_edge.mins.y() < 0.0f && es_edge.maxs.y() < 0.0f)
				continue;

			// clip wall against view-frustrum if partially behind the player
			// (find intersection-points between wall and view-frustum edges)
			if (es_edge.mins.y() < 0.0f && math::line_intersect(es_edge, es_fline_r))
				es_edge.mins = math::line_intersect_point(es_edge, es_fline_r);
			if (es_edge.maxs.y() < 0.0f && math::line_intersect(es_edge, es_fline_l))
				es_edge.maxs = math::line_intersect_point(es_edge, es_fline_l);

			// clamp distances s.t. project_y() * sf fits within an int64
			es_edge.mins.y() = std::max(es_edge.mins.y(), consts::ZN_DIST * 0.1f);
			es_edge.maxs.y() = std::max(es_edge.maxs.y(), consts::ZN_DIST * 0.1f);

			// perspective transformation scale-factors
			// these can become excessively large close to the near-plane and
			// prevent entire colums from being rendered; player must be kept
			// away from walls
			const math::t_vec2f v_sf = {consts::FOV_WIN_SIZE_X / es_edge.mins.y(), consts::FOV_WIN_SIZE_Y / es_edge.mins.y()};
			const math::t_vec2f w_sf = {consts::FOV_WIN_SIZE_X / es_edge.maxs.y(), consts::FOV_WIN_SIZE_Y / es_edge.maxs.y()};
			const math::t_vec2f2 edge_sf = {v_sf, w_sf};

			// only render if edge is visible, highlight intersected walls
			m_rstate.calc_column_bounds(rsector, es_edge, edge_sf);
			m_rstate.set_rgb_shift((rsector.sector_idx == ray_hit.ids.x()), ((-rsector.sector_idx - 1) == ray_hit.ids.x()), (ei == ray_hit.ids.y()));

			if (!m_rstate.sector_visible(rsector))
				continue;

			if ((nsi = sect->get_ngb_idx(ei)) != 0xFF) {
				// schedule neighbor sector for rendering within the span formed by wall <s>
				m_rsector_queue.push_back(draw::t_rsector{nsi, m_rstate.get_draw_xmin(), m_rstate.get_draw_xmax()});

				// another sector is visible through this edge, adjust heights
				m_rstate.proj_sector_heights({cur_sec_hgts, m_wsectors[nsi].get_rel_heights(r_pov.x())}, edge_sf, r_pov.y());
				m_rstate.fill_column_span_ngb(pixels);
			} else {
				// render the pixel-columns spanned by this edge
				m_rstate.proj_sector_heights({cur_sec_hgts, {0.0f, 0.0f}}, edge_sf, r_pov.y());
				m_rstate.fill_column_span(pixels);
			}
		}
	}
}



math::t_ray_hit base::t_world::trace_ray(const base::t_wsector& ray_sect, const math::t_ray_def& inf_ray) const {
	math::t_vec3f hit_pos{0.0f, 0.0f, 0.0f};

	math::t_vec3f pos = inf_ray.pos;
	math::t_vec3f dir = inf_ray.dir;
	math::t_vec2f hzi = {0.0f, 0.0f};

	// xy-plane projected (flat) segment
	const math::t_vec2f2 ray_seg{{pos.x(), pos.y()}, {pos.x() + dir.x() * consts::INF_DIST, pos.y() + dir.y() * consts::INF_DIST}};

	for (uint32_t cur_sector_idx = &ray_sect - &m_wsectors[0], prv_sector_idx = 0xFF; cur_sector_idx < m_wsectors.size(); /*no-op*/) {
		const base::t_wsector* const sect = &m_wsectors[cur_sector_idx];
		const math::t_vec2f* const verts = sect->get_raw_verts();

		// note:
		//   these can fail after an epsilon-nudge if a sector references a
		//   neighbor that is not (topologically) connected to it along the
		//   shared edge
		assert(pos.z() >= sect->get_base_height());
		assert(pos.z() <= sect->get_ceil_height());
		assert(sect->contains_point(pos));

		// store horizontal plane-height in .x, intersection-time in .y
		// the negative epsilon-offset is not required since "infinite"
		// z-bounds are passed to contains_point
		switch (math::sign(dir.z())) {
			case -1: { hzi.x() = sect->get_base_height(); hzi.y() = ((pos.z() - hzi.x()) / -dir.z()) - consts::EPS_DIST; } break;
			case +1: { hzi.x() = sect->get_ceil_height(); hzi.y() = ((hzi.x() - pos.z()) /  dir.z()) - consts::EPS_DIST; } break;
			default: {                                    hzi.y() =                                    consts::INF_DIST; } break;
		}

		// early-out if ray intersects this sector's floor- or ceiling-plane
		if (sect->contains_point(hit_pos = pos + dir * hzi.y(), {-consts::INF_DIST, consts::INF_DIST}))
			return {hit_pos, {0.0f, 0.0f, 1.0f * -math::sign(dir.z())}, {cur_sector_idx, 0xFFFFFFFF}};

		for (uint32_t ei = 0, nsi = 0xFF; ei < sect->get_num_edges(); ei++) {
			const math::t_vec3f  vert = {verts[ei + 0].x(), verts[ei + 0].y(), sect->get_base_height()};
			const math::t_vec2f2 edge = {verts[ei + 0], verts[ei + 1]};
			const math::t_vec3f  nrml = {-(edge.maxs.y() - edge.mins.y()), (edge.maxs.x() - edge.mins.x()), 0.0f}; // ortho=<-y,x>

			// never need to care about being behind a wall here
			if (nrml.dot(dir) >= 0.0f)
				continue;
			if (!math::line_intersect(edge, ray_seg))
				continue;

			const math::t_plane wall(nrml / nrml.len(), vert);

			// if the ray crosses this sector-edge, then it must also intersect
			// the wall quadrilateral (which may be a portal) and can move into
			// the neighboring sector
			hit_pos = pos + dir * wall.ray_dist({pos, dir});

			// test against neighbor's floor and ceiling in case edge is portal
			// if ray hits the vertical surface between non-equal sector floors
			// (or ceilings), signal this by negating the sector-index
			if ((nsi = sect->get_ngb_idx(ei)) != 0xFF && nsi != prv_sector_idx) {
				if (hit_pos.z() < m_wsectors[nsi].get_base_height() || hit_pos.z() > m_wsectors[nsi].get_ceil_height())
					return {hit_pos, nrml, {-cur_sector_idx - 1, ei}};

				// epsilon-nudge into neighbor
				pos = hit_pos + dir * consts::EPS_DIST;

				prv_sector_idx = cur_sector_idx;
				cur_sector_idx = nsi;
				break;
			}

			// not a portal, terminate ray
			return {hit_pos, nrml, {cur_sector_idx, ei}};
		}
	}

	return {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0xFFFFFFFF, 0xFFFFFFFF}};
}






math::t_vec2f base::t_player::calc_lin_acc(const uint8_t* ctrl_keys) const {
	const math::t_vec2f  mul = {0.2f, 0.2f};
	const math::t_vec2f& fwd = calc_fwd_vec(mul);
	const math::t_vec2f& rgt = calc_rgt_vec(mul);

	math::t_vec2f acc = {0.0f, 0.0f};
	acc += (fwd * ctrl_keys[util::CTRL_MOVE_FRWD_KEY]);
	acc -= (fwd * ctrl_keys[util::CTRL_MOVE_BACK_KEY]);
	acc += (rgt * ctrl_keys[util::CTRL_MOVE_RGHT_KEY]);
	acc -= (rgt * ctrl_keys[util::CTRL_MOVE_LEFT_KEY]);
	return acc;
}

math::t_vec3f base::t_player::calc_lin_vel() const {
	// apply linear acceleration
	math::t_vec2f vel = {m_lin_vel.x() + m_lin_acc.x(), m_lin_vel.y() + m_lin_acc.y()};

	// clamp and decay linear velocity
	const float max_spd = math::lerp(0.25f, 0.05f, get_state_val(PLAYER_STATE_DUCK));
	const float cur_spd = vel.len();
	const float inv_spd = 1.0f / std::max(cur_spd, consts::EPS_DIST);
	const float nxt_spd = math::clamp(cur_spd * 0.95f, -max_spd, max_spd);

	// normalize and rescale
	return {(vel.x() * inv_spd) * nxt_spd, (vel.y() * inv_spd) * nxt_spd, m_lin_vel.z()};
}

math::t_vec2f base::t_player::calc_ang_vel(const uint8_t* ctrl_keys, const int32_t* mouse_vec) const {
	math::t_vec2f vel = m_ang_vel;

	const math::t_vec2f mul = {1.500f * 0.01667f, 0.667f * 0.01667f};
	const math::t_vec2f spd = {math::lerp(0.2f, 0.05f, get_state_val(PLAYER_STATE_DUCK)), math::lerp(0.075f, 0.01875f, get_state_val(PLAYER_STATE_DUCK))};

	#ifdef USE_MOUSE_LOOK
	(void) ctrl_keys;

	vel.x() += (mouse_vec[1] * 0.025f); // pitch (vertical-delta)
	vel.y() += (mouse_vec[0] * 0.010f); // yaw (horizontal-delta)
	vel.x()  = math::clamp(vel.x() * 0.95f, -0.25f, 0.25f);
	vel.y()  = math::clamp(vel.y() * 0.95f, -0.25f, 0.25f);
	#else
	(void) mouse_vec;

	// apply angular acceleration
	vel.x() += (ctrl_keys[util::CTRL_TURN_P_UP_KEY] * mul.x());
	vel.x() -= (ctrl_keys[util::CTRL_TURN_P_DN_KEY] * mul.x());
	vel.y() -= (ctrl_keys[util::CTRL_TURN_Y_LT_KEY] * mul.y());
	vel.y() += (ctrl_keys[util::CTRL_TURN_Y_RT_KEY] * mul.y());

	// clamp and decay angular velocity
	vel.x() = math::clamp(vel.x() * 0.95f, -spd.x(), spd.x());
	vel.y() = math::clamp(vel.y() * 0.95f, -spd.y(), spd.y());
	#endif
	return vel;
}



void base::t_player::load(const math::t_vec3f& lin_pos, const math::t_vec2f& ang_pos) {
	set_state_val(PLAYER_STATE_JUMP, 0);
	set_state_val(PLAYER_STATE_DUCK, 0);

	set_lin_pos(lin_pos);
	set_ang_pos({ang_pos.x(), ang_pos.y()});
	set_lin_vel({0.0f, 0.0f, 0.0f});
	set_ang_vel({0.0f, 0.0f      });
}

void base::t_player::apply_input(const util::t_input_state& state) {
	// set unadjusted velocities; used next update-step
	set_lin_acc(calc_lin_acc(&state.ctrl_keys[0]));
	set_lin_vel(calc_lin_vel());
	set_ang_vel(calc_ang_vel(&state.ctrl_keys[0], &state.mouse_vec[0]));

	// note: ducking and clipping are stateful, cancelled only by releasing the key
	set_state_val(PLAYER_STATE_JUMP, state.ctrl_keys[util::CTRL_STATE_JUMP_KEY]);
	set_state_val(PLAYER_STATE_DUCK, state.ctrl_keys[util::CTRL_STATE_DUCK_KEY]);
	set_state_val(PLAYER_STATE_CLIP, state.ctrl_keys[util::CTRL_STATE_CLIP_KEY]);
}

void base::t_player::update(const t_wsector_vector& sectors, t_wsector_queue& queue) {
	jump(sectors[get_sector_idx()]);
	fall(sectors[get_sector_idx()]);

	adjust_vel(sectors, queue); m_temp_stamp += 1;
	set_sector_idx(calc_sector_idx(sectors, queue)); m_temp_stamp += 1;

	// set position based on adjusted velocities
	set_lin_pos(calc_lin_pos());
	set_ang_pos(calc_ang_pos());
	set_yaw_sca({std::sin(m_ang_pos.y()), std::cos(m_ang_pos.y())});

	#if 0
	// avoids minor glitches close to walls
	adjust_pos(sectors, queue); m_temp_stamp += 1;
	set_sector_idx(calc_sector_idx(sectors, queue)); m_temp_stamp += 1;
	#endif

	set_ws_mat(calc_ws_mat());
}



void base::t_player::jump(const t_wsector& sector) {
	if (get_state_val(PLAYER_STATE_JUMP) == 0)
		return;
	// no mid-air jumping
	if (get_eye_zpos() > (sector.get_base_height() + get_eye_zhgt()))
		return;

	set_lin_vel({m_lin_vel.x(), m_lin_vel.y(), m_lin_vel.z() + consts::GRAVITY_ACC * 25.0f});
	set_state_val(PLAYER_STATE_JUMP, 0);
}

void base::t_player::fall(const t_wsector& sector) {
	// add gravitational acceleration
	const float z_pos = get_eye_zpos() + (m_lin_vel.z() -= consts::GRAVITY_ACC);
	const float z_min = sector.get_base_height() + get_eye_zhgt();
	const float z_max = sector.get_ceil_height() - consts::HEAD_MARGIN;

	if (m_lin_vel.z() < 0.0f && z_pos <= z_min) {
		// prevent sinking below ground
		set_lin_pos({m_lin_pos.x(), m_lin_pos.y(), z_min});
		set_lin_vel({m_lin_vel.x(), m_lin_vel.y(), 0.0f});
		return;
	}
	if (m_lin_vel.z() > 0.0f && z_pos >= z_max) {
		// prevent jumping above ceiling
		set_lin_vel({m_lin_vel.x(), m_lin_vel.y(), 0.0f});
		return;
	}
}



bool base::t_player::adjust_vel(const t_wsector_vector& sectors, t_wsector_queue& queue) {
	const math::t_vec3f pos = m_lin_pos;
	      math::t_vec3f vel = m_lin_vel;

	if (vel.x() == 0.0f && vel.y() == 0.0f)
		return false;

	if (get_state_val(PLAYER_STATE_CLIP) != 0)
		return false;

	// if we are not leaving our own sector, just move normally
	if (sectors[m_sector_idx].contains_point(pos + vel))
		return false;

	queue.clear();
	queue.push_back(&sectors[m_sector_idx]);

	while (!queue.empty()) {
		const base::t_wsector* sect = queue.pop_front();
		const math::t_vec2f* verts = sect->get_raw_verts();

		#if 0
		// player is about to cross one of the sector's edges
		// note that it is not sufficient to just check *this* sector's edges
		// (e.g. in corners where two or more incident sectors share a vertex)
		// instead we maintain a queue of active sectors, initially containing
		// only the player-sector and expand outward through portals
		//
		// after collision adjustments <vel> can cross a portal-edge which it
		// did not previously intersect, so whenever velocity is changed each
		// edge must be retested
		if (sect->get_tmp_stamp() == m_temp_stamp)
			continue;

		const_cast<base::t_wsector*>(sect)->inc_tmp_stamp();
		#endif

		for (uint32_t iter = 0; ((iter < 100) && (vel.x() != 0.0f || vel.y() != 0.0f)); iter++) {
			uint32_t num_collisions = 0;

			for (uint32_t ei = 0; ei < sect->get_num_edges(); ++ei) {
				const uint8_t ngb_sector_idx = sect->get_ngb_idx(ei);

				const math::t_vec2f2 e = {verts[ei + 0], verts[ei + 1]};
				const math::t_vec2f np = {pos.x() + vel.x(), pos.y() + vel.y()};

				// {max_base,min_ceil}_height
				math::t_vec2f hgts = {+9e9f, -9e9f};


				if (!math::rect_intersect({{pos.x(), pos.y()}, np},  e))
					continue;

				// skip edge if we remain on inside of sector with respect to it
				if (math::point_line_side_sign(np, e) >= 0)
					continue;

				if (ngb_sector_idx != 0xFF) {
					hgts.x() = std::max(sect->get_base_height(), sectors[ngb_sector_idx].get_base_height());
					hgts.y() = std::min(sect->get_ceil_height(), sectors[ngb_sector_idx].get_ceil_height());
				}

				// determine if we can enter the neighbor-sector (if any)
				if ((hgts.y() < get_max_zpos()) || (hgts.x() > get_min_zpos())) {
					const math::t_vec2f edge_xy  = e.maxs - e.mins;
					const math::t_vec3f edge_xyz = {edge_xy.x(), edge_xy.y(), 0.0f};
					const math::t_vec3f rvel     = vel;

					// remove velocity along direction of wall-normal
					vel.x() = edge_xy.x() * (rvel.dot(edge_xyz)) / edge_xy.dot(edge_xy);
					vel.y() = edge_xy.y() * (rvel.dot(edge_xyz)) / edge_xy.dot(edge_xy);

					num_collisions += 1;
					continue;
				}

				if (ngb_sector_idx != 0xFF) {
					queue.push_back(&sectors[ngb_sector_idx]);
				}
			}

			// round very small velocities down to 0
			vel.x() *= (std::abs(vel.x()) > 0.0001f);
			vel.y() *= (std::abs(vel.y()) > 0.0001f);

			if (num_collisions == 0)
				break;

		}
	}

	// set the final velocity adjusted for collisions
	set_lin_vel(vel);
	return true;
}

#if 0
bool base::t_player::adjust_pos(t_wsector_vector& sectors, t_wsector_queue& queue) {
	queue.clear();
	queue.push_back(&sectors[m_sector_idx]);

	math::t_vec2f vec = {0.0f, 0.0f};

	const math::t_vec2f p = {m_lin_pos.x(), m_lin_pos.y()}; // unadjusted position
	const math::t_vec2f v = {m_lin_vel.x(), m_lin_vel.y()}; // xy-velocity

	while (!queue.empty()) {
		const base::t_wsector* sect = queue.pop_front();
		const math::t_vec2f* verts = sect->get_raw_verts();

		// consider each sector only once
		if (sect->get_tmp_stamp() == m_temp_stamp)
			continue;

		const_cast<base::t_wsector*>(sect)->inc_tmp_stamp();

		for (uint32_t ei = 0; ei < sect->get_num_edges(); ++ei) {
			const uint8_t ngb_sector_idx = sect->get_ngb_idx(ei);

			const math::t_vec2f2 ws_e = {verts[ei + 0], verts[ei + 1]};
			const math::t_vec2f ws_o = {-(ws_e.maxs.y() - ws_e.mins.y()), (ws_e.maxs.x() - ws_e.mins.x())}; // ortho=<-y,x>

			const float ws_e_dist = math::point_line_side(p, ws_e);

			if (ws_e_dist < 0.0f)
				continue;
			if (ws_e_dist > consts::MAX_ADJUST_DIST)
				continue;

			if (ngb_sector_idx != 0xFF) {
				queue.push_back(&sectors[ngb_sector_idx]);
				continue;
			}

			// keep away from walls
			vec += ((ws_o / ws_o.len()) * (0.1f - ws_e_dist) * (ws_e_dist <= 0.1f));
		}
	}

	set_lin_vel({vec.x(), vec.y(), m_lin_vel.z()});
	set_sector_idx(calc_sector_idx(sectors, queue));
	set_lin_vel({v.x(), v.y(), m_lin_vel.z()});
	set_lin_pos({p.x() + vec.x(), p.y() + vec.y(), m_lin_pos.z()});

	return (vec.x() != 0.0f || vec.y() != 0.0f);
}
#endif

uint32_t base::t_player::calc_sector_idx(const t_wsector_vector& sectors, t_wsector_queue& queue) const {
	// start searching from the player's last known sector
	// move() collision-detection has already ensured the
	// velocity-vector will not lead to a position outside
	// any sector so this typically is fast
	queue.clear();
	queue.push_back(&sectors[m_sector_idx]);

	while (!queue.empty()) {
		const t_wsector* sect = queue.pop_front();
		const math::t_vec2f* verts = sect->get_raw_verts();

		if (sect->get_tmp_stamp() == m_temp_stamp)
			continue;

		const_cast<base::t_wsector*>(sect)->inc_tmp_stamp();

		for (uint32_t ei = 0; ei < sect->get_num_edges(); ++ei) {
			const uint8_t ngb_sector_idx = sect->get_ngb_idx(ei);

			if (ngb_sector_idx == 0xFF)
				continue;

			const math::t_vec2f2 edge = {verts[ei + 0], verts[ei + 1]};
			const math::t_vec2f npos = {m_lin_pos.x() + m_lin_vel.x(), m_lin_pos.y() + m_lin_vel.y()};

			// NOTE:
			//   <px,py> to <px+dx, py+dy> defines a rectangle as well as a
			//   movement-vector, so it can be intersected with edge <v, w>
			//   actual sectors can be any convex planar polygon
			if (!math::rect_intersect({{m_lin_pos.x(), m_lin_pos.y()}, npos}, edge))
				continue;
			// next position must lie on the opposite side of current edge
			// edge vertices of each sector are defined in clockwise order;
			// point_line_side will always return -1 for a point outside of
			// the sector and 0 or 1 for a point inside
			if (math::point_line_side_sign(npos, edge) >= 0)
				continue;

			// found next sector player will end up in
			if (sectors[ngb_sector_idx].contains_point(m_lin_pos + m_lin_vel))
				return ngb_sector_idx;

			// continue with neighbor
			queue.push_back(&sectors[ngb_sector_idx]);
		}
	}

	return m_sector_idx;
}



void base::t_engine::load(const char* map_name) {
	m_players[CURR_PLAYER_IDX] = m_world.load(map_name);
	m_players[PREV_PLAYER_IDX] = m_players[CURR_PLAYER_IDX];
}

void base::t_engine::draw(SDL_Surface* s) {
	SDL_LockSurface(s);

	// render map from player's POV
	m_world.draw(m_players[CURR_PLAYER_IDX], static_cast<uint32_t*>(s->pixels));

	SDL_UnlockSurface(s);
	SDL_Flip(s);
}



void base::t_engine::queue_player_inputs(util::t_input_state& state) {
	if (state.misc_keys[util::MISC_SET_PAUSED_KEY] != 0)
		m_is_paused = !m_is_paused;

	if (state.misc_keys[util::MISC_SET_REPLAY_KEY] != 0) {
		m_in_replay = !m_in_replay;

		if (m_in_replay) {
			// entered replay mode; consume stored inputs
			m_players[CURR_PLAYER_IDX] = m_players[PREV_PLAYER_IDX];
			m_inputs.clear(false);
		} else {
			// exited replay-mode; erase leftover stored inputs
			m_inputs.clear(true);
			return;
		}
	}

	state.misc_keys[util::MISC_SET_PAUSED_KEY] = 0;
	state.misc_keys[util::MISC_SET_REPLAY_KEY] = 0;

	if (m_is_paused)
		return;
	if (m_in_replay)
		return;

	// capture raw inputs (minus replay and pause), including nulls
	m_inputs.push(state);
}

bool base::t_engine::apply_player_inputs() {
	// apply all inputs that were given (queued) on the same frame
	if (!m_inputs.empty()) {
		while (m_inputs.peek()) {
			m_players[CURR_PLAYER_IDX].apply_input(m_inputs.pop());
		}

		m_inputs.tick();
		return true;
	}

	// exited replay-mode normally
	m_players[PREV_PLAYER_IDX] = m_players[CURR_PLAYER_IDX];
	m_inputs.clear(true);
	return (m_in_replay = false);
}



void base::t_engine::update_player(float) {
	if (m_is_paused)
		return;

	if (!apply_player_inputs())
		return;

	m_players[CURR_PLAYER_IDX].update(m_world.get_sectors(), m_world.get_sector_queue());
}

