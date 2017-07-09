#ifndef PORTREND_DRAW_HDR
#define PORTREND_DRAW_HDR

#include <cassert>
#include <array>

#include "consts.hpp"
#include "math.hpp"

namespace draw {
	struct t_rsector {
		uint32_t sector_idx;

		int64_t view_xmin;
		int64_t view_xmax;
	};

	struct t_rstate {
	public:
		void reset() {
			column_maxs.fill(                     0);
			column_mins.fill(consts::WIN_SIZE_Y - 1);
		}

		void calc_column_bounds(const t_rsector& r_sector, const math::t_vec2f2& es_edge, const math::t_vec2f2& edge_sf) {
			// set eye-space distances (depths) for v and w
			es_depths.x() = es_edge.mins.y();
			es_depths.y() = es_edge.maxs.y();
			es_depths.z() = es_edge.maxs.y() - es_edge.mins.y();

			edge_xmin = -(es_edge.mins.x() * edge_sf.mins.x()) + consts::WIN_HSIZE_X;
			edge_xmax = -(es_edge.maxs.x() * edge_sf.maxs.x()) + consts::WIN_HSIZE_X;

			draw_xmin = std::max(edge_xmin, r_sector.view_xmin);
			draw_xmax = std::min(edge_xmax, r_sector.view_xmax);
		}

		bool sector_visible(const t_rsector& r_sector) const {
			return (edge_xmin < edge_xmax && edge_xmax >= r_sector.view_xmin && edge_xmin <= r_sector.view_xmax);
		}

		void proj_sector_heights(const math::t_vec2f2& sec_hgts, const math::t_vec2f2& edge_sf, float pitch) {
			const auto& project_y = [&pitch](float ybase, float zdist) { return (ybase + zdist * pitch); };

			// sector is considered visible; these conditions must be true
			assert(edge_xmax >           edge_xmin);
			assert(draw_xmin <=          draw_xmax);
			assert(draw_xmin >=                  0);
			assert(draw_xmax >=                  0);
			assert(draw_xmax <  consts::WIN_SIZE_X);

			// pseudo-project floor and ceil heights to screen-coors based on pitch-angle
			ceil_hgts[0] = -(project_y(sec_hgts.mins.y(), es_depths.x()) * edge_sf.mins.y()) + consts::WIN_HSIZE_Y;
			base_hgts[0] = -(project_y(sec_hgts.mins.x(), es_depths.x()) * edge_sf.mins.y()) + consts::WIN_HSIZE_Y;
			ceil_hgts[1] = -(project_y(sec_hgts.mins.y(), es_depths.y()) * edge_sf.maxs.y()) + consts::WIN_HSIZE_Y;
			base_hgts[1] = -(project_y(sec_hgts.mins.x(), es_depths.y()) * edge_sf.maxs.y()) + consts::WIN_HSIZE_Y;
			// ditto for the neighboring sector
			ceil_hgts[2] = -(project_y(sec_hgts.maxs.y(), es_depths.x()) * edge_sf.mins.y()) + consts::WIN_HSIZE_Y;
			base_hgts[2] = -(project_y(sec_hgts.maxs.x(), es_depths.x()) * edge_sf.mins.y()) + consts::WIN_HSIZE_Y;
			ceil_hgts[3] = -(project_y(sec_hgts.maxs.y(), es_depths.y()) * edge_sf.maxs.y()) + consts::WIN_HSIZE_Y;
			base_hgts[3] = -(project_y(sec_hgts.maxs.x(), es_depths.y()) * edge_sf.maxs.y()) + consts::WIN_HSIZE_Y;
		}

		void set_rgb_shift(bool sect_hit_pos, bool sect_hit_neg, bool edge_hit) { rgb_shift = ((sect_hit_pos || sect_hit_neg) && edge_hit); }


		void fill_column_pixels(uint32_t* p,  int64_t x, int64_t y1, int64_t y2,  uint32_t top, uint32_t mid, uint32_t bot) const {
			assert(x < consts::WIN_SIZE_X);

			constexpr int64_t ymin =                      0;
			constexpr int64_t ymax = consts::WIN_SIZE_Y - 1;

			constexpr int64_t ydif_min = -1;
			constexpr int64_t ydif_max = +1;

			y1 = math::clamp(y1, ymin, ymax);
			y2 = math::clamp(y2, ymin, ymax);

			switch (math::clamp(y1 - y2, ydif_min, ydif_max)) {
				case  0: { p[y1 * consts::WIN_SIZE_X + x] = mid; return; } break;
				case +1: {                                       return; } break;
				case -1: {
					// top pixel
					p[y1 * consts::WIN_SIZE_X + x] = top;

					// middle pixel(s)
					for (int64_t y = y1 + 1; y < y2; y++) {
						p[y * consts::WIN_SIZE_X + x] = mid;
					}

					// bottom pixel
					p[y2 * consts::WIN_SIZE_X + x] = bot;
				} break;
				default: {} break;
			}
		}


		void fill_column_span(uint32_t* pixels) {
			const int64_t edge_xdif = edge_xmax - edge_xmin;

			const int64_t dif_ceil_hgt = ceil_hgts[1] - ceil_hgts[0];
			const int64_t dif_base_hgt = base_hgts[1] - base_hgts[0];

			for (int64_t x = draw_xmin; x <= draw_xmax; ++x) {
				assert(column_mins[x] >= column_maxs[x]);
				assert(column_maxs[x] <= COLUMN_YMAX   );

				const int64_t  dx = x - edge_xmin;
				const int64_t cya = math::clamp(dx * dif_ceil_hgt / edge_xdif + ceil_hgts[0], column_maxs[x], column_mins[x]);
				const int64_t cyb = math::clamp(dx * dif_base_hgt / edge_xdif + base_hgts[0], column_maxs[x], column_mins[x]);

				const int64_t r_col_lerp_depth = (dx * es_depths.z() / edge_xdif + es_depths.x()) * 4;
				const uint8_t c_col_lerp_depth = math::clamp(r_col_lerp_depth, COLUMN_ZMIN, COLUMN_ZMAX);

				// does not make sense to shade horizontal surfaces this way
				// trickier within a column renderer, need to use raycasting
				const int64_t wall_shaded_rgb = (WALL_RGB * (255 - (c_col_lerp_depth / 1))) << rgb_shift;
				const int64_t ceil_shaded_rgb = (CEIL_RGB);
				const int64_t base_shaded_rgb = (BASE_RGB);

				fill_column_pixels(pixels,  x, column_maxs[x], cya - 1,  CEIL_RGB >>        1, ceil_shaded_rgb, CEIL_RGB >>        1); // ceil column
				fill_column_pixels(pixels,  x, cyb + 1, column_mins[x],  BASE_RGB  + 0x000055, base_shaded_rgb, BASE_RGB  + 0x000055); // floor colum

				// no neighbor, render wall from ceil (cya) to floor (cyb)
				fill_column_pixels(pixels,  x, cya, cyb,  0, wall_shaded_rgb * (1 - (x == edge_xmin || x == edge_xmax)), 0);
			}
		}

		void fill_column_span_ngb(uint32_t* pixels) {
			const int64_t edge_xdif = edge_xmax - edge_xmin;

			const int64_t     dif_ceil_hgt = ceil_hgts[1] - ceil_hgts[0];
			const int64_t     dif_base_hgt = base_hgts[1] - base_hgts[0];
			const int64_t ngb_dif_ceil_hgt = ceil_hgts[3] - ceil_hgts[2];
			const int64_t ngb_dif_base_hgt = base_hgts[3] - base_hgts[2];

			for (int64_t x = draw_xmin; x <= draw_xmax; ++x) {
				assert(column_mins[x] >= column_maxs[x]);
				assert(column_maxs[x] <= COLUMN_YMAX   );

				// clamped ceil (top) and floor (bottom) y-coordinates for current sector
				const int64_t  dx = x - edge_xmin;
				const int64_t cya = math::clamp(dx * dif_ceil_hgt / edge_xdif + ceil_hgts[0], column_maxs[x], column_mins[x]);
				const int64_t cyb = math::clamp(dx * dif_base_hgt / edge_xdif + base_hgts[0], column_maxs[x], column_mins[x]);

				// calculate z-coordinate for this column (only used for lighting)
				// strafing horizontally with respect to a wall does not change z
				const int64_t r_col_lerp_depth = (dx * es_depths.z() / edge_xdif + es_depths.x()) * 4;
				const uint8_t c_col_lerp_depth = math::clamp(r_col_lerp_depth, COLUMN_ZMIN, COLUMN_ZMAX);

				const int64_t ngb_uwall_shaded_rgb = ((WALL_RGB * 1        ) * (255 - (c_col_lerp_depth / 1))) << rgb_shift;
				const int64_t ngb_lwall_shaded_rgb = ((WALL_RGB * 4 - 0x1fe) * ( 31 - (c_col_lerp_depth / 8))) << rgb_shift;
				const int64_t      ceil_shaded_rgb = CEIL_RGB;
				const int64_t      base_shaded_rgb = BASE_RGB;

				// render column-pixels above this sector's ceil
				// render column-pixels below this sector's floor
				fill_column_pixels(pixels,  x, column_maxs[x], cya - 1,  CEIL_RGB >>        1, ceil_shaded_rgb, CEIL_RGB >>        1);
				fill_column_pixels(pixels,  x, cyb + 1, column_mins[x],  BASE_RGB  + 0x000055, base_shaded_rgb, BASE_RGB  + 0x000055);

				// clamped ceil (top) and floor (bottom) y-coordinates for neighbor sector
				const int64_t cnya = math::clamp(dx * ngb_dif_ceil_hgt / edge_xdif + ceil_hgts[2], column_maxs[x], column_mins[x]);
				const int64_t cnyb = math::clamp(dx * ngb_dif_base_hgt / edge_xdif + base_hgts[2], column_maxs[x], column_mins[x]);

				// render upper wall between ceils of current and neighbor sectors (if ceil is higher than ngb's)
				// render lower wall between floors of current and neighbor sectors (if floor is lower than ngb's)
				fill_column_pixels(pixels,  x, cya     , cnya - 1, 0,  ngb_uwall_shaded_rgb * (1 - (x == edge_xmin || x == edge_xmax)), 0);
				fill_column_pixels(pixels,  x, cnyb + 1, cyb     , 0,  ngb_lwall_shaded_rgb * (1 - (x == edge_xmin || x == edge_xmax)), 0);

				// restrict remaining columns to the space between ngb's ceil and floor
				column_maxs[x] = math::clamp(std::max(cya, cnya), column_maxs[x], COLUMN_YMAX);
				column_mins[x] = math::clamp(std::min(cyb, cnyb), COLUMN_YMIN, column_mins[x]);
			}
		}

		int64_t get_draw_xmin() const { return draw_xmin; }
		int64_t get_draw_xmax() const { return draw_xmax; }

	private:
		std::array<int64_t, consts::WIN_SIZE_X> column_maxs; // upper-most edge pixel per column
		std::array<int64_t, consts::WIN_SIZE_X> column_mins; // lower-most edge pixel per column

		// columns into which edge-vertices v and w are mapped
		int64_t edge_xmin;
		int64_t edge_xmax;
		// colums that are actually drawn as part of edge <v,w>
		int64_t draw_xmin;
		int64_t draw_xmax;

		int64_t base_hgts[4]; // (base(v),base(w)), (ngb_base(v),ngb_base(w))
		int64_t ceil_hgts[4]; // (ceil(v),ceil(w)), (ngb_ceil(v),ngb_ceil(w))

		uint8_t rgb_shift = 0;

		constexpr static int64_t COLUMN_YMIN =                      0;
		constexpr static int64_t COLUMN_YMAX = consts::WIN_SIZE_Y - 1;

		constexpr static int64_t COLUMN_ZMIN =   0;
		constexpr static int64_t COLUMN_ZMAX = 255;

		constexpr static int64_t WALL_RGB = 0x010101; // white
		constexpr static int64_t CEIL_RGB = 0x222222;
		constexpr static int64_t BASE_RGB = 0x0000DD; // blue

		// {min,max,dif} eye-space depths of edge-vertices
		math::t_vec3f es_depths;
	};
};

#endif

