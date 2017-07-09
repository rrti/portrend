#ifndef PORTREND_MATH_HDR
#define PORTREND_MATH_HDR

#include <algorithm>
#include <cmath>

namespace math {
	// note: +z is up
	template<typename type, size_t size> struct t_vec {
	typedef t_vec<type, size> vec_type;
	public:
		#if 0
		// neater but breaks constexpr init
		t_vec<type, size>(const std::initializer_list<type>& l = {}) {
			std::memset(&a[0], 0, size * sizeof(type));
			std::memcpy(&a[0], &l[0], std::min(l.size(), size) * sizeof(type));
		}
		#endif

		vec_type operator * (const vec_type& v) const { vec_type r; for (size_t n = 0; n < size; n++) { r[n] = a[n] * v[n]; } return r; }
		vec_type operator + (const vec_type& v) const { vec_type r; for (size_t n = 0; n < size; n++) { r[n] = a[n] + v[n]; } return r; }
		vec_type operator - (const vec_type& v) const { vec_type r; for (size_t n = 0; n < size; n++) { r[n] = a[n] - v[n]; } return r; }

		vec_type& operator += (const vec_type& v) { for (size_t n = 0; n < size; n++) { a[n] += v[n]; } return *this; }
		vec_type& operator -= (const vec_type& v) { for (size_t n = 0; n < size; n++) { a[n] -= v[n]; } return *this; }

		vec_type operator / (type s) const { return ((*this) * (type(1) / s)); }
		vec_type operator * (type s) const {
			vec_type r;
			for (size_t n = 0; n < size; n++) {
				r[n] = a[n] * s;
			}
			return r;
		}

		type  operator [] (size_t i) const { assert(i < size); return a[i]; }
		type& operator [] (size_t i)       { assert(i < size); return a[i]; }

		type len() const { return (std::sqrt(dot(*this))); }
		type dot(const vec_type& v) const {
			type r = type(0);
			for (size_t n = 0; n < size; n++) {
				r += (a[n] * v[n]);
			}
			return r;
		}

		type x() const { return ((*this)[0]); } type& x() { return ((*this)[0]); }
		type y() const { return ((*this)[1]); } type& y() { return ((*this)[1]); }
		type z() const { return ((*this)[2]); } type& z() { return ((*this)[2]); }
		type w() const { return ((*this)[3]); } type& w() { return ((*this)[3]); }

	public:
		type a[size];
	};

	typedef t_vec<float, 2> t_vec2f;
	typedef t_vec<float, 3> t_vec3f;
	typedef t_vec<float, 4> t_vec4f;

	typedef t_vec< int32_t, 2> t_vec2i;
	typedef t_vec<uint32_t, 2> t_vec2u;

	// semi-infinite ray definition
	struct t_ray_def {
		t_vec3f pos;
		t_vec3f dir;
	};

	struct t_ray_hit {
		t_vec3f pos; // intersection-coors
		t_vec3f nrm; // surface-normal vector
		t_vec2u ids; // sector and edge indices
	};

	struct t_plane {
	public:
		t_plane(const t_vec3f& nrml = {0.0f, 0.0f, 0.0f}, const t_vec3f& vert = {0.0f, 0.0f, 0.0f}) {
			m_nrml = nrml;
			m_vert = vert;
			m_dist = m_nrml.dot(m_vert);
		}

		float pos_dist(const t_vec3f& pos) const { return (m_dist - m_nrml.dot(pos)); }
		float ray_dist(const t_ray_def& ray) const { return (pos_dist(ray.pos) / m_nrml.dot(ray.dir)); }

	private:
		t_vec3f m_nrml;
		t_vec3f m_vert;

		// orthogonal distance to origin along normal
		float m_dist;
	};


	struct t_mat33f {
	public:
		explicit t_mat33f(const t_vec3f& x = {1.0f, 0.0f, 0.0f}, const t_vec3f& y = {0.0f, 1.0f, 0.0f}, const t_vec3f& t = {0.0f, 0.0f, 1.0f}) {
			set_x(x);
			set_y(y);
			set_t(t);
		}

		t_mat33f& set_x(const t_vec3f& x) { m[0] = x.x(); m[1] = x.y(); m[2] = x.z(); return *this; }
		t_mat33f& set_y(const t_vec3f& y) { m[3] = y.x(); m[4] = y.y(); m[5] = y.z(); return *this; }
		t_mat33f& set_t(const t_vec3f& t) { m[6] = t.x(); m[7] = t.y(); m[8] = t.z(); return *this; }

		t_vec3f get_x() const { return {m[0], m[1], m[2]}; }
		t_vec3f get_y() const { return {m[3], m[4], m[5]}; }
		t_vec3f get_t() const { return {m[6], m[7], m[8]}; }

		//     R*T  = {{a, b, 0}; {c, d, 0}; {0, 0, 1}} * {{1, 0, x}; {0, 1, y}; {0, 0, 1}} = {{a, b, a*x + b*y}; {c, d, c*x + d*y}; {0, 0, 1}}
		//     T*R  = {{1, 0, x}; {0, 1, y}; {0, 0, 1}} * {{a, b, 0}; {c, d, 0}; {0, 0, 1}} = {{a, b, x}; {c, d, y}; {0, 0, 1}}
		// inv(R*T) = {{d / (a*d - b*c), b / (b*c - a*d),                      -x}; {c / (b*c - a*d), a / (a*d - b*c),                      -y}; {0, 0, 1}}
		// inv(T*R) = {{d / (a*d - b*c), b / (b*c - a*d), (d*x - b*y)/(b*c - a*d)}; {c / (b*c - a*d), a / (a*d - b*c), (c*x - a*y)/(a*d - b*c)}; {0, 0, 1}}
		t_mat33f invert() const {
			t_mat33f i = *this;
			t_vec3f t = get_t() * -1.0f;

			// transpose rotation
			std::swap(i.m[1], i.m[3]);

			// inverse translation
			i.set_t(i * t_vec3f{t.x(), t.y(), 0.0f} + t_vec3f{0.0f, 0.0f, 1.0f});
			return i;
		}
		t_mat33f transp() const {
			t_mat33f i = *this;

			std::swap(i.m[1], i.m[3]);
			std::swap(i.m[2], i.m[6]);
			std::swap(i.m[5], i.m[7]);
			return i;
		}

		// create the CCW rotation-matrix {{R11=cos(a), R12=-sin(a)}; {R21=sin(a), R22=cos(a)}}
		static t_mat33f compose_rot_xy(float a) { return (compose_rot_xy(std::sin(a), std::cos(a))); }
		static t_mat33f compose_rot_xy(float sa, float ca) {
			t_mat33f r;
			r.m[0] = ca; r.m[3] = -sa;
			r.m[1] = sa; r.m[4] =  ca;
			return r;
		}

		t_mat33f operator * (const t_mat33f& q) {
			t_mat33f r;

			r.m[0] = (m[0] * q.m[0]) + (m[3] * q.m[1]) + (m[6] * q.m[2]);
			r.m[1] = (m[1] * q.m[0]) + (m[4] * q.m[1]) + (m[7] * q.m[2]);
			r.m[2] = (m[2] * q.m[0]) + (m[5] * q.m[1]) + (m[8] * q.m[2]);

			r.m[3] = (m[0] * q.m[3]) + (m[3] * q.m[4]) + (m[6] * q.m[5]);
			r.m[4] = (m[1] * q.m[3]) + (m[4] * q.m[4]) + (m[7] * q.m[5]);
			r.m[5] = (m[2] * q.m[3]) + (m[5] * q.m[4]) + (m[8] * q.m[5]);

			r.m[6] = (m[0] * q.m[6]) + (m[3] * q.m[7]) + (m[6] * q.m[8]);
			r.m[7] = (m[1] * q.m[6]) + (m[4] * q.m[7]) + (m[7] * q.m[8]);
			r.m[8] = (m[2] * q.m[6]) + (m[5] * q.m[7]) + (m[8] * q.m[8]);
			return r;
		}

		t_vec3f operator * (const t_vec2f& v) const { return ((*this) * t_vec3f{v.x(), v.y(), 1.0f}); }
		t_vec3f operator * (const t_vec3f& v) const {
			t_vec3f p;
			p.x() = v.x() * m[0] + v.y() * m[3] + v.z() * m[6]; // dot(row[0], v)
			p.y() = v.x() * m[1] + v.y() * m[4] + v.z() * m[7]; // dot(row[1], v)
			p.z() = v.x() * m[2] + v.y() * m[5] + v.z() * m[8]; // dot(row[2], v)
			return p;
		}

	public:
		float m[3 * 3];
	};


	// represents either a bounding-volume (AA) or a line-segment
	template<typename type> struct t_pair {
		type mins;
		type maxs;
	};

	typedef t_pair<t_vec2f> t_vec2f2;
	typedef t_pair<t_vec3f> t_vec3f3;


	template<typename type> static inline type square(type v) { return (v * v); }
	template<typename type> static inline type lerp(type a, type b, float t) { return (a * (1.0f - t) + (b * t)); }
	template<typename type> static inline type clamp(type a, type mi, type ma) { return (std::min(std::max(a, mi), ma)); }
	template<typename type> static inline int8_t sign(type v) { return ((v > type(0)) - (v < type(0))); }


	// outer product
	static inline float cross_xy(const t_vec2f& v, const t_vec2f& w) { return (v.x() * w.y() - w.x() * v.y()); }
	static inline t_vec3f cross_xyz(const t_vec3f& v, const t_vec3f& w) {
		const float xc = cross_xy({v.y(), v.z()}, {w.y(), w.z()});
		const float yc = cross_xy({v.z(), v.x()}, {w.z(), w.x()});
		const float zc = cross_xy({v.x(), v.y()}, {w.x(), w.y()});
		return {xc, yc, zc};
	}

	#if 0
	static inline bool inside_rect(const t_vec2f& p, const t_vec2f2& r) {
		return ((p.x() >= r.mins.x() && p.x() <= r.maxs.x()) && (p.y() >= r.mins.y() && p.y() <= r.maxs.y()));
	}
	#endif

	// determine whether two number ranges overlap
	static inline bool range_intersect(const t_vec2f& a, const t_vec2f& b) {
		return ((a.x() <= b.y()) && (a.y() >= b.x()));
	}

	// determine whether two axis-aligned (2D) boxes intersect
	static bool rect_intersect(const t_vec2f2& a, const t_vec2f2& b) {
		const t_vec2f ax = {std::min(a.mins.x(), a.maxs.x()), std::max(a.mins.x(), a.maxs.x())};
		const t_vec2f bx = {std::min(b.mins.x(), b.maxs.x()), std::max(b.mins.x(), b.maxs.x())};
		const t_vec2f ay = {std::min(a.mins.y(), a.maxs.y()), std::max(a.mins.y(), a.maxs.y())};
		const t_vec2f by = {std::min(b.mins.y(), b.maxs.y()), std::max(b.mins.y(), b.maxs.y())};

		const bool xi = range_intersect(ax, bx);
		const bool yi = range_intersect(ay, by);

		assert(xi == range_intersect(bx, ax));
		assert(yi == range_intersect(by, ay));
		return (xi && yi);
	}

	// determine which side (left < 0, on == 0 or right > 0) of a line point p is on
	static inline float point_line_side(const t_vec2f& p, const t_vec2f2& l) { return ((cross_xy(l.maxs - l.mins, p - l.mins))); }
	static inline int8_t point_line_side_sign(const t_vec2f& p, const t_vec2f2& l) { return (sign(point_line_side(p, l))); }


	// calculate the intersection-point between two line-segments
	// an actual intersection implies this point lies on both segs
	static inline t_vec2f line_intersect_point(const t_vec2f2& l0, const t_vec2f2& l1) {
		const t_vec2f d0 = l0.mins - l0.maxs;
		const t_vec2f d1 = l1.mins - l1.maxs;

		const float a = cross_xy(l0.mins, l0.maxs);
		const float b = cross_xy(l1.mins, l1.maxs);
		const float c = cross_xy(d0, d1);

		const float x = cross_xy({a, d0.x()}, {b, d1.x()});
		const float y = cross_xy({a, d0.y()}, {b, d1.y()});

		// intersect(x1,y1, x2,y2,  x3,y3, x4,y4)
		//   a = vxs(x1,y1, x2,y2)
		//   b = vxs(x3,y3, x4,y4)
		//   c = vxs(x1-x2, y1-y2,  x3-x4, y3-y4)
		//   x = vxs(a, x1-x2,   b, x3-x4)  /  c
		//   y = vxs(a, y1-y2,   b, y3-y4)  /  c

		// parallel segments can never intersect unless they overlap
		assert(c != 0.0f);
		return {x / c, y / c};
	}

	static inline bool point_on_line(const t_vec2f& p, const t_vec2f2& l) {
		const t_vec2f v = p - l.mins;
		const t_vec2f e = l.maxs - l.mins;

		return (std::abs(point_line_side(p, l)) < 0.001f && v.dot(e) < e.dot(e));
	}

	static inline bool line_intersect(const t_vec2f2& a, const t_vec2f2& b) {
		const int8_t sa = std::abs(point_line_side_sign(b.mins, a) + point_line_side_sign(b.maxs, a));
		const int8_t sb = std::abs(point_line_side_sign(a.mins, b) + point_line_side_sign(a.maxs, b));
		return ((sa != 2 && sb != 2) && rect_intersect(a, b));
	}
};

#endif

