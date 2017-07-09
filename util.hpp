#ifndef PORTREND_UTIL_HDR
#define PORTREND_UTIL_HDR

#include <cassert>
#include <cstdio> // std::fprintf
#include <chrono>
#include <vector>

#include "consts.hpp"

namespace util {
	template<typename type> struct t_vector_queue {
	public:
		t_vector_queue() { clear(); }
		t_vector_queue(size_t n) {
			clear();
			reserve(n);
		}

		t_vector_queue(const t_vector_queue& q) { *this = q; }
		t_vector_queue(t_vector_queue&& q) { *this = std::move(q); }

		t_vector_queue& operator = (const t_vector_queue& q) {
			m_elems = q.elems();

			m_indcs.first  = q.head();
			m_indcs.second = q.tail();
			return *this;
		}
		t_vector_queue& operator = (t_vector_queue&& q) {
			m_elems = std::move(q.elems());

			m_indcs.first  = q.head();
			m_indcs.second = q.tail();

			q.clear();
			return *this;
		}

		// note: queue growth will normally be unbounded, be sure to call clear() when empty()
		type& push_back(const type& t) { m_elems.push_back(t); return m_elems[get_inc_tail()]; }
		type& pop_front() { assert(!empty()); return m_elems[get_inc_head()]; }

		const type& front() const { assert(!empty()); return m_elems[head()]; }
		const type&  back() const { assert(!empty()); return m_elems[tail()]; }
		      type& front()       { assert(!empty()); return m_elems[head()]; }
		      type&  back()       { assert(!empty()); return m_elems[tail()]; }

		bool empty() const { return (size() == 0); }
		bool valid() const { return (tail() >= head()); }

		size_t size() const { return (tail() - head()); }
		size_t head() const { return (m_indcs.first ); } // or (std::get<0>(m_indcs))
		size_t tail() const { return (m_indcs.second); } // or (std::get<1>(m_indcs))

		void reserve(size_t n) { m_elems.reserve(n); }
		void clear() {
			m_elems.clear();

			m_indcs.first  = 0;
			m_indcs.second = 0;

			assert(empty());
		}

	private:
		const std::vector<type>& elems() const { return m_elems; }
		      std::vector<type>& elems()       { return m_elems; }

		size_t get_inc_head() { return (m_indcs.first ++); }
		size_t get_inc_tail() { return (m_indcs.second++); }

	private:
		// .first=head, .second=tail
		std::pair<size_t, size_t> m_indcs;
		std::vector<type> m_elems;
	};


	struct t_system_timer {
	public:
		uint64_t tick_time() { return (tick(),           0); }
		uint64_t tock_time() { return (tock(), diff_time()); }
		uint64_t diff_time() const { return ((nsecs()).count()); }

	private:
		std::chrono::high_resolution_clock::time_point& tick() { return (t0 = std::chrono::high_resolution_clock::now()); }
		std::chrono::high_resolution_clock::time_point& tock() { return (t1 = std::chrono::high_resolution_clock::now()); }
		std::chrono::nanoseconds nsecs() const { return (t1 - t0); }

	private:
		std::chrono::high_resolution_clock::time_point t0;
		std::chrono::high_resolution_clock::time_point t1;
	};


	struct t_wall_clock {
	public:
		bool update(uint64_t tock_time) {
			render_time_ns += (tock_time * render_dt_mult);
			system_time_ns += (tock_time                 );
			n_render_calls += 1;

			return (system_time_ns >= consts::WALL_SEC_TIME_NS);
		}

		void inv_render_dt_mult() { render_dt_mult = 1 - render_dt_mult; }
		void add_render_time_ns(uint64_t dt) { render_time_ns += dt; }
		void add_update_time_ns(uint64_t dt) { update_time_ns += dt; }
		void add_system_time_ns(uint64_t dt) { system_time_ns += dt; }

		void output_timings(FILE* out) {
			std::fprintf(out, "[wc::%s] {update,render}_calls={%u,%u}\n", __func__, n_update_calls, n_render_calls);
		}

		uint32_t add_update_call() { return (n_update_calls++); }

		uint64_t get_render_time_ns() const { return render_time_ns; }
		uint64_t get_system_time_ns() const { return system_time_ns; }

	private:
		uint32_t n_update_calls = 0; // total number of Update calls executed
		uint32_t n_render_calls = 0; // total number of Render calls executed

		uint32_t render_dt_mult = 1; // if 0, renderer produces no time for simulation

		uint64_t system_time_ns = 0; // total time (ns) spent in Update+Render calls
		uint64_t update_time_ns = 0; // total time (ns) spent in Update calls
		uint64_t render_time_ns = 0; // total time (ns) spent in Render calls
	};
};

#endif
