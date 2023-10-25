// Copyright 2023 watson.wang
// https://github.com/zzhawk/

#ifndef EGODYN_HPP_
#define EGODYN_HPP_

#include "gridmap.hpp"

namespace dyn
{
	class egoDynamics {
		public:
			explicit egoDynamics() {}

			virtual ~egoDynamics() = default;

			egoDynamics(const egoDynamics&) = default;
			egoDynamics(egoDynamics&&) noexcept = default;

			egoDynamics& operator = (const egoDynamics&) = default;
			egoDynamics& operator = (egoDynamics&&) noexcept = default;

			virtual gmp::pose move(gmp::pose p, double dist, double steer) = 0;
	};

	class simple : public egoDynamics {
		public:
			explicit simple() {}

			virtual ~simple() = default;

			simple(const simple&) = default;
			simple(simple&&) noexcept = default;

			simple& operator = (const simple&) = default;
			simple& operator = (simple&&) noexcept = default;

			gmp::pose move(gmp::pose p, double dist, double steer) override{
				double x = p.x + dist * cos(p.t);
				double y = p.y + dist * sin(p.t);
				double t = p.t + steer;

				return { x, y, t };
			}
	};
}
#endif