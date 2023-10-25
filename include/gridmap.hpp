// Copyright 2023 watson.wang
// https://github.com/zzhawk/

#ifndef GRIDMAP_HPP_
#define GRIDMAP_HPP_

namespace gmp
{
	struct pose {
		double x{};
		double y{};
		double t{};
	};

	struct grid {

		explicit grid(double x, double y, double cost, bool obs) :
			x(x), y(y), cost(cost), obs(obs) {};
		virtual ~grid() = default;

		grid(const grid&) = default;
		grid(grid&&) noexcept = default;

		grid& operator = (const grid&) = default;
		grid& operator = (grid&&) noexcept = default;

		double x{};
		double y{};
		double cost{};
		bool obs = false;
	};

	struct occMap{

		double width{};
		double length{};
		double resolution = 1.0;
		pose s{};
		pose g{};

		std::vector<std::vector<grid>> grids;
	};
}

#endif