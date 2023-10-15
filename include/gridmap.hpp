
namespace gmp
{
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

	struct occMap
	{
		double width{};
		double length{};
		double resolution = 1.0;
		double sx{};
		double sy{};
		double gx{};
		double gy{};

		std::vector<std::vector<grid>> grids;
	};
}