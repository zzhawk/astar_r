#include <vector>
#include <queue>
#include "gridmap.hpp"

namespace pl
{
	struct pathExt {
		double x{};
		double y{};
	};

	struct grid {

		explicit grid(int x, int y, double cost, bool obs) :
			x(x), y(y), gc(cost), obs(obs) {};
		virtual ~grid() = default;

		grid(const grid&) = default;
		grid(grid&&) noexcept = default;

		grid& operator = (const grid&) = default;
		grid& operator = (grid&&) noexcept = default;

		int x{};
		int y{};

		bool obs = false;
		bool visited = false;
		double hc{};
		double gc{};
		grid* parent = nullptr;

		double cost() const { return gc + hc; }
	};

	struct motion {

		explicit motion(int x, int y, double cost) :
			x(x), y(y), cost(cost) {};
		virtual ~motion() = default;

		motion(const motion&) = default;
		motion(motion&&) noexcept = default;

		motion& operator = (const motion&) = default;
		motion& operator = (motion&&) noexcept = default;

		int x{};
		int y{};
		double cost{};
	};

	struct NodeComparison
	{
		bool operator()(const grid* lhs, const grid* rhs)
		{
			return lhs->cost() > rhs->cost();
		}
	};


	using motions = std::vector<motion>;

	class astar {
	public:
		explicit astar(gmp::occMap& map)
		{
			mapTransformer(map);
			setMotionModel();
		};

		virtual ~astar() = default;

		astar(const astar&) = default;
		astar(astar&&) noexcept = default;

		astar& operator = (const astar&) = default;
		astar& operator = (astar&&) noexcept = default;

		void planning();
		std::vector<pathExt> &getPath(void);
		std::vector<pathExt> debug_getSearched(void);

	private:
		std::priority_queue<grid*, std::vector<grid*>, NodeComparison> _openList;
		std::vector<pathExt> _path;
		std::vector<std::vector<grid>> _grids;
		motions _mots;
		double _w_hc = 1.0;
		grid* _start = nullptr;
		grid* _goal = nullptr;
		double _res = 1.0;

		void setMotionModel(void);
		void mapTransformer(gmp::occMap& map);
		void genFinalPath();
		double calHeuristic(const grid* n1, const grid* n2);
	};
}