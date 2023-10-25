// Copyright 2023 watson.wang
// https://github.com/zzhawk/

#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include "gridmap.hpp"

namespace pl
{
	struct index {
		int x{};
		int y{};
		int t{};
	};

	enum class NodeStatus : uint8_t { None, Open, Closed };
	struct node
	{
		NodeStatus status = NodeStatus::None;
		gmp::pose p;
		double gc = 0;
		double hc = 0;
		bool is_back = false;
		std::vector<gmp::pose> path;
		node* parent = nullptr;

		double cost() const { return gc + hc; }
	};


	struct motion {

		double shift_x{};
		double shift_y{};
		double shift_theta{};
		double distance{};
		double radius{};
		bool is_curve{};
		bool is_back{};

		motion rotated(const double theta) const
		{
			motion result = *this;
			result.shift_x = std::cos(theta) * this->shift_x - std::sin(theta) * this->shift_y;
			result.shift_y = std::sin(theta) * this->shift_x + std::cos(theta) * this->shift_y;
			return result;
		}

		motion flipped() const
		{
			motion result = *this;
			result.shift_y = -result.shift_y;
			result.shift_theta = -result.shift_theta;
			return result;
		}

		motion reversed() const
		{
			motion result = *this;
			result.shift_x = -result.shift_x;
			result.shift_theta = -result.shift_theta;
			result.is_back = !result.is_back;
			return result;
		}
	};

	struct NodeComparison
	{
		bool operator()(const node* lhs, const node* rhs)
		{
			return lhs->cost() > rhs->cost();
		}
	};


	using motions = std::vector<std::vector<motion>>;

	class astar {
		public:
			explicit astar(gmp::occMap& map, double w_hc): _map(map), _w_hc(w_hc)
			{
				x_scale_ = (int)(map.width / map.resolution);
				y_scale_ = (int)(map.length / map.resolution);

				_mots = setMotionModel();
			};

			virtual ~astar() = default;

			astar(const astar&) = default;
			astar(astar&&) noexcept = default;

			astar& operator = (const astar&) = default;
			astar& operator = (astar&&) noexcept = default;

			void planning();
			std::vector<gmp::pose> &getPath(void);
			std::vector<gmp::pose> debug_getSearched(void);

		protected:
			double _motionRes = 0.1;

			gmp::occMap& _map;

			std::priority_queue<node*, std::vector<node*>, NodeComparison> _openList;
			std::vector<gmp::pose> _path;
			std::unordered_map<int, node> _graph;

			motions _mots;
			double _w_hc = 1.0;
			node* _start = nullptr;
			node* _goal = nullptr;
		
			int x_scale_;
			int y_scale_;

			virtual motions setMotionModel(void);
			virtual int discretizeAngle(const double theta);
			virtual bool hitObstacle(node* cur, index &idx, motion& mot);
			virtual index getNewIdx(node* cur, motion& mot);
			virtual index pose2index(gmp::pose p);
			virtual gmp::pose index2pose(index idx);
			virtual void fillData(node* nd, index& idx, node* cur, motion &mot);
			virtual bool isGoal(node* cur, index& idx);
			virtual double calHeuristic(const node* n1);
			virtual void genFinalPath();

			void setStartNode(void);
			void setGoalNode(void);

			node* getNodeRef(const index& idx);
			inline int getKey(const index& idx);
			inline int getIdx(double d);
			gmp::grid getGrid(double x, double y);
	};
}

#endif