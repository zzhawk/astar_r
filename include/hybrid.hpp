// Copyright 2023 watson.wang
// https://github.com/zzhawk/

#ifndef HYBRID_HPP_
#define HYBRID_HPP_

#include <memory>
#include <vector>
#include <queue>
#include <corecrt_math_defines.h>
#include "astar.hpp"
#include "egodyn.hpp"

namespace pl
{

	struct config {
		bool USE_BACK = true;
		double MIN_TURN_RADIUS = 5.0;
		double MAX_TURN_RADIUS = 10.0;
		int THETA_SIZE = 144;
		int TURN_RADIUS_SIZE = 5;

		double SB_COST = 100.0;
		double BACK_COST = 5.0;
		double STEER_CHANGE_COST = 5.0;
		double STEER_COST = 1.0;
		double H_COST = 5.0;
	};

	class hybrid :public astar {
	public:
		explicit hybrid(gmp::occMap& map, config cfg): astar(map, cfg.H_COST), _cfg(cfg)
		{
			_mots = setMotionModel(); /* ?? */
		}

		virtual ~hybrid() = default;

		hybrid(const hybrid&) = default;
		hybrid(hybrid&&) noexcept = default;

		hybrid& operator = (const hybrid&) = default;
		hybrid& operator = (hybrid&&) noexcept = default;

	private:
		config _cfg;
		double _path_res = 1.0;
		std::unique_ptr<dyn::egoDynamics> _dynamic = std::make_unique<dyn::simple>();

		motions setMotionModel(void) override;
		double calMoveCost(const node* n1, const node* n2, motion& mot);


		double normalizeRadian(const double rad, const double min_rad = -M_PI, const double max_rad = M_PI);
		bool hitObstacle(node* cur, index& idx, motion& mot);
		int discretizeAngle(const double theta) override;
		index pose2index(gmp::pose p) override;
		gmp::pose index2pose(index idx) override;
		void fillData(node* nd, index& idx, node* cur, motion& mot) override;
		bool isGoal(node* cur, index& idx) override;
		index getNewIdx(node* cur, motion& mot) override;
		double calHeuristic(const node* n1) override;
		void genFinalPath(void) override;

		double calcReedsSheppDistance(const gmp::pose& p1, const gmp::pose& p2, double radius);
		std::vector<gmp::pose> calcReedsSheppPath(const gmp::pose& p1, const gmp::pose& p2, double radius);
	};
}

#endif