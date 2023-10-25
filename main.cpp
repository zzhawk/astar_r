#include <fstream>
#include "astar.hpp"
#include "hybrid.hpp"
#include "map/libbmp.h"

int main()
{

   std::ofstream mpf("../map.csv");
   std::ofstream plf("../plan.csv");
   std::ofstream srf("../search.csv");

   gmp::occMap themap;

   BmpImg img;

   img.read("map/simple.bmp");

   const int width = img.get_width();
   const int height = img.get_height();

   themap.resolution = 1.0; 
   themap.s.x = 10.0 * themap.resolution;
   themap.s.y = 10.0 * themap.resolution;
   themap.s.t = M_PI_2;
   themap.g.x = 50.0 * themap.resolution;
   themap.g.y = 50.0 * themap.resolution;
   themap.g.t = M_PI_2;

   themap.length = width * themap.resolution;
   themap.width = height * themap.resolution;

   // Count the amount of black and white pixels:
   // Use negative loops to improve performance
   for (int y = 0; y < themap.length; ++y){
       std::vector<gmp::grid> buf;
       for (int x = 0; x < themap.width; ++x){
           int ox = x;
           int oy = height - static_cast<int>(static_cast<double>(y) / themap.resolution) - 1;
           if (img.red_at(ox, oy) < 128 &&
               img.green_at(ox, oy) < 128 &&
               img.blue_at(ox, oy) < 128){
               gmp::grid g(x, y, 0.0, true);
               buf.push_back(g);
           }
           else{
               gmp::grid g(x, y, 0.0, false);
               buf.push_back(g);
           }
       }
       themap.grids.push_back(buf);
   }

   for (int i = 0; i < themap.grids.size(); ++i) {
       for (int j = 0; j < themap.grids[0].size(); ++j) {
           mpf << themap.grids[j][i].x << "," << themap.grids[j][i].y << "," 
               << themap.grids[j][i].cost << "," << themap.grids[j][i].obs 
               << std::endl;
       }
   }

   pl::config cfg;
   cfg.USE_BACK = true;
   cfg.MIN_TURN_RADIUS = 5.0;
   cfg.MAX_TURN_RADIUS = 10.0;
   cfg.THETA_SIZE = 144;
   cfg.TURN_RADIUS_SIZE = 5;
   cfg.SB_COST = 100.0;
   cfg.BACK_COST = 5.0;
   cfg.STEER_CHANGE_COST = 5.0;
   cfg.STEER_COST = 1.0;
   cfg.H_COST = 5.0;

   //std::unique_ptr<pl::astar> astar_plan = std::make_unique<pl::astar>(themap, 1.0);
   std::unique_ptr<pl::astar> astar_plan = std::make_unique<pl::hybrid>(themap, cfg);

   astar_plan->planning();

   auto path = astar_plan->getPath();
   for (int i = 0; i < path.size(); ++i) {
       plf << path[i].x << "," << path[i].y << std::endl;
   }

   auto searched = astar_plan->debug_getSearched();
   for (int i = 0; i < searched.size(); ++i) {
       srf << searched[i].x << "," << searched[i].y << std::endl;
   }

   return 0;
}