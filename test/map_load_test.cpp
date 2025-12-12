/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "adore_map/lane.hpp"
#include "adore_map/map.hpp"
#include "adore_map/map_loader.hpp"
#include "adore_map/road_graph.hpp"

#ifndef ADORE_MAP_TEST_DATA_DIR
  // Fallback – will be overridden from CMake for real tests.
  #define ADORE_MAP_TEST_DATA_DIR "."
#endif

namespace
{
std::string
get_test_map_r2s_path()
{
  return std::string( ADORE_MAP_TEST_DATA_DIR ) + "/test_map.r2sr";
}
} // namespace

// Basic smoke test: we can load the map and it has roads, lanes and a lane graph.
TEST( MapTest, load_map_has_roads_lanes_and_graph )
{
  const std::string map_file = get_test_map_r2s_path();

  // This will also pick up the matching .r2l file with the same basename.
  adore::map::Map map = adore::map::MapLoader::load_from_file( map_file, false );

  // Structural checks
  EXPECT_FALSE( map.roads.empty() ) << "Expected at least one road in the loaded map";
  EXPECT_FALSE( map.lanes.empty() ) << "Expected at least one lane in the loaded map";

  EXPECT_FALSE( map.lane_graph.to_successors.empty() ) << "Lane graph should have successors";
  EXPECT_FALSE( map.lane_graph.to_predecessors.empty() ) << "Lane graph should have predecessors";
  EXPECT_FALSE( map.lane_graph.all_connections.empty() ) << "Lane graph should contain connections";

  // Take the first lane and do some sanity checks.
  auto lane_it = map.lanes.begin();
  ASSERT_NE( lane_it, map.lanes.end() );
  const size_t                             lane_id = lane_it->first;
  const std::shared_ptr<adore::map::Lane>& lane    = lane_it->second;
  ASSERT_TRUE( lane );

  EXPECT_GT( lane->length, 0.0 ) << "Lane length should be > 0";

  // get_lane_speed_limit must return a positive speed for a valid lane.
  const double speed_limit = map.get_lane_speed_limit( lane_id );
  EXPECT_GT( speed_limit, 0.0 );

  // The lane's road_id must refer to an existing road.
  auto road_it = map.roads.find( lane->road_id );
  ASSERT_NE( road_it, map.roads.end() ) << "Lane's road_id does not exist in map.roads";
}

// Check that all lane-graph connections refer to lanes that actually exist in the map.
TEST( MapTest, lane_graph_connections_reference_existing_lanes )
{
  const std::string map_file = get_test_map_r2s_path();
  adore::map::Map   map      = adore::map::MapLoader::load_from_file( map_file, false );

  for( const auto& connection : map.lane_graph.all_connections )
  {
    EXPECT_TRUE( map.lanes.count( connection.from_id ) > 0 ) << "Missing lane for connection.from_id " << connection.from_id;
    EXPECT_TRUE( map.lanes.count( connection.to_id ) > 0 ) << "Missing lane for connection.to_id " << connection.to_id;
  }
}

// Ensure that the quadtree actually contains points from at least one loaded lane.
TEST( MapTest, quadtree_contains_points_from_lanes )
{
  const std::string map_file = get_test_map_r2s_path();
  adore::map::Map   map      = adore::map::MapLoader::load_from_file( map_file, false );

  // Pick any lane.
  auto lane_it = map.lanes.begin();
  ASSERT_NE( lane_it, map.lanes.end() );
  const std::shared_ptr<adore::map::Lane>& lane = lane_it->second;
  ASSERT_TRUE( lane );

  // Use one of its center-line points as a query center.
  const auto& center_points = lane->borders.center.interpolated_points;
  ASSERT_FALSE( center_points.empty() ) << "Expected interpolated center points in lane borders";

  const auto& query_point = center_points.front();

  std::vector<adore::map::MapPoint> found;
  map.quadtree.query_range( query_point, 1.0, found );

  EXPECT_FALSE( found.empty() ) << "Quadtree query around a lane center point returned no points";
}
